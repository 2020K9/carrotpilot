#!/usr/bin/env python3
"""Download the light speed ONNX and cut it down for the built-in (C4) GPU, without retraining.

  python3 lightspeed_c4_fetch.py   # rebuilds models/lightspeed_c4_direct.onnx (needs the onnx package; dev machine only)

The reduced ONNX is committed (C4 has no onnx package and should not download 790 MB), and scons compiles it to
lightspeed_c4_direct_driving_tinygrad.pkl like the stock model. A failed compile never fails the build. modeld uses
it only when selected (see helpers.lightspeed_c4_direct_selected) and falls back to the stock model if it can't load.

Reduction (100.2 -> 7.2 GFLOP by ONNX count):
  - vision ConvNeXt: keep the first block of each stage (stage 2: 1 of 27), skip the rest through their residuals
  - temporal transformers (on/off policy): skip all layers through their residuals
  - every remaining GELU MLP keeps its strongest hidden units (|W1 col|*|W2 row|): 1024->4096 MLPs to 1024, others 1/8
No retraining, so the outputs move a lot (CPU, 80 frames vs the full model: 5 s path lateral 0.54 m mean / 2.4 m max,
speed 4.7 m/s mean).
"""
import argparse
import hashlib
import json
import os
import urllib.request
from pathlib import Path

import numpy as np

from openpilot.selfdrive.modeld.helpers import LIGHTSPEED_C4_ONNX

CATALOG = Path(__file__).resolve().parent / 'model_catalog' / 'catalog.json'
KEEP = {'s0': 1, 's1': 1, 's2': 1, 's3': 1, 't': 0}
WIDE_MLP_WIDTH = 1024
OTHER_MLP_RATIO = 0.125


def sha256(path):
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    for b in iter(lambda: f.read(1 << 20), b''):
      h.update(b)
  return h.hexdigest()


def drop_blocks(nodes, keep):
  res_in = {}  # (group, block) -> residual-stream inputs of the block's branches
  for n in nodes:
    if n.op_type == 'Conv' and len(n.input) > 1 and '.stages.' in n.input[1] and 'conv_dw' in n.input[1]:
      st = n.input[1].split('.stages.')[1].split('.')[0]
      j = int(n.input[1].split('.blocks.')[1].split('.')[0])
      res_in.setdefault(('s' + st, j), []).append(n.input[0])
    if n.op_type == 'LayerNormalization' and '.transformer.layers.' in n.input[1] and \
       ('.attention.norm.' in n.input[1] or '.mlp.norm.' in n.input[1]):
      grp = n.input[1].split('.transformer.layers.')[0]
      j = int(n.input[1].split('.layers.')[1].split('.')[0])
      res_in.setdefault(('t:' + grp, j), []).append(n.input[0])
  groups = {}
  for g, j in res_in:
    groups.setdefault(g, []).append(j)
  drops = []
  for g, js in groups.items():
    k = keep.get('t' if g.startswith('t:') else g)
    if k is None:
      continue
    js = sorted(js)
    kept = {js[i] for i in np.round(np.linspace(0, len(js) - 1, k)).astype(int)} if k > 0 else set()
    drops += [(g, j) for j in js if j not in kept]
  xs = {x for d in drops for x in res_in[d]}
  ren = {}
  for n in nodes:
    if n.op_type == 'Add' and len(n.input) == 2 and n.input[0] != n.input[1]:
      for x in n.input:
        if x in xs:
          ren[n.output[0]] = x

  def resolve(t):
    while t in ren:
      t = ren[t]
    return t
  for n in nodes:
    for k, i in enumerate(n.input):
      if i in ren:
        n.input[k] = resolve(i)
  return len(drops)


def narrow_mlps(model, nodes):
  import onnx
  from onnx import numpy_helper as nh
  inits = {t.name: t for t in model.graph.initializer}
  prod = {o: n for n in nodes for o in n.output}
  cons = {}
  for n in nodes:
    for i in n.input:
      cons.setdefault(i, []).append(n)
  done, orig, cnt = set(), {}, 0
  for g in nodes:
    if g.op_type != 'Gelu':
      continue
    a = prod.get(g.input[0])
    c = cons.get(g.output[0], [])
    if a is None or a.op_type != 'Add' or len(c) != 1 or c[0].op_type != 'MatMul':
      continue
    bname = [i for i in a.input if i in inits]
    mm1 = [prod.get(i) for i in a.input if i not in inits]
    if len(bname) != 1 or not mm1 or mm1[0] is None or mm1[0].op_type != 'MatMul':
      continue
    mm1, mm2 = mm1[0], c[0]
    if mm1.input[1] not in inits or mm2.input[1] not in inits:
      continue
    trio = (mm1.input[1], bname[0], mm2.input[1])
    if all(x in done for x in trio):
      continue
    # an initializer already narrowed for another MLP: give this MLP its own copy of the original
    for node, slot, x in ((mm1, 1, trio[0]), (a, list(a.input).index(bname[0]), trio[1]), (mm2, 1, trio[2])):
      if x in done:
        t = onnx.TensorProto()
        t.CopyFrom(orig[x])
        t.name = f'{x}_c{cnt}'
        model.graph.initializer.append(t)
        inits[t.name] = model.graph.initializer[-1]
        node.input[slot] = t.name
    bname = [nm for nm in a.input if nm in inits]
    done.update((mm1.input[1], bname[0], mm2.input[1]))
    w1, b1, w2 = (nh.to_array(inits[x]) for x in (mm1.input[1], bname[0], mm2.input[1]))
    h = w1.shape[-1]
    k = WIDE_MLP_WIDTH if (h == 4096 and w1.shape[0] == 1024) else max(8, int(h * OTHER_MLP_RATIO))
    if k >= h:
      continue
    score = np.linalg.norm(w1.astype(np.float32), axis=0) * np.linalg.norm(w2.astype(np.float32), axis=1)
    idx = np.sort(np.argsort(-score)[:k])
    for x in (mm1.input[1], bname[0], mm2.input[1]):
      if x not in orig:
        orig[x] = onnx.TensorProto()
        orig[x].CopyFrom(inits[x])
    for name, arr in ((mm1.input[1], w1[:, idx]), (bname[0], b1[idx]), (mm2.input[1], w2[idx])):
      inits[name].CopyFrom(nh.from_array(np.ascontiguousarray(arr), name))
    cnt += 1
  return cnt


def shrink(src, dst):
  import onnx
  from onnx.utils import Extractor
  model = onnx.load(str(src))
  ins = [i.name for i in model.graph.input]
  outs = [o.name for o in model.graph.output]
  nodes = list(model.graph.node)
  dropped = drop_blocks(nodes, KEEP)
  narrowed = narrow_mlps(model, nodes)
  del model.graph.value_info[:]
  out = Extractor(model).extract_model(ins, outs)
  # extract_model drops the model metadata, but modeld/compile_modeld need output_slices from it
  del out.metadata_props[:]
  out.metadata_props.extend(model.metadata_props)
  assert 'output_slices' in {m.key for m in out.metadata_props}, 'source onnx has no output_slices metadata'
  dst = Path(dst)
  tmp = dst.with_name(dst.name + '.tmp')
  try:
    onnx.save(out, str(tmp))
    os.replace(tmp, dst)
  finally:
    tmp.unlink(missing_ok=True)
  print(f'dropped {dropped} blocks/layers, narrowed {narrowed} MLPs')


def download(url, dst):
  tmp = dst.with_name(dst.name + '.part')
  try:
    urllib.request.urlretrieve(url, tmp)
    os.replace(tmp, dst)
  finally:
    tmp.unlink(missing_ok=True)


def main():
  entry = json.loads(CATALOG.read_text())['light-speed']['c4_builtin']
  ap = argparse.ArgumentParser()
  ap.add_argument('--src', type=Path, help='already downloaded full light speed onnx')
  ap.add_argument('--out', type=Path, default=LIGHTSPEED_C4_ONNX)
  args = ap.parse_args()

  src = args.src or args.out.with_name('lightspeed_c4.src.onnx')
  try:
    if not src.is_file() or sha256(src) != entry['sha256']:
      print('downloading', entry['url'])
      download(entry['url'], src)
    assert sha256(src) == entry['sha256'], 'sha256 mismatch'
    shrink(src, args.out)
    got = sha256(args.out)
    # the cut is deterministic: the same source must give the committed file
    if got != entry['reduced_sha256']:
      args.out.unlink(missing_ok=True)
      raise AssertionError(f'reduced sha256 {got} != catalog {entry["reduced_sha256"]}')
  finally:
    if args.src is None:
      src.unlink(missing_ok=True)
  print('wrote', args.out)


if __name__ == '__main__':
  main()
