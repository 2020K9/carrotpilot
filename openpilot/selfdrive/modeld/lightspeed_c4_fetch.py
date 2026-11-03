#!/usr/bin/env python3
"""Download the light speed ONNX for the built-in (C4) GPU and optionally narrow its vision MLPs.

  python3 lightspeed_c4_fetch.py [--mlp-width 4096|3072|2048]
  BUILD_LIGHTSPEED_C4=1 scons openpilot/selfdrive/modeld   # compiles lightspeed_c4_driving_tinygrad.pkl

--mlp-width < 4096 keeps the strongest hidden units of the 27 stage-2 (8x16 grid) 1024->4096->1024 MLPs,
without retraining. Measured on CPU against the full model it changes the 5 s path a lot (0.62 m / 0.82 m mean
lateral error for 3072 / 2048), so the default stays 4096. Delete models/lightspeed_c4* to go back to stock.
"""
import argparse
import hashlib
import json
import urllib.request
from pathlib import Path

import numpy as np

from openpilot.selfdrive.modeld.helpers import LIGHTSPEED_C4_ONNX

CATALOG = Path(__file__).resolve().parent / 'model_catalog' / 'catalog.json'


def sha256(path):
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    for b in iter(lambda: f.read(1 << 20), b''):
      h.update(b)
  return h.hexdigest()


def narrow_mlps(model, keep):
  from onnx import numpy_helper as nh
  inits = {t.name: t for t in model.graph.initializer}
  nodes = list(model.graph.node)
  count = 0
  for k, n in enumerate(nodes):
    if n.op_type != 'LayerNormalization' or '.stages.2.blocks.' not in n.input[1]:
      continue
    fc1, bias1, gelu, fc2 = nodes[k + 1:k + 5]
    assert (fc1.op_type, bias1.op_type, gelu.op_type, fc2.op_type) == ('MatMul', 'Add', 'Gelu', 'MatMul')
    w1, b1, w2 = (nh.to_array(inits[x]) for x in (fc1.input[1], bias1.input[1], fc2.input[1]))
    score = np.linalg.norm(w1.astype(np.float32), axis=0) * np.linalg.norm(w2.astype(np.float32), axis=1)
    idx = np.sort(np.argsort(-score)[:keep])
    for name, arr in ((fc1.input[1], w1[:, idx]), (bias1.input[1], b1[idx]), (fc2.input[1], w2[idx])):
      inits[name].CopyFrom(nh.from_array(np.ascontiguousarray(arr), name))
    count += 1
  assert count == 27, count
  del model.graph.value_info[:]


def main():
  entry = json.loads(CATALOG.read_text())['light-speed']['c4_builtin']
  ap = argparse.ArgumentParser()
  ap.add_argument('--mlp-width', type=int, default=entry['mlp_width'], choices=(4096, 3072, 2048))
  args = ap.parse_args()

  src = LIGHTSPEED_C4_ONNX.with_suffix('.src.onnx')
  if not src.is_file() or sha256(src) != entry['sha256']:
    print('downloading', entry['url'])
    urllib.request.urlretrieve(entry['url'], src)
  assert sha256(src) == entry['sha256'], 'sha256 mismatch'

  if args.mlp_width == 4096:
    src.replace(LIGHTSPEED_C4_ONNX)
  else:
    import onnx
    model = onnx.load(str(src))
    narrow_mlps(model, args.mlp_width)
    onnx.save(model, str(LIGHTSPEED_C4_ONNX))
    src.unlink()
  print('wrote', LIGHTSPEED_C4_ONNX, 'mlp width', args.mlp_width)


if __name__ == '__main__':
  main()
