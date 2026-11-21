#!/usr/bin/env python3
"""Read K9 NOT_FINITE tracer events (NF_TRACE.md) and print what they say. Worker/PC only; no Jetson needed.

  python3 tools/jetlink/nf_trace_analyze.py <dir with event folders or one event folder> [--onnx model.onnx] [--json]

Per event: the JL_NF line, the Jetson replay verdict (nan_again = same input+state NaN again = input/model numerics;
clean = transient GPU/power/kernel), tensor stats (non-finite, max |x| vs fp16 65504), the previous normal frames'
output peaks, Jetson health 60 s before / after (GPU clock, rails, temperatures, nvpmodel, cooling), dmesg hits.
--onnx: also run the captured inputs+state through onnxruntime in fp32 on this machine (finite in fp32 but NaN on
the Jetson points to fp16 overflow in the TensorRT engine).
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

FP16_MAX = 65504.


def load_event(d):
  d = Path(d)
  meta = json.loads((d / 'meta.json').read_text())
  rows = {}
  for name in ('health_pre', 'health_post'):
    p = d / f'{name}.jsonl'
    rows[name] = [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []
  texts = {n: (d / f'{n}.txt').read_text(errors='replace') for n in ('dmesg_at_error', 'dmesg_after', 'prev_boot_kernel')
           if (d / f'{n}.txt').exists()}
  tensors = None
  if (d / 'tensors.npz').exists():
    with np.load(d / 'tensors.npz') as z:
      tensors = {k: z[k] for k in z.files}
  return meta, rows, texts, tensors


def _num(values):
  values = [v for v in values if isinstance(v, (int, float))]
  return (min(values), max(values)) if values else None


def health_summary(rows):
  out = {}
  if not rows:
    return out
  clocks = []
  for r in rows:
    for g in (r.get('gpu') or {}).values():
      clocks.append(g.get('clock_mhz'))
  out['gpu_mhz'] = _num(clocks)
  rails = {}
  for r in rows:
    for rail in r.get('power_rails') or []:
      rails.setdefault(rail.get('name'), {'mv': [], 'ma': []})
      rails[rail.get('name')]['mv'].append(rail.get('mv'))
      rails[rail.get('name')]['ma'].append(rail.get('ma'))
  out['rails'] = {k: {'mv': _num(v['mv']), 'ma': _num(v['ma'])} for k, v in rails.items()}
  temps = {}
  for r in rows:
    for k, v in (r.get('temp_c') or {}).items():
      temps.setdefault(k, []).append(v)
  out['temp_c_max'] = {k: max(v) for k, v in temps.items()}
  out['power_modes'] = sorted({json.dumps((r.get('power_mode') or {}).get('name')) for r in rows})
  out['cooling_active'] = sorted({f"{c.get('name')}={c.get('state')}" for r in rows for c in r.get('cooling') or []})
  out['samples'] = len(rows)
  out['gaps_s'] = max((b['mono'] - a['mono'] for a, b in zip(rows, rows[1:], strict=False)), default=0)
  return out


def onnx_check(path, tensors):
  import onnxruntime as ort
  s = ort.InferenceSession(str(path), providers=['CPUExecutionProvider'])
  feed = {}
  for i in s.get_inputs():
    key = f'inputs__{i.name}' if f'inputs__{i.name}' in tensors else f'state__{i.name}'
    if key not in tensors:
      return {'error': f'missing {i.name}'}
    want = np.float16 if 'float16' in i.type else np.float32
    feed[i.name] = tensors[key].astype(want).reshape([d if isinstance(d, int) else -1 for d in i.shape])
  out = s.run(None, feed)
  return {o.name: {'nonfinite': int(v.size - np.isfinite(v).sum()), 'max_abs': float(np.nanmax(np.abs(v)))}
          for o, v in zip(s.get_outputs(), out, strict=True)}


def report(d, onnx=None):
  meta, rows, texts, tensors = load_event(d)
  r = {'id': meta.get('id'), 'frame': meta.get('frame'), 'model': str(meta.get('model'))[:12], 'boot': meta.get('boot'),
       'detail': meta.get('detail'), 'replay': meta.get('replay'), 'interrupted': meta.get('interrupted'),
       'post_complete': meta.get('post_complete'), 'state_source': meta.get('state_source')}
  stats = meta.get('stats') or {}
  r['nonfinite_tensors'] = {k: v['nonfinite'] for k, v in stats.items() if v.get('nonfinite')}
  r['near_fp16_limit'] = {k: v['max_abs'] for k, v in stats.items()
                          if v.get('max_abs') is not None and v['max_abs'] > FP16_MAX / 8}
  r['input_nonfinite'] = any(k.startswith(('inputs.', 'state.')) for k in r['nonfinite_tensors'])
  r['prev_outputs_max_abs'] = [(p['frame'], (p.get('outputs') or {}).get('max_abs')) for p in meta.get('prev') or []]
  r['health_pre'] = health_summary(rows['health_pre'])
  r['health_post'] = health_summary(rows['health_post'])
  r['health_post_last_wall'] = rows['health_post'][-1]['wall'] if rows['health_post'] else None
  for name, text in texts.items():
    hits = [x for x in text.splitlines() if x and not x.startswith('#')]
    r[name] = hits[-15:]
  if onnx and tensors:
    try:
      r['onnx_fp32'] = onnx_check(onnx, tensors)
    except Exception as exc:
      r['onnx_fp32'] = {'error': f'{type(exc).__name__}: {exc}'}
  v = (meta.get('replay') or {}).get('verdict')
  r['reading'] = {
    'nan_again': 'Jetson 재실행에서도 NaN: 입력/모델 수치 문제(fp16 넘침 등). onnx_fp32가 유한이면 TensorRT fp16 정밀도 문제',
    'clean': 'Jetson 재실행은 정상: GPU 클럭·전원·커널·메모리 쪽 순간 문제(health/dmesg 확인)',
    'mixed': '재실행 결과가 들쭉날쭉: GPU/전원 불안정 의심',
  }.get(v, f'재실행 결과 없음({(meta.get("replay") or {}).get("reason") or (meta.get("replay") or {}).get("state")})')
  if r['input_nonfinite']:
    r['reading'] += ' / 입력·상태에 이미 NaN·Inf 있음(이전 프레임에서 번진 것)'
  return r


def main(argv=None):
  ap = argparse.ArgumentParser()
  ap.add_argument('path')
  ap.add_argument('--onnx')
  ap.add_argument('--json', action='store_true')
  a = ap.parse_args(argv)
  root = Path(a.path)
  dirs = [root] if (root / 'meta.json').exists() else sorted(d for d in root.iterdir() if (d / 'meta.json').exists())
  if not dirs:
    print('no events', file=sys.stderr)
    return 1
  reports = [report(d, a.onnx) for d in dirs]
  if a.json:
    print(json.dumps(reports, indent=1, default=str))
    return 0
  verdicts = {}
  for r in reports:
    v = (r['replay'] or {}).get('verdict') or (r['replay'] or {}).get('state')
    verdicts[v] = verdicts.get(v, 0) + 1
    print(f"== {r['id']} frame={r['frame']} model={r['model']} boot={str(r['boot'])[:8]}")
    print(f"   {r['detail']}")
    print(f"   replay: {json.dumps(r['replay'], default=str)[:300]}")
    print(f"   => {r['reading']}")
    if r['interrupted']:
      print(f"   interrupted: {r['interrupted']}")
    print(f"   nonfinite: {r['nonfinite_tensors']}  near fp16 limit: {r['near_fp16_limit']}")
    print(f"   prev outputs max|x|: {r['prev_outputs_max_abs']}")
    for k in ('health_pre', 'health_post'):
      h = r[k]
      if h:
        print(f"   {k}: gpu {h['gpu_mhz']} MHz, temp max {h['temp_c_max']}, modes {h['power_modes']}")
        print(f"      cooling {h['cooling_active']}, n={h['samples']} gap {h['gaps_s']:.1f}s")
        print(f"      rails {h['rails']}")
    for k in ('dmesg_at_error', 'dmesg_after', 'prev_boot_kernel'):
      if r.get(k):
        print(f"   {k}:")
        for x in r[k]:
          print('     ' + x[:200])
    if 'onnx_fp32' in r:
      print(f"   onnx fp32: {r['onnx_fp32']}")
  print('verdicts:', verdicts)
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
