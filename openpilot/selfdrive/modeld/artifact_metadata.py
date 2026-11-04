"""Metadata/input-layout checks for comma's generic driving artifacts.

Kept from the removed eGPU runtime (generic_model_runtime.py) because the Jetson
model catalog and the navigation input contract are validated against the same
artifact metadata. Pure numpy; no GPU runtime.
"""
import base64
import math
import pickle

import numpy as np


def model_metadata(jits):
  metadata = jits['metadata']
  specs = jits['input_specs']
  outputs = jits['output_specs']
  if tuple(specs['new_img'][0]) != (2, 6, 128, 256) or np.dtype(specs['new_img'][1]) != np.uint8:
    raise ValueError('unsupported generic model image input')
  state_pairs = {name: f'next_{name}' for name in specs if f'next_{name}' in outputs}
  nav = set(specs) & {'nav_map', 'navigation'}
  if nav:
    if nav != {'nav_map', 'navigation'}:
      raise ValueError('unsupported navigation input contract')
    # Original nav uses byte maps/flat commands; RL nav exports float maps and
    # batched commands. Both take 0..255 pixels (normalization is in the model).
    contract = (np.dtype(specs['nav_map'][1]), tuple(specs['navigation'][0]))
    if (tuple(specs['nav_map'][0]) != (1, 1, 256, 256) or np.dtype(specs['navigation'][1]) != np.float32
        or contract not in ((np.dtype(np.uint8), (47,)), (np.dtype(np.float32), (1, 47)))):
      raise ValueError('unsupported navigation input contract')
  if set(specs) != {'new_img', 'desire', 'traffic_convention', 'action_t'} | set(state_pairs) | nav:
    raise ValueError('unsupported generic model inputs')
  for name, next_name in state_pairs.items():
    shape, dtype, device = specs[name]
    out_shape, out_dtype, out_device = outputs[next_name]
    if tuple(shape) != tuple(out_shape) or np.dtype(dtype) != np.dtype(out_dtype) or device != out_device:
      raise ValueError('incompatible generic model state feedback')
  if not state_pairs or any(spec[2] != 'AMD' for spec in list(specs.values()) + list(outputs.values())):
    raise ValueError('unsupported generic model device/state')
  slices = pickle.loads(base64.b64decode(metadata['metadata']['output_slices']))
  count = math.prod(outputs['outputs'][0])
  if any(s.start < 0 or s.stop > count or s.step not in (None, 1) for s in slices.values()):
    raise ValueError('invalid generic model output slices')
  return metadata['metadata']['model_checkpoint'], slices, state_pairs, count


def host_input_layout(specs, frame_size, state_pairs):
  """Pack maps and commands at declared widths, aligned to 128 bytes."""
  shapes = {name: tuple(spec[0]) for name, spec in specs.items() if name not in state_pairs and name != 'new_img'}
  sizes = {name: (math.prod(shape) * np.dtype(specs[name][1]).itemsize + 127) // 128 * 128 for name, shape in shapes.items()}
  packed = np.zeros(128 + sum(sizes.values()) + 2 * frame_size, np.uint8)
  views = {'tfm': np.ndarray((3, 3), np.float32, buffer=packed, offset=0),
                'big_tfm': np.ndarray((3, 3), np.float32, buffer=packed, offset=36)}
  offset = 128
  for name, shape in shapes.items():
    views[name] = np.ndarray(shape, np.dtype(specs[name][1]), buffer=packed, offset=offset)
    offset += sizes[name]
  frames_offset = offset
  for name in ('img', 'big_img'):
    views[name] = np.ndarray((frame_size,), np.uint8, buffer=packed, offset=offset)
    offset += frame_size
  return packed, views, frames_offset
