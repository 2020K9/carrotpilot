#!/usr/bin/env python3
"""Download a release file that GitHub stores as signed 1.9 GB parts (.part00, .part01, ...).

GitHub release assets are limited to 2 GiB, so large images are published as
parts plus `<name>.parts.json`. The manifest lists each part's size/sha256 and
the whole file's size/sha256 and is signed with the 2020K9 release key
(Ed25519, same payload rule as update_host.verify_signature).

  python release_parts.py fetch <manifest URL> <output file> [--sha256 <whole sha>]

The parts are appended straight into the output file (no second copy on disk);
each part and the whole file are checked before the output is kept.
Works with only the Python standard library (Windows included).
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import time
import urllib.request

# 2020K9 release signing key (raw Ed25519 public key, = release-signing-public.pem)
PUBLIC_KEY = bytes.fromhex('7c687e286025c2bf839062e2cb5c6ee0614d7d4fec656dd81accb7748566cefd')
PART_LIMIT = 2_000_000_000
ALLOWED_PREFIX = ('https://github.com/2020K9/carrotpilot/releases/download/',)

# --- Ed25519 verify (RFC 8032), standard library only -------------------------
_P = 2 ** 255 - 19
_L = 2 ** 252 + 27742317777372353535851937790883648493
_D = -121665 * pow(121666, _P - 2, _P) % _P
_I = pow(2, (_P - 1) // 4, _P)


def _xrecover(y, sign):
  x2 = (y * y - 1) * pow(_D * y * y + 1, _P - 2, _P) % _P
  if x2 == 0:
    return None if sign else 0
  x = pow(x2, (_P + 3) // 8, _P)
  if (x * x - x2) % _P:
    x = x * _I % _P
  if (x * x - x2) % _P:
    return None
  if (x & 1) != sign:
    x = _P - x
  return x


def _add(a, b):
  x1, y1, z1, t1 = a
  x2, y2, z2, t2 = b
  A = (y1 - x1) * (y2 - x2) % _P
  B = (y1 + x1) * (y2 + x2) % _P
  C = t1 * 2 * _D * t2 % _P
  D = z1 * 2 * z2 % _P
  E, F, G, H = B - A, D - C, D + C, B + A
  return (E * F % _P, G * H % _P, F * G % _P, E * H % _P)


def _mul(s, point):
  q = (0, 1, 1, 0)
  while s:
    if s & 1:
      q = _add(q, point)
    point = _add(point, point)
    s >>= 1
  return q


def _encode(point):
  x, y, z, _ = point
  zi = pow(z, _P - 2, _P)
  x, y = x * zi % _P, y * zi % _P
  return (y | ((x & 1) << 255)).to_bytes(32, 'little')


def _decode(data):
  y = int.from_bytes(data, 'little')
  sign, y = y >> 255, y & ((1 << 255) - 1)
  if y >= _P:
    return None
  x = _xrecover(y, sign)
  return None if x is None else (x, y, 1, x * y % _P)


_GY = 4 * pow(5, _P - 2, _P) % _P
_GX = _xrecover(_GY, 0)
_G = (_GX, _GY, 1, _GX * _GY % _P)


def ed25519_verify(public, message, signature):
  if len(public) != 32 or len(signature) != 64:
    return False
  a, r = _decode(public), _decode(signature[:32])
  s = int.from_bytes(signature[32:], 'little')
  if a is None or r is None or s >= _L:
    return False
  h = int.from_bytes(hashlib.sha512(signature[:32] + public + message).digest(), 'little') % _L
  return _encode(_mul(s, _G)) == _encode(_add(r, _mul(h, a)))
# ------------------------------------------------------------------------------


def payload(manifest):
  value = {k: v for k, v in manifest.items() if k != 'signature'}
  return json.dumps(value, sort_keys=True, separators=(',', ':')).encode()


def verify_manifest(manifest, public=None):
  """Signature + structure. Returns the manifest or raises ValueError."""
  public = PUBLIC_KEY if public is None else public
  try:
    signature = base64.b64decode(manifest.get('signature', ''), validate=True)
  except Exception:
    raise ValueError('Parts manifest signature is not valid base64')
  if not ed25519_verify(public, payload(manifest), signature):
    raise ValueError('Parts manifest signature check failed')
  sha = re.compile('[0-9a-f]{64}')
  parts = manifest.get('parts')
  if (manifest.get('format') != 1 or not isinstance(manifest.get('size'), int)
      or not sha.fullmatch(str(manifest.get('sha256', ''))) or not isinstance(parts, list) or not parts):
    raise ValueError('Unsupported parts manifest')
  offset = 0
  for index, part in enumerate(parts):
    if (part.get('name') != f"{manifest['name']}.part{index:02d}" or part.get('offset') != offset
        or not isinstance(part.get('size'), int) or not 0 < part['size'] <= PART_LIMIT
        or not sha.fullmatch(str(part.get('sha256', '')))
        or not str(part.get('url', '')).startswith(ALLOWED_PREFIX)):
      raise ValueError('Invalid part entry %d' % index)
    offset += part['size']
  if offset != manifest['size']:
    raise ValueError('Parts do not add up to the file size')
  return manifest


def _open(url):
  return urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent': 'carrot-release-parts'}), timeout=60)


def load_manifest(source):
  if re.match('https?://', source):
    if not source.startswith(ALLOWED_PREFIX):
      raise ValueError('Manifest must come from the 2020K9 release page')
    with _open(source) as response:
      data = response.read(1 << 20)
  else:
    data = Path(source).read_bytes()
  return verify_manifest(json.loads(data))


def fetch(source, output, sha256=None, progress=None, retries=4):
  """Download all parts into `output`, verifying every part and the whole file."""
  manifest = load_manifest(source)
  if sha256 is not None and manifest['sha256'] != sha256:
    raise ValueError('Signed manifest is for a different file')
  output = Path(output)
  temporary = output.with_name(output.name + '.partial')
  whole = hashlib.sha256()
  done = 0
  try:
    with temporary.open('wb') as target:
      for part in manifest['parts']:
        for attempt in range(retries):
          target.seek(part['offset'])
          target.truncate()
          h, count, saved = hashlib.sha256(), 0, whole.copy()
          try:
            with _open(part['url']) as response:
              for block in iter(lambda: response.read(1 << 20), b''):
                count += len(block)
                if count > part['size']:
                  raise ValueError('Oversized part')
                h.update(block)
                saved.update(block)
                target.write(block)
                if progress is not None:
                  progress(done + count, manifest['size'])
            if count != part['size'] or h.hexdigest() != part['sha256']:
              raise ValueError('Part checksum/length mismatch: ' + part['name'])
            break
          except Exception as error:
            if attempt + 1 == retries:
              raise
            print(f'retry {part["name"]}: {error}', file=sys.stderr, flush=True)
            time.sleep(5 * (attempt + 1))
        whole = saved
        done += part['size']
      target.flush()
      os.fsync(target.fileno())
    if temporary.stat().st_size != manifest['size'] or whole.hexdigest() != manifest['sha256']:
      raise ValueError('Joined file checksum mismatch')
    os.replace(temporary, output)
  finally:
    if temporary.exists():
      temporary.unlink()
  return manifest


def sign(manifest, private_key_path):
  """Publisher only: sign with the release private key (needs pycryptodome)."""
  from Crypto.PublicKey import ECC
  from Crypto.Signature import eddsa
  key = ECC.import_key(Path(private_key_path).read_text())
  value = {k: v for k, v in manifest.items() if k != 'signature'}
  value['signature'] = base64.b64encode(eddsa.new(key, 'rfc8032').sign(payload(value))).decode()
  verify_manifest(value)
  return value


def _progress(count, total):
  print(f'\r{count / total * 100:5.1f}%  {count / 1e9:.2f}/{total / 1e9:.2f} GB', end='', file=sys.stderr, flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = parser.add_subparsers(dest='command', required=True)
  f = sub.add_parser('fetch', help='download, join and verify')
  f.add_argument('manifest')
  f.add_argument('output', type=Path)
  f.add_argument('--sha256', help='expected whole-file sha256 (optional extra pin)')
  v = sub.add_parser('verify', help='check a manifest signature only')
  v.add_argument('manifest')
  args = parser.parse_args()
  if args.command == 'verify':
    m = load_manifest(args.manifest)
    print('OK', m['name'], m['size'], m['sha256'])
  else:
    m = fetch(args.manifest, args.output, args.sha256, _progress)
    print('\nOK', args.output, m['size'], m['sha256'])
