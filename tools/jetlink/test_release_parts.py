import base64
import hashlib
import json
from pathlib import Path
import re

import pytest

import release_parts

HERE = Path(__file__).resolve().parent


def _key():
  from Crypto.PublicKey import ECC
  return ECC.generate(curve='ed25519')


def _sign(key, manifest):
  from Crypto.Signature import eddsa
  value = dict(manifest)
  value['signature'] = base64.b64encode(eddsa.new(key, 'rfc8032').sign(release_parts.payload(value))).decode()
  return value


def _publish(tmp_path, monkeypatch, data, part_size, key):
  base = 'https://github.com/2020K9/carrotpilot/releases/download/k9-test/'
  store = {}
  parts = []
  for i, offset in enumerate(range(0, len(data), part_size)):
    chunk = data[offset:offset + part_size]
    name = f'blob.bin.part{i:02d}'
    store[base + name] = chunk
    parts.append(dict(name=name, offset=offset, size=len(chunk), sha256=hashlib.sha256(chunk).hexdigest(), url=base + name))
  manifest = _sign(key, dict(format=1, name='blob.bin', size=len(data), sha256=hashlib.sha256(data).hexdigest(), parts=parts))
  store[base + 'blob.bin.parts.json'] = json.dumps(manifest).encode()

  class Response:
    def __init__(self, body):
      self.body, self.pos = body, 0
    def read(self, n=-1):
      n = len(self.body) - self.pos if n < 0 else n
      out = self.body[self.pos:self.pos + n]
      self.pos += len(out)
      return out
    def __enter__(self):
      return self
    def __exit__(self, *a):
      return False

  monkeypatch.setattr(release_parts, '_open', lambda url: Response(store[url]))
  monkeypatch.setattr(release_parts, 'PUBLIC_KEY', key.public_key().export_key(format='raw'))
  monkeypatch.setattr(release_parts.time, 'sleep', lambda s: None)
  return base + 'blob.bin.parts.json', manifest, store


def test_pure_python_ed25519_matches_pycryptodome():
  from Crypto.Signature import eddsa
  key = _key()
  public = key.public_key().export_key(format='raw')
  for size in (0, 1, 77, 1000):
    message = bytes(range(256)) * (size // 256) + bytes(size % 256)
    signature = eddsa.new(key, 'rfc8032').sign(message)
    assert release_parts.ed25519_verify(public, message, signature)
    assert not release_parts.ed25519_verify(public, message + b'x', signature)
    assert not release_parts.ed25519_verify(public, message, signature[:-1] + bytes([signature[-1] ^ 1]))


def test_fetch_joins_and_checks(tmp_path, monkeypatch):
  data = bytes(range(256)) * 41 + b'tail'
  url, manifest, _ = _publish(tmp_path, monkeypatch, data, 1000, _key())
  out = tmp_path / 'blob.bin'
  assert release_parts.fetch(url, out, manifest['sha256'])['size'] == len(data)
  assert out.read_bytes() == data
  assert not (tmp_path / 'blob.bin.partial').exists()


def test_fetch_rejects_bad_part_wrong_pin_and_tampering(tmp_path, monkeypatch):
  data = bytes(range(256)) * 20
  key = _key()
  url, manifest, store = _publish(tmp_path, monkeypatch, data, 1000, key)
  out = tmp_path / 'blob.bin'
  with pytest.raises(ValueError, match='different file'):
    release_parts.fetch(url, out, 'f' * 64)
  bad = dict(manifest, sha256='0' * 64)
  store[url] = json.dumps(bad).encode()
  with pytest.raises(ValueError, match='signature'):
    release_parts.fetch(url, out)
  store[url] = json.dumps(manifest).encode()
  first = manifest['parts'][1]['url']
  store[first] = b'x' + store[first][1:]
  with pytest.raises(ValueError, match='Part checksum'):
    release_parts.fetch(url, out)
  assert not out.exists() and not (tmp_path / 'blob.bin.partial').exists()
  with pytest.raises(ValueError, match='2020K9'):
    release_parts.load_manifest('https://example.com/blob.bin.parts.json')


def test_published_manifests_signed_and_pinned():
  """The committed copies of the GitHub manifests carry the 2020K9 signature and match the Windows pins."""
  key = (HERE / 'release-signing-public.pem').read_text()
  from Crypto.PublicKey import ECC
  assert ECC.import_key(key).export_key(format='raw') == release_parts.PUBLIC_KEY
  ps1 = (HERE / 'get-carrot-jetson.ps1').read_text(encoding='utf-8')
  expected = {
    ('k9-jetson-v0.4.0-boot-preview', 'carrot-jetson-windows.zip'): (10098321699, 'b531393405eab54df134d51a3cad00f74430aca47e4068b855e92e2c58cc6be4'),
    ('k9-jetson-v0.4.0-boot-preview', 'carrot-jetson.img.zst'): (10073623634, 'b277f3c6318d5b3210eee6e6aa4e74e492f73ffe2f9a669e108822a7ceb6c614'),
    ('k9-jetson-v0.3.2-windows-preview', 'carrot-jetson-windows.zip'): (8274619308, 'e11a0a1da0c7fc78ace17b527d0242c98d68a752888c072c6f18ed1151e506da'),
  }
  files = sorted((HERE / 'release_manifests').glob('*.parts.json'))
  assert len(files) == 3
  for path in files:
    raw = path.read_bytes()
    manifest = release_parts.verify_manifest(json.loads(raw))
    tag = path.name.split('__')[0]
    assert (manifest['size'], manifest['sha256']) == expected[(tag, manifest['name'])]
    assert all(p['url'] == f'https://github.com/2020K9/carrotpilot/releases/download/{tag}/{p["name"]}' for p in manifest['parts'])
    assert all(p['size'] <= 1_900_000_000 for p in manifest['parts'])
    assert hashlib.sha256(raw).hexdigest() in ps1


def test_refresh_uses_signed_parts():
  text = (HERE / 'refresh_protected_image.py').read_text()
  assert 'release_parts.fetch(BASE_MANIFEST_URL, compressed, BASE_SHA)' in text
  assert 'synology' not in text
  assert re.search(r"BASE_MANIFEST_URL = 'https://github.com/2020K9/carrotpilot/releases/download/"
                   r"k9-jetson-v0.4.0-boot-preview/carrot-jetson.img.zst.parts.json'", text)


def test_guides_point_to_parts():
  import build_windows_installer as publisher
  html = (HERE / 'windows_installer/설치안내.html').read_text(encoding='utf-8')
  assert 'synology' not in html and 'get-carrot-jetson.ps1 -Item v032-zip' in html
  guide = publisher.candidate_guide(html, dict(version='v0.4.0-boot-preview', image_bytes=40 << 30, compressed_bytes=10073623634))
  assert guide.count('<p class="parts">') == 1 and 'get-carrot-jetson.ps1 -Item v040-zip' in guide
  assert 'k9-jetson-v0.4.0-boot-preview/carrot-jetson-windows.zip.parts.json' in guide
  private = publisher.candidate_guide(html, dict(version='v0.4.0-storage-candidate', image_bytes=40 << 30, compressed_bytes=1))
  assert '<p class="parts">' not in private
