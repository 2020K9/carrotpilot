from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/jetlink'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from jetlink.server import nf_trace


def test_health_sample_survives_bad_thermal_zone(tmp_path, monkeypatch):
  good = tmp_path / 'thermal_zone0'
  good.mkdir()
  (good / 'type').write_text('GPU-therm\n')
  (good / 'temp').write_text('45500\n')
  bad = tmp_path / 'thermal_zone1'
  bad.mkdir()
  (bad / 'type').write_text('bad\n')
  (bad / 'temp').write_text('1\n')
  real = nf_trace._read_small

  def flaky(path):
    if path.parent == bad:
      raise TypeError("can't concat NoneType to bytes")   # what Path.read_text raised on the Jetson
    return real(path)

  monkeypatch.setattr(nf_trace, '_read_small', flaky)
  row = nf_trace.carrot_health_sample(status=tmp_path / 'missing.json', thermal=tmp_path)
  assert row['temp_c'] == {'GPU-therm': 45.5}
  assert 'error' not in row


def test_dmesg_tries_sudo_first(monkeypatch):
  seen = []

  class R:
    returncode, stdout = 0, 'nvgpu: x\nother\n'

  def run(cmd, **kw):
    seen.append(cmd)
    return R()

  monkeypatch.setattr(nf_trace.subprocess, 'run', run)
  text = nf_trace.NfTrace._dmesg()
  assert seen[0][:3] == ['sudo', '-n', 'dmesg']
  assert 'unavailable' not in text


def test_prev_boot_kernel_adds_logkeep_copy(tmp_path, monkeypatch):
  import host_logkeep
  d = tmp_path / '20261122-060000_aaaaaaaa'
  d.mkdir()
  (d / 'kern.log').write_text('nvgpu: mmu fault\n')

  class R:
    returncode, stdout, stderr = 1, '', 'No journal boot entry found'

  monkeypatch.setattr(nf_trace.subprocess, 'run', lambda *a, **k: R())
  monkeypatch.setattr(host_logkeep, 'persistent_root', lambda data=None: tmp_path)
  text = nf_trace.NfTrace._prev_boot_kernel()
  assert 'nvgpu: mmu fault' in text
