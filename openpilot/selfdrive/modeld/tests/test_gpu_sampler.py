import pytest

from openpilot.selfdrive.modeld import gpu_sampler
from openpilot.selfdrive.modeld.gpu_sampler import GpuSampler, GpuTimingDiag, parse_gpubusy, parse_leading_int
from openpilot.selfdrive.modeld import tools_gpu_headroom as hr


def make_kgsl(tmp_path, gpubusy='  1200   10000\n', pct='12 %\n', freq='585000000\n', freq_sub=None):
  d = tmp_path / 'kgsl-3d0'
  d.mkdir()
  if gpubusy is not None:
    (d / 'gpubusy').write_text(gpubusy)
  if pct is not None:
    (d / 'gpu_busy_percentage').write_text(pct)
  if freq is not None or freq_sub is not None:
    (d / 'devfreq').mkdir()
  if freq is not None:
    (d / 'devfreq' / 'cur_freq').write_text(freq)
  if freq_sub is not None:
    (d / 'devfreq' / '3d00000.qcom,kgsl-3d0').mkdir()
    (d / 'devfreq' / '3d00000.qcom,kgsl-3d0' / 'cur_freq').write_text(freq_sub)
  return d


def test_parsers():
  assert parse_gpubusy('  1200   10000\n') == (1200, 10000)
  assert parse_gpubusy('garbage') == (None, None)
  assert parse_gpubusy(None) == (None, None)
  assert parse_leading_int('37 %') == 37
  assert parse_leading_int('37%\n') == 37
  assert parse_leading_int('585000000') == 585000000
  assert parse_leading_int('%') is None
  assert parse_leading_int(None) is None


def test_sample_parsing(tmp_path):
  s = GpuSampler(str(make_kgsl(tmp_path)), clock=lambda: 5.0)
  sample = s.maybe_sample()
  assert sample == {'gpubusy_used': 1200, 'gpubusy_total': 10000, 'gpu_busy_pct': 12,
                    'gpu_cur_freq': 585000000, 'gpu_sample_mono': 5.0}


def test_freq_glob_fallback(tmp_path):
  s = GpuSampler(str(make_kgsl(tmp_path, freq=None, freq_sub='305000000')), clock=lambda: 0.0)
  assert s.maybe_sample()['gpu_cur_freq'] == 305000000


def test_freq_gpuclk_fallback(tmp_path):
  d = make_kgsl(tmp_path, freq=None)
  (d / 'gpuclk').write_text('400000000\n')
  assert GpuSampler(str(d), clock=lambda: 0.0).maybe_sample()['gpu_cur_freq'] == 400000000


def test_missing_files_give_none(tmp_path):
  s = GpuSampler(str(tmp_path / 'does-not-exist'), clock=lambda: 0.0)
  sample = s.maybe_sample()
  assert sample['gpubusy_used'] is None and sample['gpubusy_total'] is None
  assert sample['gpu_busy_pct'] is None and sample['gpu_cur_freq'] is None


def test_rate_limit_1hz(tmp_path):
  now = [100.0]
  s = GpuSampler(str(make_kgsl(tmp_path)), clock=lambda: now[0])
  assert s.maybe_sample() is not None
  for t in (100.05, 100.5, 100.999):
    now[0] = t
    assert s.maybe_sample() is None
  now[0] = 101.0
  assert s.maybe_sample() is not None
  now[0] = 101.5
  assert s.maybe_sample() is None


def test_exceptions_swallowed(tmp_path, monkeypatch):
  def boom():
    raise RuntimeError('clock broken')
  assert GpuSampler(str(tmp_path), clock=boom).maybe_sample() is None
  monkeypatch.setattr(gpu_sampler, '_read', lambda path: (_ for _ in ()).throw(OSError('io')))
  assert GpuSampler(str(tmp_path), clock=lambda: 0.0).maybe_sample() is None

  class BadSampler:
    def maybe_sample(self):
      raise RuntimeError('x')
  diag = GpuTimingDiag(BadSampler())
  diag.poll()
  diag.add_run('a', None)
  assert diag.context({'k': 1}) == {'k': 1, 'run_intervals': []}
  diag.after_record(object())


def test_timing_diag_pending_until_emit(tmp_path):
  now = [0.0]
  diag = GpuTimingDiag(GpuSampler(str(make_kgsl(tmp_path)), clock=lambda: now[0]))
  diag.add_run(1.00001, 1.02)
  diag.poll()
  ctx = diag.context({'frame_id': 3})
  assert ctx['frame_id'] == 3 and ctx['run_intervals'] == [[1.0, 1.02]] and ctx['gpu_busy_pct'] == 12

  class Diag:
    frames = 5
  diag.after_record(Diag)
  now[0] = 0.5
  diag.poll()
  assert diag.context()['gpu_busy_pct'] == 12  # kept until the next emission
  Diag.frames = 0
  diag.after_record(Diag)
  assert diag.context() == {'run_intervals': []}


def test_overlap_math():
  assert hr.interval_overlap((0, 10), (5, 20)) == 5
  assert hr.interval_overlap((0, 1), (2, 3)) == 0
  assert hr.merge_intervals([(5, 8), (0, 2), (1, 3)]) == [[0, 3], [5, 8]]
  ov = hr.per_run_overlap([(0.0, 0.02), (0.05, 0.07)], [(0.01, 0.03), (0.015, 0.025), (0.1, 0.2)])
  assert ov[0] == (pytest.approx(0.01), pytest.approx(0.5))
  assert ov[1] == (0.0, 0.0)


def test_busy_delta_math():
  # cumulative counters -> deltas
  assert hr.busy_from_gpubusy([(100, 1000), (400, 2000)]) == [0.1, pytest.approx(0.3)]
  # windowed counters (reset each read) -> per-window ratio
  assert hr.busy_from_gpubusy([(300, 1000), (100, 1000)]) == [0.3, 0.1]
  assert hr.busy_from_gpubusy([(None, None), (0, 0)]) == []
  assert hr.headroom_ms(0.3) == pytest.approx(35.0)
  assert hr.headroom_ms(1.5) == 0.0
  assert hr.headroom_ms(None) is None
  assert hr.percentile([1, 2, 3, 4], 50) == 2.5


def test_report_handles_old_logs():
  rt = {hr.MODELD: [{'metrics': {'inference_ms': {'mean': 20.0, 'max': 30.0, 'count': 20}}}], hr.DM: []}
  frame = {'modelV2_exec_ms': [], 'dm_exec_ms': [], 'dm_gpu_exec_ms': [], 'deviceState_gpu_pct': []}
  out = hr.report(rt, frame)
  assert 'gpu_sync_wait_ms absent' in out and 'overlap not computable' in out and '[headroom] absent' in out


def test_report_with_new_fields():
  def rec(comp_runs, mono, **gpu):
    return {'mono_time': mono, 'run_intervals': comp_runs, **gpu,
            'metrics': {'inference_ms': {'mean': 20.0, 'max': 25.0, 'count': 20},
                        'gpu_sync_wait_ms': {'mean': 15.0, 'max': 18.0, 'count': 20}}}
  rt = {hr.MODELD: [rec([[0.0, 0.02]], 1.0, gpubusy_used=100, gpubusy_total=1000, gpu_busy_pct=10, gpu_cur_freq=5),
                    rec([[0.05, 0.07]], 2.0, gpubusy_used=500, gpubusy_total=2000, gpu_busy_pct=40, gpu_cur_freq=5)],
        hr.DM: [rec([[0.01, 0.03]], 1.0)]}
  frame = {'modelV2_exec_ms': [20.0], 'dm_exec_ms': [], 'dm_gpu_exec_ms': [], 'deviceState_gpu_pct': []}
  out = hr.report(rt, frame)
  assert 'NOTES' not in out
  assert 'modeld runs with any overlap : 50.0%' in out
  assert 'from gpubusy deltas' in out
