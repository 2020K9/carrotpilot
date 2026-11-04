import pytest

from openpilot.selfdrive.controls.tests.test_lateral_startup_control import control_fixture

NS_ = 1_000_000_000


def engaged():
  control, step = control_fixture()
  sm = control.sm
  sm['selfdriveState'].enabled = sm['selfdriveState'].active = True
  sm.seen['liveParameters'] = sm.seen['modelV2'] = True
  sm.logMonoTime = {'modelV2': 100 * NS_, 'selfdriveState': 100 * NS_, 'carState': 100 * NS_}
  cc, _ = step()
  assert cc.latActive
  return control, step


@pytest.mark.parametrize('age_s, active', [(0.0, True), (0.05, True), (0.5, True), (0.51, False), (5.0, False)])
def test_lateral_released_only_when_model_is_stale(age_s, active):
  control, step = engaged()
  mono = control.sm.logMonoTime
  mono['selfdriveState'] = mono['carState'] = mono['modelV2'] + int(age_s * NS_)
  cc, _ = step()
  assert cc.latActive == active
  if not active:
    assert cc.actuators.torque == 0
    assert abs(cc.actuators.curvature - control.curvature) < 1e-9  # no held model curvature


@pytest.mark.parametrize('loop_dt', [0.01, 0.05, 0.2])
def test_slow_control_loop_releases_on_elapsed_time_not_loop_count(loop_dt):
  # controlsd loop delayed: each step advances message time by loop_dt instead of 0.01 s
  control, step = engaged()
  mono = control.sm.logMonoTime
  t_model = mono['modelV2']
  t = t_model
  released = None
  for i in range(200):
    t += int(loop_dt * NS_)
    mono['selfdriveState'] = mono['carState'] = t
    control.sm.frame += 1
    cc, _ = step()
    if not cc.latActive:
      released = (t - t_model) / NS_
      break
  assert released is not None
  assert 0.5 < released <= 0.5 + loop_dt + 1e-9


def test_lateral_resumes_when_model_returns():
  control, step = engaged()
  mono = control.sm.logMonoTime
  mono['selfdriveState'] = mono['carState'] = mono['modelV2'] + 3 * NS_
  cc, _ = step()
  assert not cc.latActive
  mono['modelV2'] = mono['carState']
  cc, _ = step()
  assert cc.latActive


def test_never_received_model_is_stale():
  control, step = engaged()
  mono = control.sm.logMonoTime
  mono['modelV2'] = 0
  cc, _ = step()
  assert not cc.latActive
