import pytest

from openpilot.selfdrive.controls.tests.test_lateral_startup_control import control_fixture


def engaged():
  control, step = control_fixture()
  sm = control.sm
  sm['selfdriveState'].enabled = sm['selfdriveState'].active = True
  sm.seen['liveParameters'] = sm.seen['modelV2'] = True
  sm.recv_frame['modelV2'] = sm.frame
  cc, _ = step()
  assert cc.latActive
  return control, step


@pytest.mark.parametrize('age_frames, active', [(0, True), (5, True), (50, True), (51, False), (500, False)])
def test_lateral_released_only_when_model_is_stale(age_frames, active):
  control, step = engaged()
  control.sm.frame = control.sm.recv_frame['modelV2'] + age_frames
  cc, _ = step()
  assert cc.latActive == active
  if not active:
    assert cc.actuators.torque == 0
    assert abs(cc.actuators.curvature - control.curvature) < 1e-9  # no held model curvature


def test_lateral_resumes_when_model_returns():
  control, step = engaged()
  sm = control.sm
  sm.frame += 300
  cc, _ = step()
  assert not cc.latActive
  sm.recv_frame['modelV2'] = sm.frame
  cc, _ = step()
  assert cc.latActive


def test_never_received_model_is_stale():
  control, step = engaged()
  sm = control.sm
  sm.recv_frame['modelV2'] = 0
  sm.frame = 1000
  cc, _ = step()
  assert not cc.latActive
