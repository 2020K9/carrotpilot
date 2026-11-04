import inspect

from openpilot.selfdrive.controls import controlsd
from openpilot.selfdrive.controls.lib.model_freshness import MODEL_STALE_LAT_S, lateral_model_fresh, model_age_s


def test_fresh_model_keeps_lateral():
  assert lateral_model_fresh(True, 1000, 1000)
  assert lateral_model_fresh(True, 1005, 1000)  # one 20 Hz model period
  assert lateral_model_fresh(True, 1050, 1000)  # exactly 0.5 s


def test_stale_model_releases_lateral():
  assert not lateral_model_fresh(True, 1051, 1000)
  assert not lateral_model_fresh(True, 1500, 1000)


def test_never_seen_model_releases_lateral():
  assert not lateral_model_fresh(False, 0, 0)


def test_age_units():
  assert abs(model_age_s(150, 100) - 0.5) < 1e-9
  assert MODEL_STALE_LAT_S == 0.5


def test_controlsd_gates_lat_active_on_model_freshness():
  src = inspect.getsource(controlsd.Controls.state_control)
  gate = "CC.latActive = CC.latActive and lateral_model_fresh(self.sm.seen[modelV2], self.sm.frame, self.sm.recv_frame[modelV2])"
  assert gate in src
  # after the AlwaysLateral / readiness gates, before the actuators are computed
  assert src.index("CC.latActive = CC.latActive and lateral_ready") < src.index(gate) < src.index("actuators = CC.actuators")
