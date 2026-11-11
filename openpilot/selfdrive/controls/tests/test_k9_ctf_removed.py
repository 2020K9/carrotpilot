"""K9CenterToFront removed (2026-10-29): setting, live helper (controls/lib/k9_ctf.py) and manager migration are gone,
and the Hyundai interface is back to the original `ret.centerToFront = ret.wheelbase * 0.4` (a stale stored value is ignored).
Mass 2010 kg and the other K9 specs stay as they are (test_k9_specs.py)."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from opendbc.car.hyundai import interface
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR

OP = Path(__file__).resolve().parents[3]  # .../openpilot
ROOT = OP.parent
IFACE = ROOT / "opendbc_repo" / "opendbc" / "car" / "hyundai" / "interface.py"
KEY = "K9CenterToFront"


def test_helper_module_gone():
  assert not (OP / "selfdrive" / "controls" / "lib" / "k9_ctf.py").exists()
  assert importlib.util.find_spec("openpilot.selfdrive.controls.lib.k9_ctf") is None


def test_setting_and_key_gone():
  data = json.loads((OP / "selfdrive" / "carrot_settings.json").read_text(encoding="utf-8"))
  assert KEY not in [p["name"] for p in data["params"]]
  assert KEY not in json.dumps(data["menu"], ensure_ascii=False)
  assert KEY not in (OP / "common" / "params_keys.h").read_text(encoding="utf-8")


@pytest.mark.parametrize("rel", ["system/manager/manager.py", "selfdrive/controls/controlsd.py",
                                 "selfdrive/controls/lib/lateral_planner.py"])
def test_no_migration_or_live_hooks(rel):
  src = (OP / rel).read_text(encoding="utf-8")
  assert "k9_ctf" not in src and KEY not in src and "LiveCenterToFront" not in src


@pytest.mark.parametrize("rel", ["docs/user/ko/settings.md", "docs/user/en/settings.md", "docs/k9_menus_zh.md"])
def test_docs_gone(rel):
  assert KEY not in (ROOT / rel).read_text(encoding="utf-8")


def test_interface_uses_original_ratio_source():
  src = IFACE.read_text(encoding="utf-8")
  assert KEY not in src and "ctf" not in src
  assert src.count("ret.centerToFront = ret.wheelbase * 0.4\n") == 1


class _P:
  def __init__(self, vals):
    self.vals = vals

  def get_int(self, key, *a, **k):
    return int(self.vals.get(key, 0))

  def get_bool(self, key, *a, **k):
    return bool(self.vals.get(key, 0))

  def get(self, key, *a, **k):
    return None

  def put_int(self, *a, **k):
    pass

  def put_bool(self, *a, **k):
    pass

  def put(self, *a, **k):
    pass


@pytest.mark.parametrize("car", [CAR.KIA_K9, CAR.HYUNDAI_SONATA, CAR.KIA_EV6])
@pytest.mark.parametrize("vals", [{}, {KEY: 400}, {KEY: 450}, {KEY: 60}])
def test_stale_value_ignored(monkeypatch, car, vals):
  monkeypatch.setattr(interface, "Params", lambda: _P(vals))
  cp = CarInterface.get_params(car, {i: {} for i in range(8)}, [], False, False, False)
  assert cp.centerToFront == float(np.float32(cp.wheelbase * 0.4))  # CarParams fields are Float32
