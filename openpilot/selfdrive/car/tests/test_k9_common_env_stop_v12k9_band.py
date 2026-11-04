"""공통환경 비교 counterpart of test_k9_stop_v12k9_band.py (k9cmp2235): the original tests that stop at k9_rig's frozen-load guard,
collected again as the same objects with this tree's common modules (see k9_common_env.py). The originals are unchanged and
keep their original result. Model-only, synthetic inputs, not the historical byte-equality check (실차 미검증)."""
from openpilot.selfdrive.car.tests import test_k9_stop_v12k9_band as ORIG
from openpilot.selfdrive.car.tests.k9_common_env import bind

TESTS = (
  "test_band_object_grid",
  "test_menu1_only_band_compensation_and_stall_change",
  "test_menu1_other_cars_can_identical_to_frozen",
  "test_menus_other_than_the_k9_menu_can_identical_to_frozen",
  "test_update_identical_to_frozen",
)
FIXTURES = bind(ORIG, TESTS, globals())
