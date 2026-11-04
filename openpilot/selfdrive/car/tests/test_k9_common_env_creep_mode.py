"""공통환경 비교 counterpart of test_k9_creep_mode.py (k9cmp2235): the original tests that stop at k9_rig's frozen-load guard,
collected again as the same objects with this tree's common modules (see k9_common_env.py). The originals are unchanged and
keep their original result. Model-only, synthetic inputs, not the historical byte-equality check (실차 미검증)."""
from openpilot.selfdrive.car.tests import test_k9_creep_mode as ORIG
from openpilot.selfdrive.car.tests.k9_common_env import bind

TESTS = (
  "test_invalid_values_act_as_0",
  "test_k9creep_object_identical_to_old_menu",
  "test_k9creep_object_identical_to_old_menu_on_grids",
  "test_new_1_2_identical_can_to_1d0961da_2_3",
  "test_new_modes_identical_can_to_old_menu",
  "test_no_camera_scc_k9_identical_to_old_0",
  "test_stale_old_key_is_never_read",
)
FIXTURES = bind(ORIG, TESTS, globals())
