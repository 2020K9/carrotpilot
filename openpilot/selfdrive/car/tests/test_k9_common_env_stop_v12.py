"""공통환경 비교 counterpart of test_k9_stop_v12.py (k9cmp2235): the original tests that stop at k9_rig's frozen-load guard,
collected again as the same objects with this tree's common modules (see k9_common_env.py). The originals are unchanged and
keep their original result. Model-only, synthetic inputs, not the historical byte-equality check (실차 미검증)."""
from openpilot.selfdrive.car.tests import test_k9_stop_v12 as ORIG
from openpilot.selfdrive.car.tests.k9_common_env import bind

TESTS = (
  "test_base_and_other_values_identical_to_prev_0",
  "test_other_cars_never_run_the_menu",
  "test_v12k9_stop_phase_is_the_old_v11",
)
FIXTURES = bind(ORIG, TESTS, globals())
