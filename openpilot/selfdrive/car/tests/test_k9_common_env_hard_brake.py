"""공통환경 비교 counterpart of test_k9_hard_brake.py (k9cmp2235): the original tests that stop at k9_rig's frozen-load guard,
collected again as the same objects with this tree's common modules (see k9_common_env.py). The originals are unchanged and
keep their original result. Model-only, synthetic inputs, not the historical byte-equality check (실차 미검증)."""
from openpilot.selfdrive.car.tests import test_k9_hard_brake as ORIG
from openpilot.selfdrive.car.tests.k9_common_env import bind

TESTS = (
  "test_identical_to_badf1ea7_outside_the_ease_off",
  "test_through_carcontroller_only_jerk_upper_changes",
)
FIXTURES = bind(ORIG, TESTS, globals())
