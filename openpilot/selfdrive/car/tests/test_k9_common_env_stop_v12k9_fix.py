"""공통환경 비교 counterpart of test_k9_stop_v12k9_fix.py (k9cmp2235): the original tests that stop at k9_rig's frozen-load guard,
collected again as the same objects with this tree's common modules (see k9_common_env.py). The originals are unchanged and
keep their original result. Model-only, synthetic inputs, not the historical byte-equality check (실차 미검증)."""
from openpilot.selfdrive.car.tests import test_k9_stop_v12k9_fix as ORIG
from openpilot.selfdrive.car.tests.k9_common_env import bind

TESTS = (
  "test_fix_constants_are_the_old_v11_guard",
  "test_menu1_other_cars_can_identical_to_f9cd0960",
  "test_menus_other_than_the_k9_menu_can_identical_to_f9cd0960",
  "test_never_brakes_less_than_v11_for_the_same_inputs",
  "test_pushed_v12k9_braked_less_than_v11_somewhere",
  "test_release_limit_only_while_not_braking_harder_than_planned",
  "test_v12_parts_still_act",
)
FIXTURES = bind(ORIG, TESTS, globals())
