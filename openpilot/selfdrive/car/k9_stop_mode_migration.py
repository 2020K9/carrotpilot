"""One-time K9StopMode migration for the 2026-10-24 evening menu trim (the driver's order 18:58 "브레이크 2.3.4 제거").

The 0-6 menu (0 base, 1 v10, 2 v9 + shudder fix, 3 v10 + band 0.2, 4 v10 + shudder fix, 5 v10 + light shudder fix, 6 v11)
became 0 base, 1 v10, 2 v10 + light shudder fix (old 5), 3 v11 (old 6). card reads K9StopMode live (CarController, every
0.5 s), so this runs in manager_init before any managed process starts and therefore before the first drive on the new code.

Rules (MENU_MIGRATION_V2 below), applied once while K9StopModeVer is not STOP_MODE_VER:
- old 5 -> 2 and 6 -> 3 (the same K9Creep kinds, bit-identical behavior);
- the removed 2 / 3 / 4 -> 1 (v10: old 3 / 4 ran v10's accel/StopReq path bit for bit, old 2 was v9-based; v10 is the
  closest kept menu, and 0 would drop the stop feathering the driver chose);
- 0 / 1 unchanged; absent: nothing to change;
- any other value (7 or more, negative, unreadable) is left as stored: it acted as 0 (base braking) on the 0-6 menu and
  still acts as 0 now (CarController maps everything outside MENU to 0), so leaving it keeps the driven behavior exactly.
The marker is written right after the value (two atomic param writes). A marker that is already STOP_MODE_VER means the
value is on the new menu and is never touched again. Only a power loss exactly between the two writes could migrate twice
on the next start (5 -> 2 -> 1, 6 -> 3 -> 1): that still lands on a kept menu (v10), never on an undefined value.
"""

STOP_MODE_KEY = "K9StopMode"
STOP_MODE_VER_KEY = "K9StopModeVer"
STOP_MODE_VER = 2  # K9StopModeVer after this migration (absent = the 0-6 menu or older)
# old K9StopMode (0-6 menu) -> new (0 base, 1 v10, 2 v10 + light shudder fix, 3 v11). No opendbc import on purpose:
# manager_init must not depend on the car code (test_k9_stop_mode_migration.py pins it against k9_creep.MENU).
MENU_MIGRATION_V2 = {0: 0, 1: 1, 2: 1, 3: 1, 4: 1, 5: 2, 6: 3}


def migrate_k9_stop_mode(params) -> bool:
  """Returns True if K9StopMode was rewritten. Never raises (manager start must not fail on a bad stored value)."""
  try:
    ver = params.get(STOP_MODE_VER_KEY)
  except Exception:
    ver = None
  if ver == STOP_MODE_VER:
    return False
  changed = False
  try:
    old = params.get(STOP_MODE_KEY)
  except Exception:
    old = None  # unreadable: left as stored (acts as 0)
  if isinstance(old, int) and not isinstance(old, bool) and old in MENU_MIGRATION_V2:
    new = MENU_MIGRATION_V2[old]
    if new != old:
      try:
        params.put_int(STOP_MODE_KEY, new)
        changed = True
      except Exception:
        return False  # not written: no marker either, so the next start tries again
  try:
    params.put_int(STOP_MODE_VER_KEY, STOP_MODE_VER)
  except Exception:
    pass
  return changed
