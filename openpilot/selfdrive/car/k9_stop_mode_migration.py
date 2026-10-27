"""One-time K9StopMode migration for the K9 stop menus.

2026-10-25 (the driver's orders 00:36 "브레이크 앞전꺼는 다 별로니 지워" / "v12 원본 그대로 ... v12k9"): the 0-3 menu of
2026-10-24 evening (0 base, 1 v10, 2 v10 + light shudder fix, 3 v11; K9StopModeVer 2) became 0 base, 1 v12 original,
2 v12k9 (K9StopModeVer 3). card reads K9StopMode live (CarController, every 0.5 s), so this runs in manager_init before any
managed process starts and therefore before the first drive on the new code.

Rules, applied once while K9StopModeVer is not STOP_MODE_VER (4 since 2026-10-28, 3 before):
- K9StopModeVer 2 (the 0-3 menu): 1 / 2 / 3 -> 2 (v12k9: every deleted method was a K9-fitted stop feathering, v12k9 is the
  K9-fitted successor of the old 3 v11; the v12 original is a Niro EV patch the driver must pick himself); 0 unchanged;
- no / other K9StopModeVer (the 0-6 menu before 2026-10-24 evening, never migrated): 1-6 -> 2 for the same reason; 0 unchanged;
- absent: nothing to change; any other value (outside the table, negative, unreadable) is left as stored: it acted as 0
  (base braking) on the old menu and still acts as 0 now (CarController maps everything outside 1 / 2 to 0).
2026-10-26 (the driver's orders 23:14 "v12그냥놓아두고 v13추가하는걸로"): 3 v13 original was added as a new value with NO new
marker and no table change: a K9StopModeVer 3 device keeps its value (the K9StopModeVer 3 settings screen allowed 0-2 only
and the web server clamps to min/max, so a stored 3 there can only come from a hand-written param); the old tables still map
an old 3 (v11 on the 0-3 menu, v9 + shudder fix on the 0-6 menu) to 2, never to the new 3.
2026-10-27 (the driver's orders 15:53 / 16:14 "c1.c1+ 3번 4번추가해", 16:45 "그럼 4.5에넣어야지"): 4 C1 and 5 C1+ were added,
again with NO new marker and no table change (d76f7ecc).
2026-10-28 (the driver's orders 2026-10-27 19:48 "c1 c1+제거하고 말해라", 19:51 "v13.v13k9만든거 제거하시고" and 2026-10-28
01:06 "원본 그대로 푸쉬해라 v14"): 3 v13 original, 4 C1 and 5 C1+ were removed with their code and 3 is now the v14 original
(K9StopModeVer 4). Rule added:
- K9StopModeVer 3 (the 0-5 menu, 3 v13 original / 4 C1 / 5 C1+; 6 = the never-pushed v13k9 branch): 3 / 4 / 5 / 6 -> 0 (base:
  none of them is the v14 original, and the new 3 v14 must be picked by hand; an old 3 v13 is NOT carried to v14),
  0 / 1 / 2 unchanged; any other value is left as stored (it acted as 0 and still acts as 0);
- K9StopModeVer 2 and no marker: the tables above, unchanged (their old 3 means v11 / v9 + shudder fix -> 2, never the new 3).
The marker is written right after the value (two atomic param writes). A marker that is already STOP_MODE_VER means the
value is on the new menu and is never touched again. A power loss exactly between the two writes migrates again on the next
start: 2 -> 2 and 0 -> 0 in every table, so nothing changes.
"""

STOP_MODE_KEY = "K9StopMode"
STOP_MODE_VER_KEY = "K9StopModeVer"
STOP_MODE_VER = 4  # K9StopModeVer after this migration (3 = the 2026-10-25..27 menus up to 0-5, 2 = the 2026-10-24 evening 0-3
# menu, absent = the 0-6 menu or older)
# old K9StopMode -> new (0 base, 1 v12 original, 2 v12k9, 3 v14 original). No opendbc import on purpose: manager_init must not
# depend on the car code (test_k9_stop_v12.py / test_k9_stop_v14.py pin it against k9_creep.MENU).
MENU_MIGRATION_V4 = {0: 0, 1: 1, 2: 2, 3: 0, 4: 0, 5: 0, 6: 0}  # from K9StopModeVer 3 (3 v13 / 4 C1 / 5 C1+ / 6 v13k9 -> 0)
MENU_MIGRATION_V3 = {0: 0, 1: 2, 2: 2, 3: 2}  # from K9StopModeVer 2
MENU_MIGRATION_FROM_V1 = {0: 0, 1: 2, 2: 2, 3: 2, 4: 2, 5: 2, 6: 2}  # from no marker (the 0-6 menu)


def migrate_k9_stop_mode(params) -> bool:
  """Returns True if K9StopMode was rewritten. Never raises (manager start must not fail on a bad stored value)."""
  try:
    ver = params.get(STOP_MODE_VER_KEY)
  except Exception:
    ver = None
  if ver == STOP_MODE_VER:
    return False
  table = MENU_MIGRATION_V4 if ver == 3 else (MENU_MIGRATION_V3 if ver == 2 else MENU_MIGRATION_FROM_V1)
  changed = False
  try:
    old = params.get(STOP_MODE_KEY)
  except Exception:
    old = None  # unreadable: left as stored (acts as 0)
  if isinstance(old, int) and not isinstance(old, bool) and old in table:
    new = table[old]
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
