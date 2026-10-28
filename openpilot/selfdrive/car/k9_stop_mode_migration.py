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
2026-10-29 04:48 (the driver's orders "메뉴1번에다 내가 밟은거 그대로를 넣으라햇잔니"): the slots were swapped so that his
own recorded stop table is menu 1 and the v12 original is menu 2 (K9StopModeVer 5). Rule added:
- K9StopModeVer 4 (the 2026-10-28 0-3 menu): 1 <-> 2 (1 v12 original -> 2, 2 his own stop table -> 1); 0 base and
  3 v14 original unchanged;
- an older / absent marker first runs its own table above (unchanged) to land on the K9StopModeVer 4 numbering and
  then the swap: every output of those tables is a fixed point of the same table, so repeating them changes nothing.
2026-10-29 05:40 (the driver's order "지금 2번인 v12 원본은 메뉴에서 빼라", with 2 = his improved stop table and 4 = his
table + the 1 s hold after the stop): the v12 original left the menu with its code, so 2 no longer means it
(K9StopModeVer 6). Rule added:
- K9StopModeVer 5 (the 2026-10-29 04:48 numbering): 2 (v12 original) -> 0 (base: the slot is now the improved table, which
  is a different method and must be picked by hand); 0 base, 1 his own stop table and 3 v14 original unchanged;
- an older / absent marker first runs its own tables above (unchanged) and the 1 <-> 2 swap to land on the
  K9StopModeVer 5 numbering, then this table. The new menu value 4 is never produced by a migration: it is new and must be
  picked by hand.
DROP_V6 is idempotent (0 / 1 / 3 are fixed points and its only output 0 is one too), so unlike the swap it needs no
pending marker: if a power loss interrupts it, the next start simply applies it again to the same stored value.
A marker that is already STOP_MODE_VER means the value is on the new menu and is never touched again. The swap is the
first rule that is not idempotent, so the pre-swap value is written into the marker (SWAP_PENDING_BASE + value) before
the value itself is rewritten: whatever a power loss interrupts, the next start recomputes the same new value from the
marker instead of swapping an already swapped value.
"""

STOP_MODE_KEY = "K9StopMode"
STOP_MODE_VER_KEY = "K9StopModeVer"
STOP_MODE_VER = 6  # K9StopModeVer after this migration (5 = the 2026-10-29 04:48 numbering with 2 = v12 original,
# 4 = the 2026-10-28 0-3 menu before the 1 <-> 2 swap, 3 = the 2026-10-25..27 menus up to 0-5,
# 2 = the 2026-10-24 evening 0-3 menu, absent = the 0-6 menu or older)
# old K9StopMode -> new (0 base, 1 v12 original, 2 v12k9, 3 v14 original). No opendbc import on purpose: manager_init must not
# depend on the car code (test_k9_stop_v12.py / test_k9_stop_v14.py pin it against k9_creep.MENU).
MENU_MIGRATION_V4 = {0: 0, 1: 1, 2: 2, 3: 0, 4: 0, 5: 0, 6: 0}  # from K9StopModeVer 3 (3 v13 / 4 C1 / 5 C1+ / 6 v13k9 -> 0)
MENU_MIGRATION_V3 = {0: 0, 1: 2, 2: 2, 3: 2}  # from K9StopModeVer 2
MENU_MIGRATION_FROM_V1 = {0: 0, 1: 2, 2: 2, 3: 2, 4: 2, 5: 2, 6: 2}  # from no marker (the 0-6 menu)
# 2026-10-29: the 1 <-> 2 slot swap, applied after the tables above have put the value on the K9StopModeVer 4 numbering
SWAP_V5 = {0: 0, 1: 2, 2: 1, 3: 3}  # from K9StopModeVer 4: 1 v12 original -> 2, 2 his own stop table -> 1
SWAP_PENDING_BASE = 50  # K9StopModeVer 50 + v means "the swap is pending and the pre-swap K9StopMode value was v"
# (0 <= v <= 3). The swap is not idempotent, so this marker is written before the value is rewritten.
# 2026-10-29 05:40: the v12 original is off the menu, so a stored 2 becomes 0 (base). Applied after SWAP_V5. Idempotent
# (every output is a fixed point of the table), so it needs no pending marker.
DROP_V6 = {0: 0, 1: 1, 2: 0, 3: 3}  # from K9StopModeVer 5: 2 v12 original -> 0; 0 / 1 / 3 unchanged


def migrate_k9_stop_mode(params) -> bool:
  """Returns True if K9StopMode was rewritten. Never raises (manager start must not fail on a bad stored value)."""
  try:
    ver = params.get(STOP_MODE_VER_KEY)
  except Exception:
    ver = None
  if ver == STOP_MODE_VER:
    return False
  changed = False
  pending = isinstance(ver, int) and not isinstance(ver, bool) and SWAP_PENDING_BASE <= ver <= SWAP_PENDING_BASE + 3
  if not pending and ver != 5:  # ver 5 is already on the post-swap numbering: only DROP_V6 below is left
    if ver != 4:  # first put the stored value on the K9StopModeVer 4 numbering (those tables repeat harmlessly)
      table = MENU_MIGRATION_V4 if ver == 3 else (MENU_MIGRATION_V3 if ver == 2 else MENU_MIGRATION_FROM_V1)
      try:
        old = params.get(STOP_MODE_KEY)
      except Exception:
        old = None  # unreadable: left as stored (acts as 0)
      if isinstance(old, int) and not isinstance(old, bool) and old in table and table[old] != old:
        try:
          params.put_int(STOP_MODE_KEY, table[old])
          changed = True
        except Exception:
          return False  # not written: no marker either, so the next start tries again
    try:
      v4 = params.get(STOP_MODE_KEY)
    except Exception:
      v4 = None
    if not (isinstance(v4, int) and not isinstance(v4, bool) and v4 in SWAP_V5):
      try:
        params.put_int(STOP_MODE_VER_KEY, STOP_MODE_VER)  # outside the table: it acted as 0 and still acts as 0
      except Exception:
        pass
      return changed
    try:
      params.put_int(STOP_MODE_VER_KEY, SWAP_PENDING_BASE + v4)  # record the pre-swap value before swapping
    except Exception:
      return changed  # marker not written: the next start redoes the same steps from the stored value
    ver = SWAP_PENDING_BASE + v4
  if ver != 5:
    old = ver - SWAP_PENDING_BASE
    new = SWAP_V5[old]
    if new != old:
      try:
        params.put_int(STOP_MODE_KEY, new)
        changed = True
      except Exception:
        return changed  # the marker still says "swap pending": the next start finishes it
  try:  # 2026-10-29 05:40: a stored 2 (v12 original) is now base. Idempotent, so no pending marker is needed.
    v5 = params.get(STOP_MODE_KEY)
  except Exception:
    v5 = None  # unreadable: left as stored (acts as 0)
  if isinstance(v5, int) and not isinstance(v5, bool) and v5 in DROP_V6 and DROP_V6[v5] != v5:
    try:
      params.put_int(STOP_MODE_KEY, DROP_V6[v5])
      changed = True
    except Exception:
      return changed  # no marker written: the next start applies the same (idempotent) table again
  try:
    params.put_int(STOP_MODE_VER_KEY, STOP_MODE_VER)
  except Exception:
    pass
  return changed
