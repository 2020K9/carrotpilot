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
2026-10-29 07:10 (the driver's order "3.v14제거후 4번을 3번으로"): the v14 original left the menu with its two page blocks and
menu 4 (his table + the extra braking 1 s after the stop) moved into the free slot 3 (K9StopModeVer 7). Rule added:
- K9StopModeVer 6 (the 2026-10-29 05:40 numbering): 4 -> 3 (the same method, the new number) and 3 (v14 original) -> 0
  (base: that method is gone from the tree); 0 base, 1 his own stop table and 2 the improved table unchanged;
- an older / absent marker first runs its own tables above, the 1 <-> 2 swap and the v12-original drop to land on the
  K9StopModeVer 6 numbering, then this table.
A marker that is already STOP_MODE_VER means the value is on the new menu and is never touched again. The swap and the
07:10 move are the rules that are not idempotent (the move would drop the new 3 to 0 if it ran twice), so for each of
them the pre-rule value is written into the marker (SWAP_PENDING_BASE + value, MOVE_PENDING_BASE + value) before the
value itself is rewritten: whatever a power loss interrupts, the next start recomputes the same new value from the
marker instead of applying the rule to an already rewritten value.
"""

STOP_MODE_KEY = "K9StopMode"
STOP_MODE_VER_KEY = "K9StopModeVer"
STOP_MODE_VER = 7  # K9StopModeVer after this migration (6 = the 2026-10-29 05:40 numbering with 3 = v14 original and
# 4 = his table + the 1 s extra braking, 5 = the 04:48 numbering with 2 = v12 original, 4 = the 2026-10-28 0-3 menu
# before the 1 <-> 2 swap, 3 = the 2026-10-25..27 menus up to 0-5, 2 = the 2026-10-24 evening 0-3 menu,
# absent = the 0-6 menu or older)
# old K9StopMode -> new (0 base, 1 v12 original, 2 v12k9, 3 v14 original - both originals have since left the menu).
# No opendbc import on purpose: manager_init must not depend on the car code (test_k9_stop_v12.py pins it against
# k9_creep.MENU).
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
# 2026-10-29 07:10: the v14 original is off the menu and menu 4 moved into its slot. Applied after DROP_V6. NOT idempotent
# (a second pass would take the new 3 to 0), so the pre-move value goes into the marker first.
MOVE_V7 = {0: 0, 1: 1, 2: 2, 3: 0, 4: 3}  # from K9StopModeVer 6: 4 -> 3 (same method), 3 v14 original -> 0
# 4 was a menu value on the K9StopModeVer 6 numbering only, so the 4 -> 3 entry applies to a marker-6 device alone: on any
# older numbering a stored 4 was out of range, acted as 0 and is left exactly as stored (what the SWAP_V5 gate already does
# for a marker 2 / 3 / 4 device). Otherwise a hand-written 4 would become a stop method nobody picked.
MOVE_PENDING_BASE = 60  # K9StopModeVer 60 + v means "the move is pending and the pre-move K9StopMode value was v"
# (0 <= v <= 4). Kept apart from SWAP_PENDING_BASE (50..53) so a marker always names exactly one pending rule.


def migrate_k9_stop_mode(params) -> bool:
  """Returns True if K9StopMode was rewritten. Never raises (manager start must not fail on a bad stored value)."""
  try:
    ver = params.get(STOP_MODE_VER_KEY)
  except Exception:
    ver = None
  if ver == STOP_MODE_VER:
    return False
  changed = False
  is_int = isinstance(ver, int) and not isinstance(ver, bool)
  swap_pending = is_int and SWAP_PENDING_BASE <= ver <= SWAP_PENDING_BASE + 3
  from_v6 = ver == 6  # only a marker-6 device ever had 4 as a menu value; ver is rewritten below
  move_pending = is_int and MOVE_PENDING_BASE <= ver <= MOVE_PENDING_BASE + 4
  if move_pending:  # only the 07:10 move is left; its pre-move value is in the marker
    return _finish_move(params, ver, changed)
  if not swap_pending and ver not in (5, 6):  # ver 5 / 6 are already on the post-swap numbering
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
  if ver not in (5, 6):
    old = ver - SWAP_PENDING_BASE
    new = SWAP_V5[old]
    if new != old:
      try:
        params.put_int(STOP_MODE_KEY, new)
        changed = True
      except Exception:
        return changed  # the marker still says "swap pending": the next start finishes it
  if ver != 6:
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
  try:  # 2026-10-29 07:10: a stored 4 becomes 3 and a stored 3 (v14 original) becomes 0
    v6 = params.get(STOP_MODE_KEY)
  except Exception:
    v6 = None  # unreadable: left as stored (acts as 0)
  if not (isinstance(v6, int) and not isinstance(v6, bool) and v6 in MOVE_V7 and (from_v6 or v6 != 4)):
    try:
      params.put_int(STOP_MODE_VER_KEY, STOP_MODE_VER)  # outside the table: it acted as 0 and still acts as 0
    except Exception:
      pass
    return changed
  try:
    params.put_int(STOP_MODE_VER_KEY, MOVE_PENDING_BASE + v6)  # record the pre-move value before moving
  except Exception:
    return changed  # marker not written: the next start redoes the same steps from the stored value
  return _finish_move(params, MOVE_PENDING_BASE + v6, changed)


def _finish_move(params, ver, changed: bool) -> bool:
  """Apply MOVE_V7 to the pre-move value recorded in the marker, then write STOP_MODE_VER. Never raises."""
  old = ver - MOVE_PENDING_BASE
  new = MOVE_V7[old]
  if new != old:
    try:
      params.put_int(STOP_MODE_KEY, new)
      changed = True
    except Exception:
      return changed  # the marker still says "move pending": the next start finishes it
  try:
    params.put_int(STOP_MODE_VER_KEY, STOP_MODE_VER)
  except Exception:
    pass
  return changed
