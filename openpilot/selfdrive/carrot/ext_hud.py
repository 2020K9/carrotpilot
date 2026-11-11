"""K9ExtHud: experimental external HUD screen selector (실험중 > 외부 HUD 화면 선택).

Values (anything else, a missing key or a read error = -1):
  -1  not used (default). Everything is exactly as before: ClusterHud and the
      external HUD group settings decide whether and how the HUD runs.
   0  existing external HUD screen. Starts the same carrot_cluster HUD that
      ClusterHud=1 starts, with all external HUD group settings applied.
   1  taxi meter screen. Same as 0, but the right information panel shows the
      taxi meter (cluster/cluster_taxi_meter.py).

Conflict rule: K9ExtHud 0/1 and ClusterHud=1 never stay on together.
  * Setting K9ExtHud to 0/1 turns ClusterHud off (0).
  * Setting ClusterHud to 1 returns K9ExtHud to -1.
The web settings save path applies the rule at write time
(apply_exclusive_after_set); the HUD autorun loop applies it again with
ExtHudReconciler for every other writer (backup restore, QR import, ...).
Both on with no known order (e.g. a restored backup) -> K9ExtHud wins, which
shows the same screen. Either way there is only one carrot_cluster process,
because both settings feed the same manager process gate.

This module only uses the Params get_int/put interface so it stays importable
from manager, the web server and the HUD process alike.
"""
from __future__ import annotations

EXT_HUD_PARAM = "K9ExtHud"
CLUSTER_HUD_PARAM = "ClusterHud"

EXT_HUD_OFF = -1
EXT_HUD_DEFAULT_SCREEN = 0
EXT_HUD_TAXI_METER = 1
EXT_HUD_VALUES = (EXT_HUD_OFF, EXT_HUD_DEFAULT_SCREEN, EXT_HUD_TAXI_METER)


def normalize_ext_hud(value) -> int:
  try:
    number = int(value)
  except (TypeError, ValueError):
    return EXT_HUD_OFF
  return number if number in (EXT_HUD_DEFAULT_SCREEN, EXT_HUD_TAXI_METER) else EXT_HUD_OFF


def read_ext_hud(params) -> int:
  try:
    return normalize_ext_hud(params.get_int(EXT_HUD_PARAM))
  except Exception:
    return EXT_HUD_OFF


def ext_hud_active(value) -> bool:
  return normalize_ext_hud(value) != EXT_HUD_OFF


def effective_cluster_hud_mode(cluster_hud: int, ext_hud) -> int:
  """The ClusterHud number the HUD process should act on.

  K9ExtHud -1 returns ClusterHud unchanged (identical to before)."""
  if ext_hud_active(ext_hud):
    return 1
  return cluster_hud


def read_effective_cluster_hud_mode(params) -> int:
  """params.get_int("ClusterHud") with K9ExtHud folded in. Raises like get_int
  when ClusterHud itself cannot be read, so callers keep their old handling."""
  cluster_hud = int(params.get_int(CLUSTER_HUD_PARAM))
  return effective_cluster_hud_mode(cluster_hud, read_ext_hud(params))


def taxi_meter_selected(params) -> bool:
  return read_ext_hud(params) == EXT_HUD_TAXI_METER


def _put_int(params, key: str, value: int) -> None:
  put = getattr(params, "put_int", None)
  if callable(put):
    put(key, int(value))
  else:
    params.put(key, str(int(value)))


def exclusive_partner_change(name: str, value) -> tuple[str, int] | None:
  """Which other key must change when `name` is set to `value`."""
  if name == EXT_HUD_PARAM and ext_hud_active(value):
    return CLUSTER_HUD_PARAM, 0
  if name == CLUSTER_HUD_PARAM:
    try:
      on = int(value) == 1
    except (TypeError, ValueError):
      on = False
    if on:
      return EXT_HUD_PARAM, EXT_HUD_OFF
  return None


def apply_exclusive_after_set(params, name: str, value) -> tuple[str, int, int] | None:
  """Call right after `name` was written. Returns (key, old, new) if the
  partner key was changed, None otherwise. Never raises."""
  change = exclusive_partner_change(name, value)
  if change is None:
    return None
  key, new = change
  try:
    old = int(params.get_int(key))
  except Exception:
    old = None
  if key == EXT_HUD_PARAM and normalize_ext_hud(old) == EXT_HUD_OFF:
    return None
  if key == CLUSTER_HUD_PARAM and old is not None and old != 1:
    return None
  try:
    _put_int(params, key, new)
  except Exception:
    return None
  return key, old, new


class ExtHudReconciler:
  """Process-side guard. Remembers the last seen values so the setting that
  was switched on last wins; with no history K9ExtHud wins."""

  def __init__(self) -> None:
    self._last_cluster_hud: int | None = None
    self._last_ext_hud: int | None = None

  def step(self, params) -> str | None:
    try:
      cluster_hud = int(params.get_int(CLUSTER_HUD_PARAM))
    except Exception:
      return None
    ext_hud = read_ext_hud(params)
    action = None
    if cluster_hud == 1 and ext_hud_active(ext_hud):
      cluster_turned_on = self._last_cluster_hud is not None and self._last_cluster_hud != 1
      ext_turned_on = self._last_ext_hud is not None and not ext_hud_active(self._last_ext_hud)
      try:
        if cluster_turned_on and not ext_turned_on:
          _put_int(params, EXT_HUD_PARAM, EXT_HUD_OFF)
          ext_hud = EXT_HUD_OFF
          action = f"{CLUSTER_HUD_PARAM}=1 turned on -> {EXT_HUD_PARAM}=-1"
        else:
          _put_int(params, CLUSTER_HUD_PARAM, 0)
          cluster_hud = 0
          action = f"{EXT_HUD_PARAM}={ext_hud} -> {CLUSTER_HUD_PARAM}=0"
      except Exception:
        action = None
    self._last_cluster_hud = cluster_hud
    self._last_ext_hud = ext_hud
    return action
