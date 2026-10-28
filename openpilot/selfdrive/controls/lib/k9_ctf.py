"""K9CenterToFront live update (실험중 메뉴, 2026-10-28, 대감님 21:20 "바꾸는데로 바로적용되게").

centerToFront = wheelbase * value/1000 (value 300..800, 400 = the original Hyundai `wheelbase * 0.4`, 405 = 0.405;
anything else -> 0.4). 대감님 23:45 "기본값이 40이면 내가 셋팅할때 400쓰게 ... 405라고하면 40.5되게".
Old x0.01 values (30..80, the 653441e2 build) still read the same (45 -> 0.45) and manager_init rewrites them
once to the new unit (45 -> 450, migrate_k9_ctf); the two ranges do not overlap.
The car interface reads it once at start (CarParams). This keeps controlsd's VehicleModel and the lateral planner's
factors in step with later changes, read at most once per poll (controlsd 1 Hz, planner every 10 model frames).

Nothing changes until the stored value differs from the one seen at start, so an untouched setting is bit for bit
the old behaviour. paramsd (the learner) keeps the start value until the next restart.
Hyundai/Kia only (CP.brand == "hyundai"). Not tested in the car.
"""
from opendbc.car import scale_tire_stiffness

K9_CTF_PARAM = "K9CenterToFront"
K9_CTF_DEFAULT = 400  # x0.001 of the wheelbase
K9_CTF_MIN, K9_CTF_MAX = 300, 800
K9_CTF_OLD_DEFAULT = 40  # old x0.01 unit (653441e2)
K9_CTF_OLD_MIN, K9_CTF_OLD_MAX = 30, 80


def ctf_ratio(raw) -> float:
  """Stored int -> ratio; default, unreadable or out of range -> 0.4."""
  try:
    v = int(raw)
  except (TypeError, ValueError):
    return 0.4
  if v != K9_CTF_DEFAULT and K9_CTF_MIN <= v <= K9_CTF_MAX:
    return v / 1000.0
  if v != K9_CTF_OLD_DEFAULT and K9_CTF_OLD_MIN <= v <= K9_CTF_OLD_MAX:
    return v / 100.0  # old unit, same double as (v * 10) / 1000.0
  return 0.4


def migrate_k9_ctf(params) -> bool:
  """manager_init, before card/controlsd start: old x0.01 value 30..80 -> x0.001 (40 -> 400, 45 -> 450), so the web
  shows the new unit. Same ratio before and after. Idempotent (300..800 is never touched). Never raises."""
  try:
    v = params.get_int(K9_CTF_PARAM)
  except Exception:
    return False
  if isinstance(v, bool) or not isinstance(v, int) or not (K9_CTF_OLD_MIN <= v <= K9_CTF_OLD_MAX):
    return False
  try:
    params.put_int(K9_CTF_PARAM, v * 10)
    return True
  except Exception:
    return False


def read_raw(params):
  try:
    return params.get_int(K9_CTF_PARAM)
  except Exception:
    return None


class LiveCenterToFront:
  def __init__(self, CP, raw_at_start, every=1):
    self.CP = CP
    self.enabled = getattr(CP, "brand", "") == "hyundai"
    self.raw = raw_at_start
    self.every = max(int(every), 1)
    self._n = 0

  def due(self) -> bool:
    """True once per `every` calls (own counter, so callers need no frame number)."""
    if not self.enabled:
      return False
    self._n += 1
    if self._n >= self.every:
      self._n = 0
      return True
    return False

  def poll(self, raw):
    """New centerToFront [m] when the stored value changed since the last poll, else None."""
    if not self.enabled or raw is None or raw == self.raw:
      return None
    self.raw = raw
    return self.CP.wheelbase * ctf_ratio(raw)

  def check(self, params):
    """Read the param from `params` and return the new centerToFront [m] if it changed, else None."""
    return self.poll(read_raw(params))

  @classmethod
  def from_params(cls, CP, params, every=1):
    return cls(CP, read_raw(params), every=every)

  def tire_stiffness(self, ctf):
    return scale_tire_stiffness(self.CP.mass, self.CP.wheelbase, ctf, self.CP.tireStiffnessFactor)

  def apply_vehicle_model(self, VM, ctf):
    VM.aF = ctf
    VM.aR = self.CP.wheelbase - ctf
    VM.cF_orig, VM.cR_orig = self.tire_stiffness(ctf)  # VM.update_params() (every frame) rescales cF/cR from these

  def planner_factors(self, ctf):
    _, tire_rear = self.tire_stiffness(ctf)
    return self.CP.wheelbase - ctf, (ctf * self.CP.mass) / (self.CP.wheelbase * tire_rear)
