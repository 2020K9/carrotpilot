"""K9HudNavdy 3 (대감님 11/18 11:49~11:56): smooth 3D board for the K9 D screen.

Black field, grey 3D road seen from behind / above the ego car, white lane lines (dashes move with the car),
white box model ego car (orange blinkers), other cars as grey box models, the lead car with a blue outline
(red while it brakes). Big speed / PRND / set speed / lead distance, relative speed and gap time top left,
clock top right, small front camera bottom right.

Motion: the renderer runs at 20 fps on the Jetson (10 fps before) and the scene is interpolated between
the last two data samples (one sample interval behind, never extrapolated), so lines and cars glide instead of
jumping at the data rate. Display only: nothing here feeds control or any other screen.
"""
from __future__ import annotations

import math
import time

import pyray as rl

# camera behind / above the ego car (metres, design px)
CAM_BACK_M = 10.5
CAM_H_M = 5.0
CAM_PITCH = math.radians(19.0)
FOCAL_PX = 820.0
CX, CY = 960.0, 290.0
NEAR_M = 0.6          # clip distance in front of the camera
ROAD_MAX_M = 110.0
STATIONS = tuple([-6.0, -3.0] + [float(f) for f in range(0, 30, 2)] + [float(f) for f in range(30, 60, 4)]
                 + [float(f) for f in range(60, 111, 6)])

BG = (6, 6, 8)
ROAD_NEAR = (66, 68, 74)
ROAD_FAR = (24, 25, 29)
SHOULDER = (34, 35, 40)
LINE = (236, 238, 242)
EGO_BODY = (238, 240, 244)
EGO_TOP = (252, 252, 254)
EGO_SIDE = (196, 200, 208)
OTHER_BODY = (150, 154, 162)
OTHER_TOP = (186, 190, 198)
OTHER_SIDE = (112, 116, 124)
GLASS = (34, 38, 46)
TAIL = (210, 40, 46)
BLINK = (255, 150, 30)
LEAD_BLUE = (70, 150, 255)
LEAD_RED = (255, 60, 60)
TEXT = (240, 242, 246)
DIM = (110, 114, 124)

LEAD_BRAKE_MPS2 = -1.0      # lead acceleration at or below this = braking (red outline)
SAMPLE_MAX_GAP_S = 0.35     # longer gap between samples: draw the new one as is (no glide across a stall)
DASH_ON_M, DASH_OFF_M = 3.0, 6.0


def lerp(a: float, b: float, t: float) -> float:
  return a + (b - a) * t


def project(x: float, y: float, z: float = 0.0):
  """car frame (x forward, y right, z up) -> (screen x, screen y, depth) or None behind the near plane"""
  rx, rz = x + CAM_BACK_M, z - CAM_H_M
  c, s = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
  depth = rx * c - rz * s
  if depth < NEAR_M:
    return None
  up = rx * s + rz * c
  return CX + FOCAL_PX * y / depth, CY - FOCAL_PX * up / depth, depth


def lat_at(pts, f: float):
  """lateral (m) of a (forward, lateral) polyline at forward f, linear, None outside"""
  if len(pts) < 2 or f < pts[0][0] or f > pts[-1][0]:
    return None
  for (f0, l0), (f1, l1) in zip(pts, pts[1:], strict=False):
    if f0 <= f <= f1:
      return l0 if f1 - f0 < 1e-6 else l0 + (l1 - l0) * (f - f0) / (f1 - f0)
  return None


def resample(pts):
  """polyline on the fixed STATIONS (None where it does not reach); near end extended straight back"""
  if len(pts) < 2:
    return None
  out = []
  for f in STATIONS:
    v = lat_at(pts, f)
    if v is None and f < pts[0][0]:
      v = pts[0][1]
    out.append(v)
  return out if sum(v is not None for v in out) >= 3 else None


def _points(points):
  out = []
  for p in points or ():
    f, lat = getattr(p, "forward_m", None), getattr(p, "lateral_m", None)
    if f is None or lat is None or not (math.isfinite(f) and math.isfinite(lat)):
      continue
    if f > ROAD_MAX_M + 10.0:
      break
    out.append((float(f), float(lat)))
  return out


class Sample:
  __slots__ = ("lines", "edges", "cars", "speed")

  def __init__(self, lines, edges, cars, speed):
    self.lines, self.edges, self.cars, self.speed = lines, edges, cars, speed


def sample_from_state(state, is_lead, others) -> Sample:
  lines = []
  for lane in getattr(state, "lanes", ()) or ():
    r = resample(_points(getattr(lane, "model_points", ())))
    if r is not None and getattr(lane, "visible", True):
      near = next(v for v in r if v is not None)
      lines.append((near, r))
  lines.sort(key=lambda t: t[0])
  edges = (resample(_points(getattr(state, "left_road_edge_points", ()))),
           resample(_points(getattr(state, "right_road_edge_points", ()))))
  cars = {}
  vs = list(others(state))
  lead = next((v for v in getattr(state, "detected_vehicles", ()) or () if is_lead(v)), None)
  if lead is not None:
    vs.append(lead)
  for i, v in enumerate(vs):
    lead_flag = v is lead
    key = "lead" if lead_flag else (f"r{v.radar_track_id}" if getattr(v, "radar_track_id", None) is not None
                                     else f"{v.label}{i}")
    brake = lead_flag and v.acceleration_mps2 is not None and v.acceleration_mps2 <= LEAD_BRAKE_MPS2
    cars[key] = (float(v.longitudinal_m), float(v.lateral_m), lead_flag, brake)
  return Sample(lines, edges, cars, float(getattr(state, "speed_kph", 0.0) or 0.0))


def _mix_list(a, b, t):
  if a is None or b is None or len(a) != len(b):
    return b
  return [y if x is None or y is None else lerp(x, y, t) for x, y in zip(a, b, strict=False)]


def mix(a: Sample | None, b: Sample, t: float) -> Sample:
  """b at t = 1; lines matched by order when the count is the same (else b as is), cars by key"""
  if a is None or t >= 1.0:
    return b
  if len(a.lines) == len(b.lines):
    lines = [(lerp(la[0], lb[0], t), _mix_list(la[1], lb[1], t)) for la, lb in zip(a.lines, b.lines, strict=False)]
  else:
    lines = b.lines
  edges = tuple(_mix_list(ea, eb, t) for ea, eb in zip(a.edges, b.edges, strict=False))
  cars = {}
  for k, (f, lat, lead, brake) in b.cars.items():
    if k in a.cars and abs(a.cars[k][0] - f) < 15.0:
      cars[k] = (lerp(a.cars[k][0], f, t), lerp(a.cars[k][1], lat, t), lead, brake)
    else:
      cars[k] = (f, lat, lead, brake)
  return Sample(lines, edges, cars, lerp(a.speed, b.speed, t))


class Interpolator:
  """keeps the last two samples; a sample is new when its content differs from the last one"""

  def __init__(self):
    self.prev = self.cur = None
    self.t_prev = self.t_cur = 0.0
    self._sig = None

  @staticmethod
  def signature(s: Sample):
    return (tuple((round(n, 3), tuple(None if v is None else round(v, 3) for v in r[:6])) for n, r in s.lines),
            tuple(sorted((k, round(v[0], 2), round(v[1], 2), v[2], v[3]) for k, v in s.cars.items())), round(s.speed, 2))

  def push(self, s: Sample, now: float) -> None:
    sig = self.signature(s)
    if sig == self._sig:
      return
    self._sig = sig
    if self.cur is not None and now - self.t_cur <= SAMPLE_MAX_GAP_S:
      self.prev, self.t_prev = self.cur, self.t_cur
    else:
      self.prev = None
    self.cur, self.t_cur = s, now

  def at(self, now: float) -> Sample | None:
    if self.cur is None:
      return None
    if self.prev is None:
      return self.cur
    span = max(1e-3, self.t_cur - self.t_prev)
    t = (now - self.t_cur) / span   # one sample interval behind: prev at t_cur, cur at t_cur + span
    return mix(self.prev, self.cur, max(0.0, min(1.0, t)))


# ---- drawing --------------------------------------------------------------------------------------------------

def _c(rgb, a=255):
  return rl.Color(int(rgb[0]), int(rgb[1]), int(rgb[2]), int(a))


def _tri_strip(left, right, colors):
  """quads between two point lists (screen), one colour per row; culling off (orientation free)"""
  rl.rl_begin(rl.RL_TRIANGLES)
  for i in range(len(left) - 1):
    a, b, c, d = left[i], right[i], left[i + 1], right[i + 1]
    if None in (a, b, c, d):
      continue
    c0, c1 = colors[i], colors[i + 1]
    for p, col in ((a, c0), (b, c0), (c, c1), (b, c0), (d, c1), (c, c1)):
      rl.rl_color4ub(*col)
      rl.rl_vertex2f(p[0], p[1])
  rl.rl_end()


def _quad(pts, col):
  if any(p is None for p in pts):
    return
  rl.rl_begin(rl.RL_TRIANGLES)
  rl.rl_color4ub(*col)
  for i in (0, 1, 2, 0, 2, 3):
    rl.rl_vertex2f(pts[i][0], pts[i][1])
  rl.rl_end()


def _road_color(f: float):
  k = max(0.0, min(1.0, f / ROAD_MAX_M)) ** 0.7
  fade = 1.0 - max(0.0, (f - ROAD_MAX_M * 0.75) / (ROAD_MAX_M * 0.25))
  rgb = [lerp(n, r, k) for n, r in zip(ROAD_NEAR, ROAD_FAR, strict=False)]
  return tuple(int(lerp(b, v, max(0.0, fade))) for b, v in zip(BG, rgb, strict=False)) + (255,)


def _line_ribbon(lats, half_w: float, alpha_fn):
  left, right, cols = [], [], []
  for f, lat in zip(STATIONS, lats, strict=False):
    if lat is None or f > ROAD_MAX_M:
      left.append(None)
      right.append(None)
      cols.append((0, 0, 0, 0))
      continue
    p, q = project(f, lat - half_w), project(f, lat + half_w)
    left.append(p and (p[0], p[1]))
    right.append(q and (q[0], q[1]))
    a = alpha_fn(f)
    cols.append((*LINE, int(255 * a)))
  _tri_strip(left, right, cols)


def _far_alpha(f: float) -> float:
  return max(0.0, min(1.0, 1.0 - (f - ROAD_MAX_M * 0.7) / (ROAD_MAX_M * 0.3)))


def _dashes(lats, half_w: float, phase: float):
  pts = [(f, l) for f, l in zip(STATIONS, lats, strict=False) if l is not None and f <= ROAD_MAX_M]
  if len(pts) < 2:
    return
  period = DASH_ON_M + DASH_OFF_M
  start = pts[0][0] - ((pts[0][0] + phase) % period)
  f0 = start
  while f0 < pts[-1][0]:
    a, b = max(f0, pts[0][0]), min(f0 + DASH_ON_M, pts[-1][0])
    if b > a:
      la, lb = lat_at(pts, a), lat_at(pts, b)
      if la is not None and lb is not None:
        p = [project(a, la - half_w), project(a, la + half_w), project(b, lb + half_w), project(b, lb - half_w)]
        if all(pts_ is not None for pts_ in p):
          _quad([(q[0], q[1]) for q in p], (*LINE, int(255 * _far_alpha(b))))
    f0 += period


def _box(fx: float, ly: float, yaw: float, length: float, width: float, z0: float, z1: float,
         top, side, rear, shrink: float = 0.0):
  """one cuboid (car frame), visible faces only: rear (camera is behind), top, the side facing the camera"""
  c, s = math.cos(yaw), math.sin(yaw)
  hl, hw = length * 0.5 - shrink, width * 0.5 - shrink

  def w(dx, dy, z):
    return project(fx + dx * c - dy * s, ly + dx * s + dy * c, z)
  rl_ = [w(-hl, -hw, z0), w(-hl, hw, z0), w(-hl, hw, z1), w(-hl, -hw, z1)]
  fr = [w(hl, -hw, z0), w(hl, hw, z0), w(hl, hw, z1), w(hl, -hw, z1)]
  if any(p is None for p in rl_ + fr):
    return None
  p2 = lambda p: (p[0], p[1])  # noqa: E731
  # side facing the camera: the car's left side when it is right of the camera line
  if ly > 0.3:
    _quad([p2(rl_[0]), p2(fr[0]), p2(fr[3]), p2(rl_[3])], side)
  elif ly < -0.3:
    _quad([p2(rl_[1]), p2(fr[1]), p2(fr[2]), p2(rl_[2])], side)
  _quad([p2(rl_[3]), p2(rl_[2]), p2(fr[2]), p2(fr[3])], top)
  _quad([p2(p) for p in rl_], rear)
  return rl_, fr


def draw_car(fx: float, ly: float, yaw: float, ego: bool = False, lead: bool = False, brake: bool = False,
             blink=(False, False), length: float = 4.9, width: float = 1.9):
  body_h, roof_h = 0.78, 1.42
  base = EGO_BODY if ego else OTHER_BODY
  top = EGO_TOP if ego else OTHER_TOP
  side = EGO_SIDE if ego else OTHER_SIDE
  # contact shadow
  sh = [project(fx - length * 0.55, ly - width * 0.62), project(fx - length * 0.55, ly + width * 0.62),
        project(fx + length * 0.55, ly + width * 0.62), project(fx + length * 0.55, ly - width * 0.62)]
  if any(p is None for p in sh):
    return None
  _quad([(p[0], p[1]) for p in sh], (0, 0, 0, 120))
  low = _box(fx, ly, yaw, length, width, 0.18, body_h, (*top, 255), (*side, 255), (*base, 255))
  if low is None:
    return None
  _box(fx - 0.25, ly, yaw, length * 0.56, width * 0.86, body_h, roof_h, (*top, 255), (*side, 255),
       (*GLASS, 255))
  rear = low[0]
  # wheels peeking under the body
  for t0, t1 in ((0.04, 0.2), (0.8, 0.96)):
    a = (lerp(rear[0][0], rear[1][0], t0), rear[0][1])
    b = (lerp(rear[0][0], rear[1][0], t1), rear[0][1])
    _quad([(a[0], a[1] - 2), (b[0], b[1] - 2), (b[0], b[1] + 5), (a[0], a[1] + 5)], (18, 18, 22, 255))
  # tail lamps: a strip across the rear face, blinkers orange on their side
  y_top = lerp(rear[0][1], rear[3][1], 0.78)
  y_bot = lerp(rear[0][1], rear[3][1], 0.62)
  x0, x1 = rear[0][0], rear[1][0]
  wlamp = (x1 - x0) * 0.24
  _quad([(x0, y_bot), (x1, y_bot), (x1, y_top), (x0, y_top)], (*(TAIL if ego else (150, 40, 44)), 255))
  if ego:
    for on, lx in ((blink[0], x0), (blink[1], x1 - wlamp)):
      if on:
        _quad([(lx, y_bot - 2), (lx + wlamp, y_bot - 2), (lx + wlamp, y_top + 2), (lx, y_top + 2)], (*BLINK, 255))
        rl.draw_circle_gradient(rl.Vector2(lx + wlamp * 0.5, (y_top + y_bot) * 0.5), max(8.0, wlamp * 0.9),
                                _c(BLINK, 120), _c(BLINK, 0))
  if lead:
    col = LEAD_RED if brake else LEAD_BLUE
    xs = [p[0] for p in rear] + [p[0] for p in low[1]]
    ys = [p[1] for p in rear] + [p[1] for p in low[1]]
    rx0, rx1, ry0, ry1 = min(xs), max(xs), min(ys), max(ys)
    roof = project(fx, ly, roof_h)
    if roof is not None:
      ry0 = min(ry0, roof[1])
    pad = 4.0
    for k, a in ((10.0, 30), (6.0, 60), (3.0, 110)):
      rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(rx0 - pad - k, ry0 - pad - k, rx1 - rx0 + 2 * (pad + k),
                                                      ry1 - ry0 + 2 * (pad + k)), 0.3, 6, 2.0, _c(col, a))
    rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(rx0 - pad, ry0 - pad, rx1 - rx0 + 2 * pad, ry1 - ry0 + 2 * pad),
                                       0.25, 6, 2.0, _c(col, 255))
  return rear


class K9Scene3D:
  def __init__(self, clock=time.monotonic):
    self.clock = clock
    self.interp = Interpolator()
    self.odo_m = 0.0
    self._last_t = None

  def update(self, state, is_lead, others):
    now = float(self.clock())
    self.interp.push(sample_from_state(state, is_lead, others), now)
    s = self.interp.at(now)
    if self._last_t is not None and s is not None:
      dt = max(0.0, min(0.2, now - self._last_t))
      self.odo_m += s.speed / 3.6 * dt
    self._last_t = now
    return s

  def draw_scene(self, s: Sample, heading_fn, blink):
    rl.draw_rectangle(0, 0, 1920, 480, _c(BG))
    rl.rl_draw_render_batch_active()
    rl.rl_disable_backface_culling()
    try:
      lines = [r for _, r in s.lines]
      lw = s.edges[0] or (lines[0] if lines else None)
      rw = s.edges[1] or (lines[-1] if lines else None)
      if lw is None or rw is None:
        lw = [-5.4 for _ in STATIONS]
        rw = [5.4 for _ in STATIONS]
      # road surface (shoulder 0.6 m beyond the edges)
      left, right, cols, sl, sr = [], [], [], [], []
      for f, a, b in zip(STATIONS, lw, rw, strict=False):
        if a is None or b is None or f > ROAD_MAX_M:
          left.append(None)
          right.append(None)
          sl.append(None)
          sr.append(None)
          cols.append((0, 0, 0, 0))
          continue
        p, q = project(f, a), project(f, b)
        pl, pr = project(f, a - 0.9), project(f, b + 0.9)
        left.append(p and (p[0], p[1]))
        right.append(q and (q[0], q[1]))
        sl.append(pl and (pl[0], pl[1]))
        sr.append(pr and (pr[0], pr[1]))
        cols.append(_road_color(f))
      shoulder = [tuple(int(v * 0.55) for v in c[:3]) + (c[3],) for c in cols]
      _tri_strip(sl, left, shoulder)
      _tri_strip(right, sr, shoulder)
      _tri_strip(left, right, cols)
      # lane lines: outermost solid, inner dashed (dashes move with the car)
      for i, (_, r) in enumerate(s.lines):
        if i in (0, len(s.lines) - 1) and len(s.lines) > 2:
          _line_ribbon(r, 0.09, _far_alpha)
        else:
          _dashes(r, 0.08, self.odo_m)
      if s.edges[0] is not None:
        _line_ribbon(s.edges[0], 0.07, lambda f: 0.55 * _far_alpha(f))
      if s.edges[1] is not None:
        _line_ribbon(s.edges[1], 0.07, lambda f: 0.55 * _far_alpha(f))
      # cars far to near, the ego car last
      for f, lat, lead, brake in sorted(s.cars.values(), key=lambda c: -c[0]):
        # f = gap from my front bumper to its rear bumper; centre = my front 2.45 + f + its half 2.45
        if f > ROAD_MAX_M:
          continue
        if abs(lat) < 2.2:
          f = max(f, 1.0)   # never inside the ego car
        draw_car(f + 4.9, lat, math.radians(heading_fn(f)), lead=lead, brake=brake)
      draw_car(0.0, 0.0, 0.0, ego=True, blink=blink)
    finally:
      rl.rl_draw_render_batch_active()
      rl.rl_enable_backface_culling()
