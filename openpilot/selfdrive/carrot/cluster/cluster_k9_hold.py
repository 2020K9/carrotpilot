"""K9 D road card: screen-only time hold of lane lines, road edges and cars (11/13 대감님 01:59).

The incoming state is never changed in place and nothing here feeds control: update() returns a copy for the
drawing code plus per-object alpha. Times are seconds of the caller's clock (log time in replay, monotonic live).

Hold / fade numbers from the 11/12 rlogs (13 segments, 12.4 driving min, hud_hold_0203 scan):
  road edge dropouts p95 0.41 s (98 % <= 1 s), adjacent lane line p50 0.10-0.15 s (86-90 % <= 1 s),
  other cars gap p90 0.84 s, lead (L1) gap p75 0.29 s. Longer gaps are real changes, so a dropped item
  stays full for HOLD_S, then fades out over FADE_OUT_S and is gone after HOLD_S + FADE_OUT_S = 1.1 s.
"""
from __future__ import annotations

import math
from dataclasses import replace

HOLD_S = 0.6           # full strength after the last real sighting
FADE_OUT_S = 0.5       # then linear fade to 0; never drawn after HOLD_S + FADE_OUT_S
FADE_IN_S = 0.25       # new lane line / side car: alpha FADE_IN_FROM -> 1 (the lead is shown at once)
FADE_IN_FROM = 0.35
RESET_DT_S = 1.0       # clock not moving or jumped (single frames, seek, new scene): no hold, state as given
LANE_TAU_S = 0.08      # lane position low pass ...
LANE_MAX_LAG_M = 0.15  # ... never further than this from the real line (lane change follows)
LANE_SNAP_M = 0.6      # bigger jump = another line / re-index: take it as is
LANE_GATE_M = 0.9
CAR_TAU_S = 0.12
CAR_MAX_LAG_M = 0.6
CAR_MAX_LAG_LAT_M = 0.25
CAR_GATE_F_M = 4.0
CAR_GATE_F_K = 0.10
CAR_GATE_L_M = 1.2
CAR_LOOSE_K = 3.0      # second pass: a car not seen this frame vs a new one, 12 m / 30 % and 1.5 m


def _near_lat(points, at_m: float = 5.0) -> float | None:
    pts = [(p.forward_m, p.lateral_m) for p in points or ()
           if p.forward_m is not None and p.lateral_m is not None and math.isfinite(p.forward_m) and math.isfinite(p.lateral_m)]
    if len(pts) < 2:
        return None
    if at_m <= pts[0][0]:
        return pts[0][1]
    for (f0, l0), (f1, l1) in zip(pts, pts[1:], strict=False):
        if f0 <= at_m <= f1:
            return l0 if f1 - f0 < 1e-6 else l0 + (l1 - l0) * (at_m - f0) / (f1 - f0)
    return pts[0][1]


def _shift_points(points, d: float):
    if abs(d) < 1e-9:
        return points
    return tuple(replace(p, lateral_m=p.lateral_m + d) for p in points)


def _alpha(age_seen: float, unseen: float, fade_in: bool) -> float:
    a = 1.0
    if fade_in and age_seen < FADE_IN_S:
        a = FADE_IN_FROM + (1.0 - FADE_IN_FROM) * max(0.0, age_seen) / FADE_IN_S
    if unseen > HOLD_S:
        a *= max(0.0, 1.0 - (unseen - HOLD_S) / FADE_OUT_S)
    return a


def _lp(prev: float, raw: float, dt: float, tau: float, max_lag: float) -> float:
    k = 1.0 - math.exp(-dt / tau) if tau > 0.0 else 1.0
    v = prev + (raw - prev) * k
    return min(raw + max_lag, max(raw - max_lag, v))


class _Track:
    __slots__ = ("obj", "pos", "born", "seen", "fade_in", "vrel")

    def __init__(self, obj, pos, now, fade_in, vrel=None):
        self.obj, self.pos, self.born, self.seen, self.fade_in, self.vrel = obj, pos, now, now, fade_in, vrel


class K9RoadHold:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.now: float | None = None
        self.lanes: list[_Track] = []
        self.edges: dict[str, _Track] = {}
        self.cars: list[_Track] = []
        self.lead: _Track | None = None

    # ---------------------------------------------------------------- public
    def update(self, state, now: float, others_fn, is_lead):
        """(state for drawing, {id(obj): alpha}). others_fn(state) = the side cars the road draws,
        is_lead(vehicle) = that vehicle is the drawn lead (L1)."""
        dt = None if self.now is None else now - self.now
        fresh = dt is None or not (0.0 < dt <= RESET_DT_S)
        if fresh:
            self.reset()
        self.now = now
        lanes, lane_alpha = self._lanes(state, now, 0.0 if fresh else dt, fresh)
        edges = self._edges(state, now)
        vehicles, car_alpha = self._cars(state, now, 0.0 if fresh else dt, fresh, others_fn, is_lead)
        if fresh:
            return state, {}
        alpha = {**lane_alpha, **car_alpha}
        return replace(state, lanes=tuple(lanes), detected_vehicles=tuple(vehicles), **edges), alpha

    # ---------------------------------------------------------------- lanes
    def _lanes(self, state, now, dt, fresh):
        raw = list(getattr(state, "lanes", ()) or ())
        with_lat = [(ln, _near_lat(ln.model_points)) for ln in raw]
        drawable = [(ln, lat) for ln, lat in with_lat if lat is not None]
        inner = set()
        lefts = [t for t in drawable if t[1] < 0.0]
        rights = [t for t in drawable if t[1] >= 0.0]
        if lefts:
            inner.add(id(max(lefts, key=lambda t: t[1])[0]))
        if rights:
            inner.add(id(min(rights, key=lambda t: t[1])[0]))
        # a line counts as seen when the road would draw it: ego lines always, the others when visible
        seen = [(ln, lat) for ln, lat in drawable if id(ln) in inner or getattr(ln, "visible", True)]
        used: set[int] = set()
        for tr in sorted(self.lanes, key=lambda t: -t.seen):
            best = None
            for i, (_, lat) in enumerate(seen):
                if i in used:
                    continue
                e = abs(lat - tr.pos)
                if e < LANE_GATE_M and (best is None or e < best[0]):
                    best = (e, i)
            if best is None:
                continue
            used.add(best[1])
            ln, lat = seen[best[1]]
            pos = lat if abs(lat - tr.pos) > LANE_SNAP_M else _lp(tr.pos, lat, dt, LANE_TAU_S, LANE_MAX_LAG_M)
            tr.pos, tr.seen = pos, now
            tr.obj = ln if abs(pos - lat) < 1e-9 else replace(ln, model_points=_shift_points(ln.model_points, pos - lat))
        for i, (ln, lat) in enumerate(seen):
            if i not in used:
                self.lanes.append(_Track(ln, lat, now, not fresh))
        self.lanes = [t for t in self.lanes if now - t.seen < HOLD_S + FADE_OUT_S]
        out, alpha = [], {}
        live_ids = {id(t.obj) for t in self.lanes}
        for t in self.lanes:
            if t.seen != now and not t.obj.visible:
                t.obj = replace(t.obj, visible=True)  # held line: its last drawn shape
            obj = t.obj
            a = _alpha(now - t.born, now - t.seen, t.fade_in)
            if a < 1.0:
                alpha[id(obj)] = a
            out.append(obj)
        # lines the road would not draw now (invisible outer lines) stay in the list, unchanged
        out += [ln for ln, _ in with_lat if id(ln) not in live_ids and not any(ln is o for o in out)
                and not any(ln is s for s, _ in seen)]
        return out, alpha

    # ---------------------------------------------------------------- road edges
    def _edges(self, state, now):
        upd = {}
        for side in ("left", "right"):
            pts = getattr(state, f"{side}_road_edge_points", ()) or ()
            ok = sum(1 for p in pts if math.isfinite(p.forward_m) and math.isfinite(p.lateral_m)) >= 2
            if ok:
                self.edges[side] = _Track(pts, 0.0, now, False)
            else:
                tr = self.edges.get(side)
                if tr is not None and now - tr.seen < HOLD_S + FADE_OUT_S:
                    upd[f"{side}_road_edge_points"] = tr.obj  # surface keeps its last kerb (no fade: the band
                else:                                         # falls back to the outer lane line after)
                    self.edges.pop(side, None)
        return upd

    # ---------------------------------------------------------------- cars
    @staticmethod
    def _match_cost(tr, v, dt, k: float = 1.0) -> float:
        if tr.obj.radar_track_id is not None and tr.obj.radar_track_id == v.radar_track_id:
            return 0.0
        pf = tr.pos[0] + (tr.vrel or 0.0) * dt
        return abs(v.longitudinal_m - pf) / (k * max(CAR_GATE_F_M, CAR_GATE_F_K * v.longitudinal_m)) + \
            abs(v.lateral_m - tr.pos[1]) / (min(k, 1.25) * CAR_GATE_L_M)

    def _smooth(self, tr, v, now, dt):
        pf = tr.pos[0] + (tr.vrel or 0.0) * (now - tr.seen)
        f = _lp(pf, v.longitudinal_m, dt, CAR_TAU_S, max(CAR_MAX_LAG_M, 0.03 * v.longitudinal_m))
        lat = _lp(tr.pos[1], v.lateral_m, dt, CAR_TAU_S, CAR_MAX_LAG_LAT_M)
        tr.pos, tr.seen, tr.vrel = (f, lat), now, v.relative_speed_mps
        tr.obj = v if (f, lat) == (v.longitudinal_m, v.lateral_m) else replace(v, longitudinal_m=f, lateral_m=lat)

    def _predict(self, tr, now):
        """held car: last relative speed carries it on"""
        f = tr.pos[0] + (tr.vrel or 0.0) * (now - tr.seen)
        return replace(tr.obj, longitudinal_m=f, lateral_m=tr.pos[1]) if f != tr.obj.longitudinal_m else tr.obj

    def _cars(self, state, now, dt, fresh, others_fn, is_lead):
        raw = list(getattr(state, "detected_vehicles", ()) or ())
        lead = next((v for v in raw if is_lead(v)), None)
        others = list(others_fn(state))
        drawn = {id(v) for v in others} | ({id(lead)} if lead is not None else set())
        alpha = {}
        out = [v for v in raw if id(v) not in drawn]  # cars the road never draws: as given
        # lead: shown at once at full strength; held (predicted, fading) only while no L1 at all
        if lead is not None:
            if self.lead is not None and self._match_cost(self.lead, lead, now - self.lead.seen) < 1.0:
                self._smooth(self.lead, lead, now, dt)
            else:
                self.lead = _Track(lead, (lead.longitudinal_m, lead.lateral_m), now, False, lead.relative_speed_mps)
            out.append(self.lead.obj)
        elif self.lead is not None and now - self.lead.seen < HOLD_S + FADE_OUT_S:
            obj = self._predict(self.lead, now)
            if obj.longitudinal_m > 0.0:
                a = _alpha(1.0, now - self.lead.seen, False)
                alpha[id(obj)] = a
                out.append(obj)
            else:
                self.lead = None
        else:
            self.lead = None
        used: set[int] = set()
        for tr in sorted(self.cars, key=lambda t: -t.seen):
            best = None
            for i, v in enumerate(others):
                if i in used:
                    continue
                c = self._match_cost(tr, v, now - tr.seen)
                if c < 1.0 and (best is None or c < best[0]):
                    best = (c, i)
            if best is not None:
                used.add(best[1])
                self._smooth(tr, others[best[1]], now, dt)
        # noisy model cars jump: a car left over near a car not seen this frame is that car again (no twin)
        for tr in sorted(self.cars, key=lambda t: -t.seen):
            if tr.seen == now:
                continue
            best = None
            for i, v in enumerate(others):
                if i in used:
                    continue
                c = self._match_cost(tr, v, now - tr.seen, CAR_LOOSE_K)
                if c < 1.0 and (best is None or c < best[0]):
                    best = (c, i)
            if best is not None:
                used.add(best[1])
                tr.pos = (others[best[1]].longitudinal_m, others[best[1]].lateral_m)
                self._smooth(tr, others[best[1]], now, dt)
        for i, v in enumerate(others):
            if i not in used:
                self.cars.append(_Track(v, (v.longitudinal_m, v.lateral_m), now, not fresh, v.relative_speed_mps))
        keep = []
        for tr in self.cars:
            unseen = now - tr.seen
            if unseen >= HOLD_S + FADE_OUT_S:
                continue
            obj = tr.obj if unseen == 0.0 else self._predict(tr, now)
            if obj.longitudinal_m <= 0.0:
                continue
            if unseen > 0.0 and any(abs(obj.longitudinal_m - o.longitudinal_m) < CAR_GATE_F_M and
                                    abs(obj.lateral_m - o.lateral_m) < CAR_GATE_L_M for o in out if o is not obj):
                continue  # a held car on top of a real one: the real one wins
            keep.append(tr)
            a = _alpha(now - tr.born, unseen, tr.fade_in)
            if a < 1.0:
                alpha[id(obj)] = a
            out.append(obj)
        self.cars = keep
        return out, alpha
