"""K9 D road: display-only smoothing of the drawn lane lines / road edges / plan band.

대감님 11/13 04:59 "D단 차선 흐물흐물거리잖아 개선해". Nothing here feeds control or any decision: the
renderer hands over the model points it is about to draw and draws the returned points instead.

Per line slot (ego left/right, adjacent lines, road edges, plan path):
  1. space: weighted low-order fit over forward distance on a fixed forward grid, so one noisy model
     point cannot put a kink in the line
  2. time: per grid point position+rate (alpha-beta) filter on log/wall time. It smooths the frame to frame
     noise but follows a steady sideways motion (lane change, curve entry) without falling behind, and a
     change much larger than the noise is taken almost at once
  3. far end: an unsure ego line (low lane-line confidence) is drawn only over its near part, and the far
     end itself is smoothed so the tip does not whip back and forth
A slot whose line jumps by more than SNAP_M near the car (another line took the slot) starts fresh.
"""
from __future__ import annotations

import math

import numpy as np

GRID = tuple([float(f) for f in range(0, 20, 2)] + [float(f) for f in range(20, 141, 5)])


class K9LaneSmoother:
    def __init__(self, alpha_near: float = 0.35, alpha_far: float = 0.2, beta_k: float = 0.4, fast_m: float = 1.5,
                 snap_m: float = 1.6, end_alpha: float = 0.05, stale_s: float = 0.6, deg: int = 3,
                 conf_cut: bool = True, end_grow: float = 0.05, max_end: float = 140.0) -> None:
        self.alpha_near, self.alpha_far, self.beta_k, self.fast_m = alpha_near, alpha_far, beta_k, fast_m
        self.snap_m, self.end_alpha, self.stale_s, self.deg, self.conf_cut = snap_m, end_alpha, stale_s, deg, conf_cut
        self.end_grow, self.max_end = end_grow, max_end
        self.tracks: dict[str, tuple[float, float, dict[float, tuple[float, float]]]] = {}  # key -> (t, end, {f: (lat, rate)})

    def reset(self) -> None:
        self.tracks.clear()

    def _fit(self, points: list[tuple[float, float]], end: float) -> list[tuple[float, float]] | None:
        pts = [(f, lat) for f, lat in points if f <= end + 1e-6]
        if len(pts) < 2:
            return None
        f = np.array([p[0] for p in pts], dtype=float)
        lat = np.array([p[1] for p in pts], dtype=float)
        span = float(f[-1] - f[0])
        grid = [g for g in GRID if f[0] - 1e-6 <= g <= f[-1] + 1e-6]
        if span < 4.0 or len(pts) < 4 or len(grid) < 2:
            return pts
        deg = min(self.deg, 3 if len(pts) >= 6 and span >= 25.0 else 2)
        w = 1.0 / (1.0 + np.maximum(f, 0.0) / 35.0)
        try:
            c = np.polyfit(f, lat, deg, w=w)
        except (ValueError, np.linalg.LinAlgError):
            return pts
        val = np.polyval(c, np.array(grid))
        if not np.all(np.isfinite(val)):
            return pts
        return [(g, float(v)) for g, v in zip(grid, val, strict=True)]

    def smooth(self, key: str, points: list[tuple[float, float]], now: float,
               conf: float | None = None) -> list[tuple[float, float]]:
        if len(points) < 2:
            return points
        end = min(points[-1][0], self.max_end)
        if self.conf_cut and conf is not None and math.isfinite(conf):
            end = min(end, 35.0 + 105.0 * max(0.0, min(1.0, conf)))  # unsure line: near part only
        tr = self.tracks.get(key)
        dt = 0.0 if tr is None else now - tr[0]
        if tr is not None and not (0.0 <= dt <= self.stale_s):
            tr = None  # not drawn for a while (or time went back): start fresh
        if tr is not None:
            ke = 1.0 - (1.0 - self.end_alpha) ** (dt / 0.05)
            if end > tr[1]:
                ke = max(ke, 1.0 - (1.0 - self.end_grow) ** (dt / 0.05))  # a line that grows longer is shown soon
            end = tr[1] + (end - tr[1]) * ke
        fit = self._fit(points, max(end, points[0][0] + 4.0))
        if fit is None:
            return points
        if tr is None or fit[0][0] not in tr[2] or abs(fit[0][1] - tr[2][fit[0][0]][0]) > self.snap_m:
            self.tracks[key] = (now, end, {f: (v, 0.0) for f, v in fit})
            return fit
        n = dt / 0.05  # filter gains are per 20 Hz frame; scale them for other frame times
        # one "how big is this change" for the whole line (per point gains would bend the line into waves)
        resid = []
        for f, v in fit:
            old = tr[2].get(f)
            if old is not None:
                resid.append(((v - (old[0] + old[1] * dt)) / (self.fast_m * (1.0 + max(0.0, f) / 30.0))) ** 2)
        big = min(1.0, sum(resid) / len(resid)) if resid else 1.0
        state, out = {}, []
        for f, v in fit:
            old = tr[2].get(f)
            if old is None:
                state[f] = (v, 0.0)
                out.append((f, v))
                continue
            lat0, rate0 = old
            pred = lat0 + rate0 * dt
            r = v - pred
            u = min(1.0, max(0.0, f) / 90.0)
            a = self.alpha_near + (self.alpha_far - self.alpha_near) * u
            a = a + (1.0 - a) * big
            a = 1.0 - (1.0 - a) ** n
            b = self.beta_k * a * a
            lat = pred + a * r
            rate = rate0 + b * r / max(dt, 1e-3)
            state[f] = (lat, rate)
            out.append((f, lat))
        self.tracks[key] = (now, end, state)
        return self._fit(out, out[-1][0]) or out  # once more through the low-order fit: no waves along the line
