"""K9 D road path band / chevrons square to the path (대감님 11/13 20:28 "경로하고 예상추종커브나 똑바로 나오게").

Drawing only: the band edges are the path moved sideways along its own normal, so a bend keeps the band width
and a straight path is drawn exactly as before."""
import math
import sys
from pathlib import Path

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_renderer import ClusterUiRenderer as R


def test_straight_path_unchanged():
    path = [(float(f), 0.3) for f in range(0, 90, 5)]
    assert R.k9_path_side(path, -1.62) == [(f, 0.3 - 1.62) for f, _ in path]
    assert R.k9_path_side(path, 1.62) == [(f, 0.3 + 1.62) for f, _ in path]
    assert R.k9_path_normal(1.2, 0.0) == (0.0, 1.0)


def test_bend_keeps_band_width():
    k = 1.0 / 60.0  # 60 m radius left bend
    path = [(60.0 * math.sin(s * k), -60.0 * (1.0 - math.cos(s * k))) for s in [i * 2.0 for i in range(40)]]
    path = [p for p in path if p[0] <= 50.0]
    left, right = R.k9_path_side(path, -1.62), R.k9_path_side(path, 1.62)
    for f in (10.0, 25.0, 40.0):
        heading = math.asin(f / 60.0)
        span = R._k9_turzx_lat_at(right, f) - R._k9_turzx_lat_at(left, f)
        assert abs(span - 3.24 / math.cos(heading)) < 0.05  # width 3.24 m measured square to the path
    for side in (left, right):
        assert all(b[0] > a[0] for a, b in zip(side, side[1:], strict=False))


def test_normal_points_right_of_travel():
    nf, nl = R.k9_path_normal(1.0, -1.0)  # heading left
    assert nl > 0.0 and nf > 0.0 and abs(math.hypot(nf, nl) - 1.0) < 1e-9
