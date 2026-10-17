"""tools/k9_stop_soft_brake_replay.py 를 따로 띄운 파이썬으로 돌려 종료코드와 결과를 본다.

- --help 는 0, 필수 인수가 빠지면 argparse 인수 오류(2) 그대로.
- PARAMS_ROOT 가 이미 있는 폴더·파일·심볼릭 링크·끊어진 링크면 0 이 아닌 종료, 원본 내용·링크 대상은 그대로.
- 작은 재생 입력(워커의 실제 기록에서 몇 초만 잘라 임시 폴더에 만든 rlog.zst)은 0 으로 끝나고
  출력 파일에 재생 행(card·longcontrol)과 송신 결과(sends·dec)가 들어 있다. 실제 기록이 없으면 건너뛴다.
출력 파일과 PARAMS_ROOT 는 시험마다 새 임시 경로를 쓴다.
"""
import os
import pickle
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[4]
TOOL = ROOT / "tools/k9_stop_soft_brake_replay.py"

# 워커의 실제 기록(읽기만 한다). 저장소에는 넣지 않는다.
REAL_SCHEMA = Path("/home/ssm-user/clawd/work/k9_stop_bounce_cause_20261014/schema_00000181/cereal")
REAL_SEG = Path("/home/ssm-user/clawd/work/k9_stop_bounce_cause_20261014/logs/00000181--fa2d554836--0")
STOP_T = 105136967555  # 이 구간의 정차 시각(logMonoTime)


def run_tool(args, params_root):
  env = dict(os.environ, PARAMS_ROOT=str(params_root))
  return subprocess.run([sys.executable, str(TOOL), *args], env=env, cwd=ROOT, capture_output=True, text=True,
                        timeout=600)


def test_help_exits_0(tmp_path):
  r = run_tool(["--help"], tmp_path / "params")
  assert r.returncode == 0, r.stderr
  assert "--schema" in r.stdout and "--time" in r.stdout
  assert not (tmp_path / "params").exists()


@pytest.mark.parametrize("args", [[], ["--schema", "x", "--out", "y.pkl"]])
def test_missing_required_args_is_argparse_error(tmp_path, args):
  out = tmp_path / "out.pkl"
  args = [a if a != "y.pkl" else str(out) for a in args]
  r = run_tool(args, tmp_path / "params")
  assert r.returncode == 2, r.stderr
  assert "the following arguments are required" in r.stderr
  assert "ModuleNotFoundError" not in r.stderr
  assert not (tmp_path / "params").exists() and not out.exists()


def make_existing(kind, tmp_path):
  """이미 있는 PARAMS_ROOT 하나와 그 원본 상태를 확인하는 함수를 만든다."""
  p = tmp_path / "params"
  if kind == "dir":
    p.mkdir()
    (p / "keep").write_bytes(b"dir-original")
    return p, lambda: p.is_dir() and not p.is_symlink() and sorted(os.listdir(p)) == ["keep"] and \
      (p / "keep").read_bytes() == b"dir-original"
  if kind == "file":
    p.write_bytes(b"file-original")
    return p, lambda: p.is_file() and not p.is_symlink() and p.read_bytes() == b"file-original"
  if kind == "symlink":
    target = tmp_path / "target"
    target.mkdir()
    (target / "keep").write_bytes(b"target-original")
    p.symlink_to(target)
    return p, lambda: os.readlink(p) == str(target) and sorted(os.listdir(target)) == ["keep"] and \
      (target / "keep").read_bytes() == b"target-original"
  # 끊어진 링크: 가리키는 곳이 없고, 실행 뒤에도 생기지 않아야 한다.
  target = tmp_path / "missing_target"
  p.symlink_to(target)
  return p, lambda: p.is_symlink() and os.readlink(p) == str(target) and not os.path.lexists(target)


@pytest.mark.parametrize("kind", ["dir", "file", "symlink", "dangling_symlink"])
def test_existing_params_root_is_refused_and_left_untouched(tmp_path, kind):
  params, unchanged = make_existing(kind, tmp_path)
  assert unchanged()
  out = tmp_path / "out.pkl"
  # PARAMS_ROOT 검사는 기록을 읽기 전에 하므로 스키마·구간은 없는 경로여도 된다.
  r = run_tool(["--schema", str(tmp_path / "no_schema"), "--out", str(out), "--start", "0", "--end", "1",
                str(tmp_path / "no_seg")], params)
  assert r.returncode != 0
  assert "refusing PARAMS_ROOT" in r.stderr, r.stderr
  assert unchanged()
  assert not out.exists()


def make_small_input(seg_dir, lo, hi):
  """실제 기록에서 initData·carParams 와 [lo, hi] 안의 메시지만 골라 작은 rlog.zst 를 만든다."""
  capnp = pytest.importorskip("capnp")
  zstandard = pytest.importorskip("zstandard")
  capnp.remove_import_hook()
  log = capnp.load(str(REAL_SCHEMA / "log.capnp"), imports=[str(REAL_SCHEMA), str(REAL_SCHEMA / "include")])
  data = zstandard.ZstdDecompressor().decompressobj().decompress((REAL_SEG / "rlog.zst").read_bytes())
  kept, counts = [], {}
  try:
    for ev in log.Event.read_multiple_bytes(data):
      w = ev.which()
      if w in ("initData", "carParams") and w not in counts or lo <= ev.logMonoTime <= hi:
        counts[w] = counts.get(w, 0) + 1
        kept.append(ev.as_builder().to_bytes())
  except capnp.KjException:
    pass  # 끊긴 구간 끝
  seg_dir.mkdir()
  (seg_dir / "rlog.zst").write_bytes(zstandard.ZstdCompressor().compress(b"".join(kept)))
  return counts


@pytest.mark.skipif(not (REAL_SEG / "rlog.zst").is_file() or not (REAL_SCHEMA / "log.capnp").is_file(),
                    reason="워커의 실제 기록(00000181--fa2d554836--0)이 없어 작은 재생 입력을 만들 수 없음")
def test_small_input_replay_writes_rows_and_sends(tmp_path):
  seg = tmp_path / "seg"
  counts = make_small_input(seg, STOP_T - 4_000_000_000, STOP_T + 2_000_000_000)
  assert counts.get("initData") == 1 and counts.get("carParams", 0) >= 1 and counts.get("sendcan", 0) > 0
  out, params = tmp_path / "out.pkl", tmp_path / "params"
  start, end = STOP_T - 2_000_000_000, STOP_T + 2_000_000_000
  r = run_tool(["--schema", str(REAL_SCHEMA), "--out", str(out), "--time", "20", "--start", str(start),
                "--end", str(end), str(seg)], params)
  assert r.returncode == 0, r.stderr
  assert params.is_dir()  # 새 경로는 도구가 만든다
  with open(out, "rb") as fh:
    d = pickle.load(fh)
  assert d["cp"]["fp"] == "KIA_K9" and d["time"] == "20"
  assert d["overrides"]["K9StopSoftBrakeTime"] == "20"
  card, lc = d["card"], d["longcontrol"]
  assert len(card) > 100 and len(lc) > 100
  assert all(start <= row["t"] <= end for row in card + lc)
  # 송신 결과: 걸음마다 만든 CAN 송신이 있고, SCC12/SCC14 를 풀어 읽은 값이 들어 있다.
  assert all(row["sends"] for row in card)
  decoded = [row["dec"] for row in card if "SCC12" in row["dec"]]
  assert len(decoded) > 50
  assert all(set(dec["SCC12"]) == {"ACCMode", "StopReq", "aReqRaw", "aReqValue"} for dec in decoded)
  assert any("SCC14" in row["dec"] for row in card)
  assert any(dec["SCC12"]["StopReq"] == 1 for dec in decoded)  # 창 안에서 정차 명령까지 재생됨
