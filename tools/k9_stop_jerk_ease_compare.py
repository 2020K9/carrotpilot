#!/usr/bin/env python3
"""Compare k9_stop_jerk_ease_replay.py outputs: base tree vs K9StopJerkEase 0/1/2.

usage: python k9_stop_jerk_ease_compare.py --out-dir <dir> NAME=<base.pkl>,<new.pkl>,<stop logMonoTime ns> [...]
Writes <NAME>_frames.csv (every generated SCC frame side by side) and summary.json.
Reports transmitted values only; this is not vehicle response, braking delay or comfort evidence.
"""
import argparse
import csv
import json
import os
import pickle

SCC14_ADDR = 905
JERK_UPPER_MASK = ((1 << 7) - 1) << 12  # SCC14 JerkUpperLimit: start bit 12, 7 bits, little endian


def runs(flags):
  """Longest run of consecutive True values."""
  best = cur = 0
  for f in flags:
    cur = cur + 1 if f else 0
    best = max(best, cur)
  return best


def compare(name, base_pkl, new_pkl, stop_t, out_dir):
  base = pickle.load(open(base_pkl, "rb"))["base"]["rows"]
  new = pickle.load(open(new_pkl, "rb"))
  modes = {m: new[m]["rows"] for m in ("0", "1", "2")}
  assert all(len(r) == len(base) for r in modes.values())
  for r in modes.values():
    assert all(a["t"] == b["t"] for a, b in zip(base, r, strict=True))

  summary = {"name": name, "steps": len(base), "git_recorded": pickle.load(open(base_pkl, "rb"))["git"]}

  # Recording vs base replay (fidelity of the harness, recorded code version may differ).
  fid = {}
  for x in base:
    for msg, sigs in x["rec"].items():
      if msg not in x["dec"]:
        fid[f"{msg}.missing"] = fid.get(f"{msg}.missing", 0) + 1
        continue
      fid[f"{msg}.frames"] = fid.get(f"{msg}.frames", 0) + 1
      for s, v in sigs.items():
        if abs(v - x["dec"][msg][s]) > 1e-6:
          fid[f"{msg}.{s}"] = fid.get(f"{msg}.{s}", 0) + 1
  summary["recorded_vs_base_mismatch"] = fid

  # Off must reproduce every transmitted message of the base tree.
  summary["off_all_messages_identical"] = all(a["sends"] == b["sends"] for a, b in zip(base, modes["0"], strict=True))

  scc = [i for i, x in enumerate(base) if "SCC12" in x["dec"]]
  for m in ("1", "2"):
    rows = modes[m]
    other_diff = 0
    for a, b in zip(base, rows, strict=True):
      sa = [(addr, dat if addr != SCC14_ADDR else int.from_bytes(dat, "little") & ~JERK_UPPER_MASK, bus) for addr, dat, bus in a["sends"]]
      sb = [(addr, dat if addr != SCC14_ADDR else int.from_bytes(dat, "little") & ~JERK_UPPER_MASK, bus) for addr, dat, bus in b["sends"]]
      other_diff += sa != sb
    ju_b = [base[i]["dec"]["SCC14"]["JerkUpperLimit"] for i in scc]
    ju_m = [rows[i]["dec"]["SCC14"]["JerkUpperLimit"] for i in scc]
    ju_diff = [abs(x - y) > 1e-6 for x, y in zip(ju_b, ju_m, strict=True)]
    higher = sum(y > x + 1e-6 for x, y in zip(ju_b, ju_m, strict=True))

    # Stronger deceleration than base in either aReq field (this scope keeps aReq unchanged).
    stronger = []
    for i in scc:
      for s in ("aReqRaw", "aReqValue"):
        vb, vm = base[i]["dec"]["SCC12"][s], rows[i]["dec"]["SCC12"][s]
        if vm < vb - 1e-6:
          stronger.append(dict(t=(base[i]["t"] - stop_t) / 1e9, signal=s, base=vb, mode=vm, diff=vm - vb))
    areq_changed = sum(any(abs(base[i]["dec"]["SCC12"][s] - rows[i]["dec"]["SCC12"][s]) > 1e-6
                           for s in ("aReqRaw", "aReqValue")) for i in scc)
    stronger_flags = [any(abs(base[i]["dec"]["SCC12"][s] - rows[i]["dec"]["SCC12"][s]) > 1e-6 and
                          rows[i]["dec"]["SCC12"][s] < base[i]["dec"]["SCC12"][s] for s in ("aReqRaw", "aReqValue")) for i in scc]

    # Risk candidate: base command weakening (aReqValue rising) while this mode's Upper is lower.
    risk = []
    for k in range(1, len(scc)):
      i, j = scc[k], scc[k - 1]
      weakening = base[i]["dec"]["SCC12"]["aReqValue"] > base[j]["dec"]["SCC12"]["aReqValue"] + 1e-6
      lower = ju_m[k] < ju_b[k] - 1e-6
      risk.append(weakening and lower)
    # Same, with weakening judged over a 0.2 s look-back (10 sent frames) to ignore 0.01 quantization steps.
    risk_slow = []
    for k in range(len(scc)):
      j = scc[max(0, k - 10)]
      weakening = base[scc[k]]["dec"]["SCC12"]["aReqValue"] > base[j]["dec"]["SCC12"]["aReqValue"] + 1e-6
      risk_slow.append(weakening and ju_m[k] < ju_b[k] - 1e-6)
    risk_frames = [dict(t=(base[scc[k + 1]]["t"] - stop_t) / 1e9, aReqValue=base[scc[k + 1]]["dec"]["SCC12"]["aReqValue"],
                        ju_base=ju_b[k + 1], ju_mode=ju_m[k + 1]) for k, f in enumerate(risk) if f]

    # Limit residue: Upper differs where the sent StopReq is 0, or with driver input / disengaged.
    residue = []
    for k, i in enumerate(scc):
      x = base[i]
      released = (x["dec"]["SCC12"]["StopReq"] == 0 or x["gas"] or x["brake"] or x["override"] or not x["enabled"])
      if released and ju_diff[k]:
        residue.append((x["t"] - stop_t) / 1e9)

    # Onsets of the sent StopReq and the limited span after each.
    onsets = []
    for k in range(1, len(scc)):
      if base[scc[k]]["dec"]["SCC12"]["StopReq"] == 1 and base[scc[k - 1]]["dec"]["SCC12"]["StopReq"] == 0:
        span = 0
        while k + span < len(scc) and ju_diff[k + span]:
          span += 1
        onsets.append(dict(t=(base[scc[k]]["t"] - stop_t) / 1e9, ju_prev=ju_b[k - 1], ju_base=ju_b[k], ju_mode=ju_m[k],
                           limited_frames=span, max_reduction=max([ju_b[k + q] - ju_m[k + q] for q in range(span)], default=0.0)))

    summary[f"mode{m}"] = dict(
      non_upper_message_diff_steps=other_diff, upper_diff_frames=sum(ju_diff), upper_higher_than_base=higher,
      areq_changed_frames=areq_changed, stronger_decel_frames=len(stronger), stronger_decel_longest_run=runs(stronger_flags),
      stronger_decel_longest_run_s=runs(stronger_flags) * 0.02, stronger_decel=stronger[:20],
      fail_stronger_decel=bool(stronger), risk_candidate_frames=len(risk_frames), risk_candidate_longest_run=runs(risk),
      risk_candidate=risk_frames, risk_candidate_0p2s_frames=sum(risk_slow), risk_candidate_0p2s_longest_run=runs(risk_slow),
      risk_candidate_0p2s_t=[(base[scc[k]]["t"] - stop_t) / 1e9 for k, f in enumerate(risk_slow) if f][:20], limit_residue_frames=len(residue), limit_residue_t=residue[:20], onsets=onsets,
    )

  with open(os.path.join(out_dir, f"{name}_frames.csv"), "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t_rel_s", "vEgo", "aEgo", "longControlState", "enabled", "gas", "brake", "override", "softHold",
                "StopReq_base", "StopReq_m1", "StopReq_m2", "aReqRaw_base", "aReqRaw_m1", "aReqRaw_m2",
                "aReqValue_base", "aReqValue_m1", "aReqValue_m2", "JerkUpper_base", "JerkUpper_off", "JerkUpper_m1", "JerkUpper_m2",
                "JerkLower_base", "CBUpper_base", "CBUpper_m1", "CBUpper_m2", "CBLower_base", "CBLower_m1", "CBLower_m2"])
    for i in scc:
      b, o, m1, m2 = base[i], modes["0"][i], modes["1"][i], modes["2"][i]
      d = lambda x, msg, s: x["dec"][msg][s]  # noqa: E731
      w.writerow([f"{(b['t'] - stop_t) / 1e9:.3f}", f"{b['vEgo']:.3f}", f"{b['aEgo']:.3f}", b["lcs"], int(b["enabled"]), int(b["gas"]),
                  int(b["brake"]), int(b["override"]), b["softHold"],
                  *(int(d(x, "SCC12", "StopReq")) for x in (b, m1, m2)),
                  *(f"{d(x, 'SCC12', 'aReqRaw'):.2f}" for x in (b, m1, m2)),
                  *(f"{d(x, 'SCC12', 'aReqValue'):.2f}" for x in (b, m1, m2)),
                  *(f"{d(x, 'SCC14', 'JerkUpperLimit'):.1f}" for x in (b, o, m1, m2)),
                  f"{d(b, 'SCC14', 'JerkLowerLimit'):.1f}",
                  *(f"{d(x, 'SCC14', 'ComfortBandUpper'):.2f}" for x in (b, m1, m2)),
                  *(f"{d(x, 'SCC14', 'ComfortBandLower'):.2f}" for x in (b, m1, m2))])
  return summary


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--out-dir", required=True)
  ap.add_argument("cases", nargs="+")
  args = ap.parse_args()
  out = []
  for case in args.cases:
    name, rest = case.split("=", 1)
    base_pkl, new_pkl, stop_t = rest.split(",")
    out.append(compare(name, base_pkl, new_pkl, int(stop_t), args.out_dir))
  with open(os.path.join(args.out_dir, "summary.json"), "w") as fh:
    json.dump(out, fh, indent=1, ensure_ascii=False)


if __name__ == "__main__":
  main()
