#!/usr/bin/env python3
"""Compare k9_stop_soft_brake_replay.py outputs: base tree 131f8b1c (--time base, StopEaseEnable=0) vs the modified
tree with the trial off (--time 0), normally with --stop-ease-leftover (removed StopEase values still stored).

usage: python k9_stop_soft_brake_compare.py [--out <summary.json>] NAME=<base.pkl>,<new.pkl> [...]
Off must reproduce the base exactly: every transmitted message (all addresses/bytes, including SCC11/12/14 and
StopReq), softHoldActive, cruise set speed/activation, and every LongControl output (accel, aTarget, jerk, state).
Exit code: 2 if a case cannot be evaluated (wrong tree/setting, base not run with StopEaseEnable=0, no steps,
mismatched steps, no sent SCC12 or StopReq=1 frame), 1 if anything differs or the trial produced a soft brake while off, 0 otherwise.
Transmitted values only; not vehicle response.
"""
import argparse
import json
import pickle
import sys

CARD_KEYS = ("sends", "soft_hold", "v_cruise", "activate")
LC_KEYS = ("accel", "aTarget", "jerk", "state")


def compare(name, base_pkl, new_pkl):
  base, new = pickle.load(open(base_pkl, "rb")), pickle.load(open(new_pkl, "rb"))
  s = dict(name=name, git_recorded=base.get("git"), unevaluable=[], differences={})
  if base.get("time") != "base" or base.get("has_trial"):
    s["unevaluable"].append("base pkl is not a base-tree --time base replay")
  if base.get("overrides", {}).get("StopEaseEnable") != "0":
    s["unevaluable"].append("base replay did not force StopEaseEnable=0")
  s["new_stop_ease_leftover"] = {k: v for k, v in new.get("overrides", {}).items() if k.startswith("StopEase")}
  s["recorded_stop_ease"] = base.get("recorded_stop_ease")
  if new.get("time") != "0" or not new.get("has_trial"):
    s["unevaluable"].append("new pkl is not a modified-tree --time 0 replay")
  for kind in ("card", "longcontrol"):
    b, n = base.get(kind, []), new.get(kind, [])
    if not b:
      s["unevaluable"].append(f"no {kind} steps")
    if [r["t"] for r in b] != [r["t"] for r in n]:
      s["unevaluable"].append(f"{kind} steps do not match")
  scc12 = [r["dec"]["SCC12"] for r in base.get("card", []) if "SCC12" in r["dec"]]
  if not scc12:
    s["unevaluable"].append("no sent SCC12 frame")
  elif not any(x["StopReq"] == 1 for x in scc12):
    s["unevaluable"].append("no sent StopReq=1 frame")
  if s["unevaluable"]:
    return s

  s["card_steps"], s["longcontrol_steps"], s["scc12_frames"] = len(base["card"]), len(base["longcontrol"]), len(scc12)
  s["stopreq_frames"] = sum(x["StopReq"] == 1 for x in scc12)
  s["soft_hold_frames"] = sum(r["soft_hold"] > 0 for r in base["card"])
  for kind, keys in (("card", CARD_KEYS), ("longcontrol", LC_KEYS)):
    for key in keys:
      diff = [r["t"] for r, q in zip(base[kind], new[kind], strict=True) if r[key] != q[key]]
      if diff:
        s["differences"][f"{kind}.{key}"] = dict(count=len(diff), first_t=diff[:5])
  active = [r["t"] for r in new["card"] if r["soft_brake"] is not None]
  if active:
    s["differences"]["card.soft_brake_while_off"] = dict(count=len(active), first_t=active[:5])
  return s


def main(argv=None):
  ap = argparse.ArgumentParser()
  ap.add_argument("--out")
  ap.add_argument("cases", nargs="+")
  args = ap.parse_args(argv)
  out = []
  for case in args.cases:
    name, rest = case.split("=", 1)
    base_pkl, new_pkl = rest.split(",")
    out.append(compare(name, base_pkl, new_pkl))
  if args.out:
    with open(args.out, "w") as fh:
      json.dump(out, fh, indent=1, ensure_ascii=False)
  for x in out:
    if x["unevaluable"]:
      print(f"{x['name']}: UNEVALUABLE {'; '.join(x['unevaluable'])}")
    elif x["differences"]:
      print(f"{x['name']}: DIFFERENT {json.dumps(x['differences'])}")
    else:
      print(f"{x['name']}: IDENTICAL card_steps={x['card_steps']} scc12={x['scc12_frames']} stopreq1={x['stopreq_frames']} "
            f"soft_hold={x['soft_hold_frames']} longcontrol_steps={x['longcontrol_steps']} "
            f"new_leftover={x['new_stop_ease_leftover'] or 'none'} recorded={x['recorded_stop_ease'] or 'none'}")
  code = 2 if any(x["unevaluable"] for x in out) else 1 if any(x["differences"] for x in out) else 0
  print(f"result: {'IDENTICAL' if code == 0 else 'UNEVALUABLE' if code == 2 else 'DIFFERENT'} cases={len(out)} exit={code}")
  return code


if __name__ == "__main__":
  sys.exit(main())
