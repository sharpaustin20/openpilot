#!/usr/bin/env python3
# Read-only review of the driver's flags, car OFF: every tap of the onroad flag button (or the sidebar flag) is a
# userBookmark in the log. For each flag this prints what the car did from BEFORE_S before to AFTER_S after it: speed,
# lead, real gap (distance / speed, bumper to bumper, the 6 m stop distance included), the planner's request, the
# driver's pedals, FCW and alerts, and a one-line guess at what happened.
#   cd /data/openpilot && PYTHONPATH=/data/openpilot python3 system/hardware/test_job.py \
#     /usr/local/venv/bin/python3 tools/camry/flag_review.py [--last N | ROUTE ...] [--all-segments]
# About 4 lines per flag. Full 0.5 s timelines go to --out (default /data/camry_flags.json, merged by flag so earlier
# reviews are kept after the drive itself is deleted). Reads logs only; writes nothing but that file.
import argparse
import datetime
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

MPH = 2.237
STOP_DISTANCE = 6.0
REALDATA = "/data/media/0/realdata"
ONROAD_PARAM = "/data/params/d/IsOnroad"
PRESERVE_ATTR = "user.preserve"   # loggerd marks the segment of every userBookmark (system/loggerd/deleter.py)
BEFORE_S, AFTER_S = 20.0, 10.0    # the tap comes after the moment, so most of the window is before it
SEG_S = 60.0
PERSONALITY = {0: "Personalized/Aggressive", 1: "Standard", 2: "Relaxed"}
PLAN_SRC = {0: "cruise", 1: "lead0", 2: "lead1", 3: "lead2", 4: "e2e"}
SP_SRC = {0: "cruise", 1: "sccVision", 2: "sccMap", 3: "speedLimitAssist"}
HARD_A = -1.5      # m/s^2, a planner request under this is "hard braking"
BRAKE_A = -0.5


def onroad():
  try:
    with open(ONROAD_PARAM, "rb") as f:
      return f.read().strip() == b"1"
  except OSError:
    return False


def all_segments(root):
  """{route: {seg: path}} for every segment with an rlog."""
  out: dict = {}
  for d in os.listdir(root):
    if "--" not in d:
      continue
    r, s = d.rsplit("--", 1)
    if not s.isdigit():
      continue
    for name in ("rlog.zst", "rlog.bz2", "rlog"):
      p = os.path.join(root, d, name)
      if os.path.exists(p):
        out.setdefault(r, {})[int(s)] = p
        break
  return out


def preserved(root, route, seg):
  try:
    return os.getxattr(os.path.join(root, f"{route}--{seg}"), PRESERVE_ATTR) == b"1"
  except OSError:
    return False


def read_segment(path):
  """One segment -> dict(flags, buttons, ticks, fcw, alerts, clock). ticks at the longitudinalPlan rate (20 Hz)."""
  if onroad():
    return None
  from openpilot.tools.lib.logreader import LogReader
  cs = ss = rs = sp = cc = None
  out = dict(flags=[], buttons=[], ticks=[], fcw=[], alerts=[], clock=None, git=None)
  fcw_prev, alert_prev = False, ""
  for m in LogReader(path):
    w = m.which()
    t = m.logMonoTime * 1e-9
    if w == "carState":
      cs = m.carState
    elif w == "longitudinalPlan":
      if cs is None:
        continue
      lp = m.longitudinalPlan
      if lp.fcw and not fcw_prev:
        out["fcw"].append(t)
      fcw_prev = bool(lp.fcw)
      lead = rs.leadOne if rs is not None else None
      has_lead = bool(lead.status) if lead is not None else False
      out["ticks"].append((t, cs.vEgo, cs.aEgo, lp.aTarget, cc.actuators.accel if cc is not None else 0.0,
                           bool(ss.enabled) if ss is not None else False, cs.gasPressed, cs.brakePressed,
                           has_lead, lead.dRel if has_lead else 0.0, lead.vRel if has_lead else 0.0,
                           lp.longitudinalPlanSource.raw, sp.longitudinalPlanSource.raw if sp is not None else 0,
                           ss.personality.raw if ss is not None else 1, cs.vCruise / 3.6, bool(lp.hasLead)))
    elif w == "selfdriveState":
      ss = m.selfdriveState
      alert = ss.alertType if ss.alertStatus.raw != 0 else ""
      if alert and alert != alert_prev and not alert.startswith("userBookmark"):
        out["alerts"].append((t, alert.split("/")[0]))
      alert_prev = alert
    elif w == "radarState":
      rs = m.radarState
    elif w == "longitudinalPlanSP":
      sp = m.longitudinalPlanSP
    elif w == "carControl":
      cc = m.carControl
    elif w == "userBookmark":
      out["flags"].append(t)
    elif w == "bookmarkButton":
      out["buttons"].append(t)
    elif w == "clocks" and out["clock"] is None and m.clocks.wallTimeNanos > 0:
      out["clock"] = (t, m.clocks.wallTimeNanos * 1e-9)
    elif w == "initData":
      out["git"] = f"{m.initData.gitCommit[:7]} ({m.initData.gitBranch})"
  return out


def window(segs, t0):
  """Ticks, FCW and alerts from t0 - BEFORE_S to t0 + AFTER_S over the read segments of one route."""
  rows = [r for s in segs for r in s["ticks"] if t0 - BEFORE_S <= r[0] <= t0 + AFTER_S]
  fcw = [x - t0 for s in segs for x in s["fcw"] if t0 - BEFORE_S <= x <= t0 + AFTER_S]
  alerts = [(round(x - t0, 1), a) for s in segs for x, a in s["alerts"] if t0 - BEFORE_S <= x <= t0 + AFTER_S]
  return np.array(sorted(rows), dtype=float) if rows else None, fcw, alerts


def episodes(mask, t):
  """[(t_start, t_end)] of True runs in mask (joined across 1 s gaps)."""
  idx = np.flatnonzero(mask)
  runs = []
  for i in idx:
    if runs and t[i] - runs[-1][1] <= 1.0:
      runs[-1][1] = t[i]
    else:
      runs.append([t[i], t[i]])
  return [(round(a, 1), round(b, 1)) for a, b in runs]


def describe(B, t0, fcw, alerts):
  """Summary dict for one flag window B (columns as in read_segment)."""
  t = B[:, 0] - t0
  v, a_ego, a_tgt, eng, gas, brake = B[:, 1], B[:, 2], B[:, 3], B[:, 5] > 0, B[:, 6] > 0, B[:, 7] > 0
  lead, d, vrel, src, sp_src, pers = B[:, 8] > 0, B[:, 9], B[:, 10], B[:, 11], B[:, 12], B[:, 13]
  i0 = int(np.argmin(np.abs(t)))
  real = np.where(lead, d / np.maximum(v, 0.1), np.nan)
  before = t <= 0.5
  s = dict(
    mph_at_flag=round(float(v[i0] * MPH)), mph_min=round(float(v.min() * MPH)), mph_max=round(float(v.max() * MPH)),
    engaged_at_flag=bool(eng[i0]), engaged_pct=round(float(eng.mean() * 100)),
    personality=PERSONALITY.get(int(pers[i0]), "?"),
    plan_a_min=round(float(a_tgt[before].min()), 2) if before.any() else None,
    plan_a_min_at=round(float(t[before][np.argmin(a_tgt[before])]), 1) if before.any() else None,
    plan_a_max=round(float(a_tgt[before].max()), 2) if before.any() else None,
    car_a_min=round(float(a_ego.min()), 2),
    lead_at_flag=bool(lead[i0]),
    real_gap_at_flag=round(float(real[i0]), 2) if lead[i0] else None,
    real_gap_min=round(float(np.nanmin(real)), 2) if lead.any() else None,
    closing_max_mph=round(float(max(0.0, -vrel[lead].min()) * MPH), 1) if lead.any() else None,
    source_at_hardest=PLAN_SRC.get(int(src[before][np.argmin(a_tgt[before])]), "?") if before.any() else None,
    sp_source_at_hardest=SP_SRC.get(int(sp_src[before][np.argmin(a_tgt[before])]), "?") if before.any() else None,
    gas=episodes(gas, t), brake=episodes(brake, t), disengaged=episodes(~eng, t)[:3],
    fcw=[round(x, 1) for x in fcw], alerts=alerts[:5],
  )
  s["guess"] = guess(s, t, a_tgt, lead, eng)
  return s


def guess(s, t, a_tgt, lead, eng):
  """One line: the most likely thing the driver didn't like."""
  why = []
  if s["fcw"]:
    why.append(f"FCW at {s['fcw'][0]:+.1f} s")
  if s["plan_a_min"] is not None and s["plan_a_min"] < HARD_A:
    src = s["source_at_hardest"] if s["sp_source_at_hardest"] == "cruise" else s["sp_source_at_hardest"]
    who = "lead" if src and src.startswith("lead") else src
    why.append(f"hard braking {s['plan_a_min']} m/s2 at {s['plan_a_min_at']:+.1f} s (for {who})")
  elif s["plan_a_min"] is not None and s["plan_a_min"] < BRAKE_A:
    why.append(f"braking {s['plan_a_min']} m/s2 at {s['plan_a_min_at']:+.1f} s (for {s['source_at_hardest']})")
  # run-up: accelerating behind a lead, then braking within 8 s
  acc = (a_tgt > 0.3) & lead & eng & (t <= 0.5)
  brk = (a_tgt < BRAKE_A) & eng & (t <= 0.5)
  if acc.any() and brk.any():
    ta, tb = t[acc][0], t[brk][-1]
    if 0 < tb - ta < 8:
      why.append("sped up toward the lead, then braked")
  if s["real_gap_min"] is not None and s["real_gap_min"] < 0.9:
    why.append(f"close: real gap down to {s['real_gap_min']} s")
  if s["brake"] and any(a <= 0.5 for a, _ in s["brake"]):
    why.append(f"you braked at {s['brake'][0][0]:+.1f} s")
  if s["gas"] and any(a <= 0.5 for a, _ in s["gas"]):
    why.append(f"you pressed the gas at {s['gas'][0][0]:+.1f} s")
  if not s["engaged_at_flag"] and s["engaged_pct"] < 10:
    why.append("not engaged")
  return "; ".join(why) or "nothing stands out (calm request, no takeover): note what you saw"


def wall(seg, t):
  if seg["clock"] is None:
    return None
  mono, wall_s = seg["clock"]
  return datetime.datetime.fromtimestamp(wall_s + (t - mono), tz=datetime.UTC).isoformat(timespec="seconds")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="*")
  ap.add_argument("--root", default=REALDATA)
  ap.add_argument("--last", type=int, default=0, help="only the newest N routes (default: every route on the device)")
  ap.add_argument("--all-segments", action="store_true",
                  help="read every segment, not only the ones loggerd marked (slower; also finds flags that never " +
                       "became a userBookmark)")
  ap.add_argument("--out", default="/data/camry_flags.json")
  ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
  a = ap.parse_args()
  if onroad():
    sys.exit("car is onroad; run this with the car off")
  segs_by_route = all_segments(a.root)
  routes = sorted(a.routes) if a.routes else sorted(segs_by_route)[-a.last if a.last else 0:]
  # the marked segments plus their neighbors (a window can cross a segment boundary)
  todo = set()
  for r in routes:
    segs = segs_by_route.get(r, {})
    for n in segs:
      if a.all_segments or preserved(a.root, r, n):
        todo |= {(r, k) for k in (n - 1, n, n + 1) if k in segs}
  print(f"{len(routes)} routes, {len(todo)} segments to read ({'all' if a.all_segments else 'flagged + neighbors'}), " +
        f"{a.jobs} workers", flush=True)
  if not todo:
    print("no flagged segments found")
    return
  todo_l = sorted(todo)
  with Pool(a.jobs) as pool:
    res = pool.map(read_segment, [segs_by_route[r][n] for r, n in todo_l])
  if any(x is None for x in res):
    sys.exit("car turned on; stopped")
  by_route: dict = {}
  for (r, n), x in zip(todo_l, res, strict=True):
    by_route.setdefault(r, {})[n] = x

  try:
    with open(a.out) as f:
      saved = json.load(f)
  except (OSError, ValueError):
    saved = {}
  found, unrelayed = 0, 0
  for r in sorted(by_route):
    segs = by_route[r]
    all_reads = list(segs.values())
    for n in sorted(segs):
      seg = segs[n]
      start = seg["ticks"][0][0] if seg["ticks"] else None
      # a button press with no userBookmark within 1 s: feedbackd didn't pass it on
      unrelayed += sum(1 for b in seg["buttons"] if not any(0 <= f - b < 1.0 for s in all_reads for f in s["flags"]))
      for t0 in seg["flags"]:
        B, fcw, alerts = window(all_reads, t0)
        if B is None or len(B) < 10:
          continue
        found += 1
        into = n * SEG_S + (t0 - start if start is not None else 0.0)
        s = describe(B, t0, fcw, alerts)
        key = f"{r}--{n}@{into:.1f}"
        when = wall(seg, t0)
        s.update(route=r, segment=n, drive_time=f"{int(into // 60)}:{int(into % 60):02d}", utc=when, code=seg["git"])
        s["timeline"] = [dict(t=round(float(row[0] - t0), 1), mph=round(float(row[1] * MPH), 1), plan_a=round(float(row[3]), 2),
                              car_a=round(float(row[2]), 2), engaged=bool(row[5]), gas=bool(row[6]), brake=bool(row[7]),
                              real_gap=round(float(row[9] / max(row[1], 0.1)), 2) if row[8] else None,
                              closing_mph=round(float(-row[10] * MPH), 1) if row[8] else None)
                         for row in B[::10]]   # 0.5 s steps
        saved[key] = s
        gap = f"real gap {s['real_gap_at_flag']} s (min {s['real_gap_min']}), closing up to {s['closing_max_mph']} mph" \
          if s["lead_at_flag"] else "no lead at the tap"
        print(f"\n# {r} at {s['drive_time']}" + (f" ({when[11:16]} UTC)" if when else "") + f", {s['personality']}, "
              + ("engaged" if s["engaged_at_flag"] else "not engaged"))
        print(f"  {s['mph_at_flag']} mph (range {s['mph_min']}-{s['mph_max']}); {gap}")
        print(f"  planner {s['plan_a_min']}..{s['plan_a_max']} m/s2 in the {BEFORE_S:.0f} s before; car min {s['car_a_min']}; "
              + f"FCW {len(s['fcw'])}; alerts {', '.join(a_ for _, a_ in s['alerts']) or 'none'}")
        print(f"  likely: {s['guess']}")
  print(f"\n{found} flags" + (f"; {unrelayed} button presses never became a flag (feedbackd not running?)" if unrelayed else ""))
  tmp = a.out + ".tmp"
  with open(tmp, "w") as f:
    json.dump(saved, f, indent=1)
  os.replace(tmp, a.out)
  print(f"details: {a.out} ({len(saved)} flags kept)")


if __name__ == "__main__":
  main()
