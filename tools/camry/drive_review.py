#!/usr/bin/env python3
# Read-only review of a drive, car OFF: did the PR #8 planner run, how did following and glide behave, and where did
# sunnypilot brake only because it was over its target speed (the case for the overspeed-glide setting).
#   cd /data/openpilot && PYTHONPATH=/data/openpilot python3 system/hardware/test_job.py \
#     /usr/local/venv/bin/python3 /tmp/drive_review.py [--last N | ROUTE ...] [--out /tmp/drive_review.json]
# Prints about 40 lines; details (every overspeed episode) go to the JSON file. Writes nothing else.
import argparse
import ast
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

MPH = 2.237
STOP_DISTANCE = 6.0
REALDATA = "/data/media/0/realdata"
ONROAD_PARAM = "/data/params/d/IsOnroad"
SP_SRC = {0: "cruise", 1: "sccVision", 2: "sccMap", 3: "speedLimitAssist"}
PLAN_SRC = {0: "cruise", 1: "lead0", 2: "lead1", 3: "lead2", 4: "e2e"}
COAST_A = -0.35          # m/s^2, requests above this count as coasting (the Camry's own coast is about -0.3)
OVER_A = -0.3            # a request below this while over the target speed counts as overspeed braking
OVER_V = 0.3             # m/s over the target before it counts (0.7 mph)
RUNUP_A = 0.3            # m/s^2, an acceleration request above this behind a lead starts a possible run-up
RUNUP_S = 8.0            # s, braking this soon after the acceleration ends counts as a run-up
BINS = [(0, 20), (20, 35), (35, 50), (50, 65), (65, 200)]
# target gaps (s) by personality: Aggressive slot = PR #7 table before PR #8, the Personalized table after (the installed
# default, or the table in the route's settings log line)
PR7 = ([17.9, 26.8], [1.0, 1.45])
try:
  from openpilot.selfdrive.controls.lib import camry_personalized as _cp
  PR8_DEFAULT = ([b / MPH for b in _cp.DEFAULT_FOLLOW_BP_MPH], list(_cp.DEFAULT_FOLLOW_S))
except Exception:   # PR #8 not installed on this device: its first default table
  PR8_DEFAULT = ([35 / MPH, 40 / MPH, 60 / MPH], [0.8, 1.0, 1.45])


def personalized_table(pers_line):
  """The Personalized gap table (m/s breakpoints, s) a route ran with, from its 'camry personalized settings' log line
  (an override there wins), else the installed default; None when PR #8 was not running."""
  if not pers_line:
    return None
  try:
    msg = json.loads(pers_line).get("msg", pers_line)
  except Exception:
    msg = pers_line
  try:
    st = ast.literal_eval(msg.split("settings: ", 1)[1].strip())
    if st.get("follow_bp_mph") and st.get("follow_s"):
      return [b / MPH for b in st["follow_bp_mph"]], list(st["follow_s"])
  except Exception:
    pass
  return PR8_DEFAULT


def onroad():
  try:
    with open(ONROAD_PARAM, "rb") as f:
      return f.read().strip() == b"1"
  except OSError:
    return False


def segments(root, routes):
  segs = []
  for d in os.listdir(root):
    if "--" not in d:
      continue
    r, s = d.rsplit("--", 1)
    if r in routes and s.isdigit():
      for name in ("rlog.zst", "rlog.bz2", "rlog"):
        p = os.path.join(root, d, name)
        if os.path.exists(p):
          segs.append((r, int(s), p))
          break
  return sorted(segs)


def read_segment(path):
  """One segment -> (git, branch, personalized log line, ticks at longitudinalPlan rate, events: FCW onsets, alert onsets,
  glide failures)."""
  if onroad():
    return None
  from openpilot.tools.lib.logreader import LogReader
  git = branch = pers = None
  cs = cc = sp = rs = ss = ctl = None
  ticks = []
  ev = dict(fcw=[], alerts=[], glide_fail=[])
  fcw_prev, alert_prev = False, ""
  for m in LogReader(path):
    w = m.which()
    if w == "carState":
      cs = m.carState
    elif w == "longitudinalPlan":
      if cs is None or ctl is None:
        continue
      lp = m.longitudinalPlan
      if lp.fcw and not fcw_prev:
        ev["fcw"].append(m.logMonoTime * 1e-9)
      fcw_prev = bool(lp.fcw)
      pitch = float(cc.orientationNED[1]) if cc is not None and len(cc.orientationNED) == 3 else 0.0
      lead = rs.leadOne if rs is not None else None
      ticks.append((m.logMonoTime * 1e-9, cs.vEgo, cs.vCruise / 3.6, ctl.longControlState.raw != 0, cs.gasPressed, cs.brakePressed,
                    lp.aTarget, lp.longitudinalPlanSource.raw, lp.hasLead, lp.shouldStop,
                    sp.longitudinalPlanSource.raw if sp is not None else 0, sp.vTarget if sp is not None else 0.0,
                    sp.speedLimit.resolver.speedLimit if sp is not None else 0.0, sp.speedLimit.resolver.speedLimitOffset if sp is not None else 0.0,
                    bool(sp.speedLimit.resolver.speedLimitValid) if sp is not None else False,
                    bool(sp.speedLimit.assist.active) if sp is not None else False,
                    bool(sp.smartCruiseControl.vision.active or sp.smartCruiseControl.map.active) if sp is not None else False,
                    ss.personality.raw if ss is not None else 1, bool(ss.enabled) if ss is not None else False,
                    bool(lead.status) if lead is not None else False, lead.dRel if lead is not None else 0.0,
                    lead.vRel if lead is not None else 0.0, pitch))
    elif w == "controlsState":
      ctl = m.controlsState
    elif w == "carControl":
      cc = m.carControl
    elif w == "longitudinalPlanSP":
      sp = m.longitudinalPlanSP
    elif w == "radarState":
      rs = m.radarState
    elif w == "selfdriveState":
      ss = m.selfdriveState
      alert = ss.alertType if ss.alertStatus.raw != 0 else ""
      if alert and alert != alert_prev:
        ev["alerts"].append((m.logMonoTime * 1e-9, alert.split("/")[0]))
      alert_prev = alert
    elif w == "initData":
      git, branch = m.initData.gitCommit[:7], m.initData.gitBranch
    elif w == "logMessage" and pers is None and "camry personalized" in m.logMessage:
      pers = m.logMessage[:400]
    if w in ("logMessage", "errorLogMessage") and "glide off for this drive" in getattr(m, w):
      ev["glide_fail"].append((m.logMonoTime * 1e-9, getattr(m, w)[:300]))
  return git, branch, pers, ticks, ev


def t_follow(personality, v, table):
  if personality == 2:
    return 1.75
  if personality == 1:
    return 1.45
  bp, vals = table or PR7
  return float(np.interp(v, bp, vals))


def episodes(mask, t, max_gap=1.0):
  """[(i0, i1)] runs of True in mask, joined when the gap between them is at most max_gap seconds."""
  idx = np.flatnonzero(mask)
  if len(idx) == 0:
    return []
  runs, start, prev = [], idx[0], idx[0]
  for i in idx[1:]:
    if t[i] - t[prev] > max_gap:
      runs.append((start, prev))
      start = i
    prev = i
  runs.append((start, prev))
  return runs


def mmss(s):
  return f"{int(s // 60):d}:{int(s % 60):02d}"


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="*")
  ap.add_argument("--root", default=REALDATA)
  ap.add_argument("--last", type=int, default=1, help="newest N routes when none are given")
  ap.add_argument("--out", default="/tmp/drive_review.json")
  ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
  a = ap.parse_args()
  if onroad():
    sys.exit("car is onroad; run this with the car off")
  routes = set(a.routes)
  if not routes:
    names = sorted({d.rsplit("--", 1)[0] for d in os.listdir(a.root) if "--" in d})
    routes = set(names[-a.last:])
  segs = segments(a.root, routes)
  if not segs:
    sys.exit(f"no segments for {sorted(routes)} under {a.root}")
  print(f"routes {sorted(routes)}: {len(segs)} segments, {a.jobs} workers", flush=True)
  with Pool(a.jobs) as pool:
    results = pool.map(read_segment, [p for _, _, p in segs])
  if any(r is None for r in results):
    sys.exit("car turned on; stopped")

  # per route: time runs from the route's first tick; routes are laid end to end (a reboot resets the log clock)
  by_route = {}
  for (route, _, _), (git, branch, p, ticks, ev) in zip(segs, results, strict=True):
    r = by_route.setdefault(route, dict(code=set(), pr8=None, rows=[], fcw=[], alerts=[], glide_fail=[]))
    for k in ("fcw", "alerts", "glide_fail"):
      r[k] += ev[k]
    if git:
      r["code"].add(f"{git} ({branch})")
    r["pr8"] = r["pr8"] or p
    r["rows"] += ticks
  print("\n== install")
  blocks, spans, offset = [], [], 0.0
  events = dict(fcw=[], alerts=[], glide_fail=[])
  for n, (route, r) in enumerate(sorted(by_route.items()), 1):
    line = "YES: " + r["pr8"][:120] if r["pr8"] else "NOT found (PR #8 was not running on this route)"
    print(f"  route {n} {route}: code {', '.join(sorted(r['code'])) or 'unknown'}; PR #8 planner line: {line}")
    if r["rows"]:
      B = np.array(r["rows"], dtype=float)
      shift = offset - B[0, 0]
      B[:, 0] += shift
      events["fcw"] += [x + shift for x in r["fcw"]]
      events["alerts"] += [(x + shift, a) for x, a in r["alerts"]]
      events["glide_fail"] += [(x + shift, msg) for x, msg in r["glide_fail"]]
      spans.append((n, offset, B[-1, 0]))
      table = personalized_table(r["pr8"])
      tf = [[t_follow(int(p), v, table)] for p, v in zip(B[:, 17], B[:, 1], strict=True)]   # the route's own target gaps
      blocks.append(np.hstack([B, np.full((len(B), 1), float(r["pr8"] is not None)), np.array(tf)]))
      offset = B[-1, 0] + 60.0
  if not blocks:
    sys.exit("no longitudinalPlan data")
  A = np.vstack(blocks)
  t = A[:, 0]
  dt = np.diff(t, append=t[-1] + 0.05).clip(0, 0.2)
  v, long_on, gas, brake = A[:, 1], A[:, 3] > 0.5, A[:, 4] > 0.5, A[:, 5] > 0.5
  a_tgt, src, stop = A[:, 6], A[:, 7].astype(int), A[:, 9] > 0.5
  sp_src, v_tgt, lim, lim_off = A[:, 10].astype(int), A[:, 11], A[:, 12], A[:, 13]
  lim_ok, sla_on = A[:, 14] > 0.5, A[:, 15] > 0.5
  pers_raw, enabled, l_ok, d_rel, v_rel, pitch = A[:, 17].astype(int), A[:, 18] > 0.5, A[:, 19] > 0.5, A[:, 20], A[:, 21], A[:, 22]
  pr8_tick, tf = A[:, 23] > 0.5, A[:, 24]
  eng = long_on & enabled

  def when(i):
    for n, lo, hi in spans:
      if lo <= t[i] <= hi:
        return f"route {n} {mmss(t[i] - lo)}"
    return mmss(t[i])

  by_pers = ", ".join(f"{n} {mmss(dt[eng & (pers_raw == k)].sum())}" for k, n in ((0, 'personalized/aggressive'), (1, 'standard'), (2, 'relaxed')))
  print(f"\n== drive: {mmss(dt.sum())} total, engaged {mmss(dt[eng].sum())}, {(dt[eng] * v[eng]).sum() / 1609:.1f} mi engaged; {by_pers}")

  def when_t(x):
    return when(min(int(np.searchsorted(t, x)), len(t) - 1))

  # health: forward collision warnings, alerts shown, glide switching itself off after an error
  print(f"\n== health: FCW {len(events['fcw'])}" + (f" (at {', '.join(when_t(x) for x in events['fcw'][:6])})" if events["fcw"] else "")
        + f"; glide errors {len(events['glide_fail'])}")
  for x, msg in events["glide_fail"][:3]:
    print(f"  glide error at {when_t(x)}: {msg}")
  counts = {}
  for _, a_name in events["alerts"]:
    counts[a_name] = counts.get(a_name, 0) + 1
  if counts:
    print("  alerts shown: " + ", ".join(f"{k} {n}" for k, n in sorted(counts.items(), key=lambda kv: -kv[1])[:12]))

  # following
  fol = eng & (src == 1) & l_ok & (v > 2.0)
  gap = (d_rel - STOP_DISTANCE) / np.maximum(v, 0.1)
  print(f"\n== following (engaged, lead is the limiting source): {mmss(dt[fol].sum())}")
  for lo, hi in BINS:
    m = fol & (v * MPH >= lo) & (v * MPH < hi)
    if dt[m].sum() > 10:
      brk_pct = dt[m & (a_tgt < -0.5)].sum() / max(dt[m].sum(), 0.01) * 100
      print(f"  {lo:>2}-{hi:<3} mph: {mmss(dt[m].sum())}, gap median {np.median(gap[m]):.2f} s (target {np.median(tf[m]):.2f}), "
            + f"braking under -0.5: {brk_pct:.0f}% of the time")
  band = fol & (gap < tf) & (gap >= tf - 0.2) & (v_rel > -2.0)
  below = fol & (gap < tf - 0.2) & (gap >= tf - 0.45) & (v_rel > -2.0)
  for name, m in (("coast band (0-0.2 s under target, not closing fast)", band), ("light band (0.2-0.45 s under)", below)):
    tot = dt[m].sum()
    if tot > 0:
      coast_pct, brake_pct = dt[m & (a_tgt >= COAST_A)].sum() / tot * 100, dt[m & (a_tgt < COAST_A)].sum() / tot * 100
      print(f"  {name}: {mmss(tot)}; coasting {coast_pct:.0f}%, braking harder than -0.35: {brake_pct:.0f}%"
            + f" (glide {'on' if pr8_tick[m].all() else 'on for part of it' if pr8_tick[m].any() else 'not installed'})")
  prev_eng = np.roll(eng, 1)
  gas_on = eng & gas & ~np.roll(gas, 1) & l_ok
  brk_on = prev_eng & brake & ~np.roll(brake, 1)
  per_hour = (gas_on.sum() + brk_on.sum()) / max(dt[eng].sum() / 3600, 0.01)
  print(f"  gas presses with a lead while engaged: {int(gas_on.sum())}; brake takeovers: {int(brk_on.sum())}; per engaged hour: {per_hour:.1f}")

  # each takeover with its context: was it the routine end of an engaged stretch (slowing below what the PCM allows),
  # or did the driver step in while following (closing on the lead, or under the target gap)?
  takeovers = []
  for kind, idx in (("brake", np.flatnonzero(brk_on)), ("gas", np.flatnonzero(gas_on))):
    for i in idx:
      j0, j1 = np.searchsorted(t, t[i] - 3.0), np.searchsorted(t, t[i] + 20.0)
      gap_i = float(gap[i]) if l_ok[i] else None
      closing = float(-v_rel[i]) if l_ok[i] else 0.0
      v_min_after = float(v[i:j1].min() * MPH)
      if kind == "brake" and v_min_after < 20.0:
        why = "slowing below 20 mph (end of a stretch)"
      elif l_ok[i] and closing > 1.0 and gap_i is not None and gap_i < tf[i] + 0.3:
        why = "closing on the lead"
      elif l_ok[i] and gap_i is not None and gap_i < tf[i] - 0.2:
        why = "under the target gap"
      elif l_ok[i]:
        why = "behind a lead, not close"
      else:
        why = "no lead"
      takeovers.append(dict(kind=kind, at=when(i), why=why, v_mph=round(float(v[i] * MPH), 1), v_min_20s_mph=round(v_min_after, 1),
                            gap_s=None if gap_i is None else round(gap_i, 2), target_s=round(float(tf[i]), 2),
                            closing_ms=round(closing, 1), min_a_3s=round(float(a_tgt[j0:i + 1].min()), 2),
                            source=PLAN_SRC.get(int(src[i]), "?")))
  if takeovers:
    counts = {}
    for k in takeovers:
      counts[(k["kind"], k["why"])] = counts.get((k["kind"], k["why"]), 0) + 1
    print("  takeovers by cause: " + "; ".join(f"{kind} {why}: {n}" for (kind, why), n in sorted(counts.items())))
    for k in [k for k in takeovers if not k["why"].startswith("slowing")][:12]:
      gap_txt = f"gap {k['gap_s']} s (target {k['target_s']}), closing {k['closing_ms']} m/s" if k["gap_s"] is not None else "no lead"
      print(f"    {k['kind']} at {k['at']}: {k['why']}, {k['v_mph']} mph, {gap_txt}, planner min {k['min_a_3s']} m/s2 in the 3 s before "
            + f"({k['source']})")

  # run-ups: engaged behind a lead, accelerating (request over RUNUP_A) and then braking (under -0.5) within RUNUP_S of the
  # end of that acceleration, with the same lead (no distance jump of 4 m or more between ticks: a cut-in is not a run-up)
  runups = []
  for i0, i1 in episodes(eng & l_ok & (a_tgt > RUNUP_A), t):
    j1 = int(np.searchsorted(t, t[i1] + RUNUP_S))
    brk = np.flatnonzero(eng[i1:j1] & l_ok[i1:j1] & (a_tgt[i1:j1] < -0.5))
    if len(brk) == 0:
      continue
    k = i1 + int(brk[0])
    if not l_ok[i0:k + 1].all() or (np.abs(np.diff(d_rel[i0:k + 1])) >= 4.0).any():
      continue
    runups.append(dict(at=when(i0), v_mph=round(float(v[i1] * MPH), 1), peak_a=round(float(a_tgt[i0:i1 + 1].max()), 2),
                       gap_s=round(float(gap[i1]), 2), target_s=round(float(tf[i1]), 2), closing_ms=round(float(-v_rel[i1]), 1),
                       brake_after_s=round(float(t[k] - t[i1]), 1), min_a=round(float(a_tgt[k:j1].min()), 2)))
  print(f"  run-ups (accelerating behind a lead, then braking under -0.5 within {RUNUP_S:.0f} s): {len(runups)}")
  for r in runups[:8]:
    print(f"    at {r['at']}: {r['v_mph']} mph, accel up to {r['peak_a']}, then braking {r['brake_after_s']} s later to {r['min_a']} m/s2; "
          + f"gap {r['gap_s']} s (target {r['target_s']}), closing {r['closing_ms']} m/s when the accel ended")

  # overspeed braking: engaged, no lead limiting, over the target speed, planner asks for braking
  over = v - v_tgt
  base = eng & (src == 0) & ~stop & (over > OVER_V) & (a_tgt < OVER_A)
  glideable = base & ((sp_src == 0) | (sp_src == 3))
  curve = base & ((sp_src == 1) | (sp_src == 2))
  e2e_over = eng & (src == 4) & (over > OVER_V) & (a_tgt < OVER_A)
  print(f"\n== overspeed braking (engaged, no lead, over the target speed, request under {OVER_A} m/s2)")
  print(f"  set speed / speed limit target: {mmss(dt[glideable].sum())}; curve target (not for glide): {mmss(dt[curve].sum())}; "
        + f"end-to-end model: {mmss(dt[e2e_over].sum())}; speed limit assist active {mmss(dt[eng & sla_on].sum())} of the drive")
  for x in (3, 5, 8):
    print(f"  X = {x} mph would cover {mmss(dt[glideable & (over * MPH <= x)].sum())} of it")
  eps = []
  for i0, i1 in episodes(glideable, t):
    m = np.zeros(len(t), bool)
    m[i0:i1 + 1] = True
    m &= glideable
    limit_mph = round(float(np.median(lim[m])) * MPH, 1) if lim_ok[m].any() else None
    eps.append(dict(start=when(i0), dur_s=round(float(dt[m].sum()), 1), over_mph=round(float(over[m].max() * MPH), 1),
                    v_mph=round(float(v[m].max() * MPH), 1), target_mph=round(float(np.median(v_tgt[m]) * MPH), 1),
                    source=SP_SRC.get(int(np.median(sp_src[m])), "?"), limit_mph=limit_mph, offset=round(float(np.median(lim_off[m])), 2),
                    min_a=round(float(a_tgt[m].min()), 2), pitch_deg=round(float(np.degrees(pitch[m].mean())), 1)))
  eps.sort(key=lambda e: -e["dur_s"])
  print(f"  {len(eps)} episode(s); longest:")
  for e in eps[:8]:
    slope = 'downhill' if e['pitch_deg'] < -0.5 else 'uphill' if e['pitch_deg'] > 0.5 else 'flat'
    limit = f", limit {e['limit_mph']} +{e['offset']}" if e['limit_mph'] is not None else ""
    print(f"    at {e['start']}: {e['dur_s']} s, {e['v_mph']} mph vs target {e['target_mph']} ({e['source']}{limit}), "
          + f"over by {e['over_mph']} mph, braking to {e['min_a']} m/s2, pitch {e['pitch_deg']} deg ({slope})")
  drops = eng & (sp_src == 3) & (np.roll(v_tgt, 20) - v_tgt > 2.0) & (t - np.roll(t, 20) < 2.0)
  print(f"  speed-limit target drops of more than 4.5 mph while engaged: {len(episodes(drops, t))}")

  with open(a.out, "w") as f:
    json.dump(dict(routes=sorted(routes), code=sorted(set().union(*(r['code'] for r in by_route.values()))),
                   pr8_line={k: r['pr8'] for k, r in by_route.items()}, engaged_s=float(dt[eng].sum()),
                   overspeed_s=float(dt[glideable].sum()), curve_s=float(dt[curve].sum()), episodes=eps, takeovers=takeovers, runups=runups,
                   fcw=[when_t(x) for x in events['fcw']], alerts=[(when_t(x), a_name) for x, a_name in events['alerts']],
                   glide_errors=[(when_t(x), msg) for x, msg in events['glide_fail']]), f, indent=1)
  print(f"\nwritten: {a.out}")


if __name__ == "__main__":
  main()
