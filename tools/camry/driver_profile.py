# Read-only: learn Austin's following style from recorded drives.
# Finds (1) manual following (long control off), (2) gas overrides while engaged, (3) brake takeovers,
# (4) distance-button / personality changes. Prints a short summary; full detail goes to a JSON file.
#
# Run with the car OFF (refuses to start, and stops, if the comma goes onroad):
#   cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 /tmp/driver_profile.py [route ...]
# On the Mac (logs copied to a folder of <route>--<seg>/rlog.zst dirs):
#   PYTHONPATH=~/openpilot python3 driver_profile.py --root ~/radar_work/logs [route ...]
# Options: --root DIR (default /data/media/0/realdata), --out FILE (default /tmp/driver_profile.json), --jobs N
import argparse, json, os, sys
from collections import defaultdict
from multiprocessing import Pool

STOP_DISTANCE = 6.0      # m, sunnypilot long_mpc.py: steady-state gap = t_follow * v + STOP_DISTANCE
TICK = 0.05              # s, analysis rate (radarState is 20 Hz)
MPH = 2.237
SPEED_BINS = [(0, 20), (20, 35), (35, 50), (50, 65), (65, 200)]   # mph
PERSONALITY = {0: "aggressive", 1: "standard", 2: "relaxed"}
ONROAD_PARAM = "/data/params/d/IsOnroad"


def onroad():
  try:
    with open(ONROAD_PARAM, "rb") as f:
      return f.read().strip() == b"1"
  except OSError:
    return False  # not on a comma (Mac) or param missing


def seg_file(root, route, s):
  for name in ("rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
    p = os.path.join(root, f"{route}--{s}", name)
    if os.path.exists(p):
      return p
  return None


# ---------- extraction: log -> flat 20 Hz ticks ----------

def extract_route(args):
  """Returns (route, ticks, note). Each tick is a dict of the latest values at that time."""
  root, route, segs = args
  from openpilot.tools.lib.logreader import LogReader
  ticks, cur, last_t, note, errors, seen = [], {}, None, "", 0, set()
  cur.update(v=0.0, a=0.0, gas=False, brake=False, blinker=False, steer=False, cruise=False,
             long_active=False, sd_state="disabled", enabled=False, personality=None, a_target=None,
             plan_fcw=False, lead=False, d=0.0, vrel=0.0, vlead=0.0, alead=0.0, yrel=0.0, prob=0.0,
             radar=False, gap_btn=False)
  for s in segs:
    if onroad():
      return route, ticks, "stopped: car went onroad"
    f = seg_file(root, route, s)
    if f is None:
      continue
    if "qlog" in os.path.basename(f):
      note = "some segments are qlog only (10x fewer carState samples)"
    try:
      for m in LogReader(f):
        w = m.which()
        if w in ("carControl", "selfdriveState", "longitudinalPlan", "radarState"):
          seen.add(w)
        if w == "carState":
          cs = m.carState
          cur.update(v=cs.vEgo, a=cs.aEgo, gas=cs.gasPressed, brake=cs.brakePressed,
                     blinker=cs.leftBlinker or cs.rightBlinker, steer=cs.steeringPressed,
                     cruise=cs.cruiseState.enabled)
          if any(str(b.type) == "gapAdjustCruise" and b.pressed for b in cs.buttonEvents):
            cur["gap_btn"] = True
          t = m.logMonoTime / 1e9
          # wait until this segment has state from every source, so segment starts don't look like transitions
          if len(seen) == 4 and (last_t is None or t - last_t >= TICK):
            ticks.append(dict(cur, t=t, seg=s))
            cur["gap_btn"] = False
            last_t = t
        elif w == "carControl":
          cur["long_active"] = m.carControl.longActive
        elif w == "selfdriveState":
          sd = m.selfdriveState
          cur.update(sd_state=str(sd.state), enabled=sd.enabled, personality=sd.personality.raw)
        elif w == "longitudinalPlan":
          lp = m.longitudinalPlan
          cur.update(a_target=lp.aTarget, plan_fcw=lp.fcw)
        elif w == "radarState":
          l = m.radarState.leadOne
          cur.update(lead=l.status, d=l.dRel, vrel=l.vRel, vlead=l.vLead, alead=l.aLeadK,
                     yrel=l.yRel, prob=l.modelProb, radar=l.radar)
    except Exception as e:
      errors += 1
      note = f"{errors} segment read errors, last in seg {s}: {e}"
  return route, ticks, note


# ---------- analysis: pure functions over ticks ----------

def good_lead(k):
  return k["lead"] and k["prob"] > 0.5 and 2.0 < k["d"] < 120.0 and abs(k["yrel"]) < 2.0 and k["v"] > 4.0


def t_equiv(k):
  """Gap expressed as sunnypilot's t_follow (what the planner setting means)."""
  return (k["d"] - STOP_DISTANCE) / max(k["v"], 0.1)


def ttc(k):
  return k["d"] / -k["vrel"] if k["vrel"] < -0.3 else None


def mode(k):
  if k["long_active"] and not k["gas"]:
    return "engaged"
  if k["enabled"] and k["gas"]:
    return "gas_override"
  if not k["enabled"] and not k["cruise"]:
    return "manual"
  return "other"


def same_lead_window(ticks, i0, i1):
  """True if the lead stays the same car (no big distance jump = cut-in/out) and no blinker."""
  i0, i1 = max(i0, 0), min(i1, len(ticks) - 1)
  for j in range(i0 + 1, i1 + 1):
    a, b = ticks[j - 1], ticks[j]
    if not good_lead(b) or b["blinker"] or abs(b["d"] - a["d"]) > 4.0:
      return False
  return True


def pct(xs, p):
  if not xs:
    return None
  xs = sorted(xs)
  return round(xs[min(len(xs) - 1, int(p / 100 * len(xs)))], 2)


def bin_of(v):
  mph = v * MPH
  for lo, hi in SPEED_BINS:
    if lo <= mph < hi:
      return f"{lo}-{hi if hi < 200 else '+'}"
  return None


def analyze(route, ticks):
  out = dict(route=route, seconds=round(len(ticks) * TICK), steady_gap=defaultdict(lambda: defaultdict(list)),
             manual_accel_lead_away=[], manual_brake_onsets=[], manual_coast_onsets=[],
             gas_overrides=[], brake_takeovers=[], personality_changes=[], mode_seconds=defaultdict(float))
  n = len(ticks)
  W = int(3.0 / TICK)   # 3 s windows
  for i, k in enumerate(ticks):
    md = mode(k)
    out["mode_seconds"][md] += TICK
    prev = ticks[i - 1] if i else k

    # steady following: same lead, matched speed, not accelerating hard
    if good_lead(k) and abs(k["vrel"]) < 1.0 and abs(k["a"]) < 0.4 and not k["blinker"] and md in ("manual", "engaged"):
      b = bin_of(k["v"])
      if b and i % 4 == 0 and same_lead_window(ticks, i - 20, i):   # 5 Hz samples, lead stable for 1 s
        out["steady_gap"][md][b].append(round(t_equiv(k), 3))

    if md == "manual" and good_lead(k):
      if k["vrel"] > 1.0 and k["gas"] and i % 4 == 0:
        out["manual_accel_lead_away"].append(round(k["a"], 2))
      # brake onset with a lead closing in
      if k["brake"] and not prev["brake"] and k["vrel"] < -0.3 and same_lead_window(ticks, i - W, i):
        after = [x["a"] for x in ticks[i:i + W]]
        out["manual_brake_onsets"].append(dict(t=round(k["t"], 1), mph=round(k["v"] * MPH), gap_s=round(t_equiv(k), 2),
                                               ttc=round(ttc(k), 1) if ttc(k) else None, min_a=round(min(after), 2)))
      # lift off the gas while closing (start of a coast)
      if prev["gas"] and not k["gas"] and not k["brake"] and k["vrel"] < -0.3 and same_lead_window(ticks, i - W, i):
        out["manual_coast_onsets"].append(dict(t=round(k["t"], 1), mph=round(k["v"] * MPH), gap_s=round(t_equiv(k), 2),
                                               ttc=round(ttc(k), 1) if ttc(k) else None))

    # gas override while engaged: start of a press
    if k["enabled"] and k["gas"] and not prev["gas"]:
      j = i
      while j < n - 1 and ticks[j]["gas"] and ticks[j]["enabled"]:
        j += 1
      seg = ticks[i:j + 1]
      ok = good_lead(k) and same_lead_window(ticks, i - W, j)
      out["gas_overrides"].append(dict(
        t=round(k["t"], 1), seg=k["seg"], dur=round((j - i) * TICK, 1), mph=round(k["v"] * MPH),
        gap_s=round(t_equiv(k), 2) if good_lead(k) else None, vrel=round(k["vrel"], 1) if good_lead(k) else None,
        planner_a=round(prev["a_target"], 2) if prev["a_target"] is not None else None,
        driver_max_a=round(max(x["a"] for x in seg), 2),
        gap_s_end=round(t_equiv(ticks[j]), 2) if good_lead(ticks[j]) else None, same_lead=ok))

    # brake takeover: was engaged, now disengaged with the brake down
    if prev["enabled"] and not k["enabled"] and k["brake"]:
      after = [x["a"] for x in ticks[i:i + W]]
      ok = good_lead(k) and same_lead_window(ticks, i - W, i)
      out["brake_takeovers"].append(dict(
        t=round(k["t"], 1), seg=k["seg"], mph=round(k["v"] * MPH),
        gap_s=round(t_equiv(k), 2) if good_lead(k) else None, vrel=round(k["vrel"], 1) if good_lead(k) else None,
        ttc=round(ttc(k), 1) if good_lead(k) and ttc(k) else None,
        planner_a=round(prev["a_target"], 2) if prev["a_target"] is not None else None,
        driver_min_a=round(min(after), 2), fcw_before=any(x["plan_fcw"] for x in ticks[max(0, i - W):i]), same_lead=ok))

    if k["personality"] is not None and prev["personality"] is not None and k["personality"] != prev["personality"]:
      out["personality_changes"].append(dict(t=round(k["t"], 1), mph=round(k["v"] * MPH),
                                             to=PERSONALITY.get(k["personality"], k["personality"]),
                                             gap_s=round(t_equiv(k), 2) if good_lead(k) else None))
  return out


def merge(results):
  tot = dict(routes=[], seconds=0, steady_gap=defaultdict(lambda: defaultdict(list)), manual_accel_lead_away=[],
             manual_brake_onsets=[], manual_coast_onsets=[], gas_overrides=[], brake_takeovers=[],
             personality_changes=[], mode_seconds=defaultdict(float))
  for r in results:
    tot["routes"].append(r["route"])
    tot["seconds"] += r["seconds"]
    for md, bins in r["steady_gap"].items():
      for b, xs in bins.items():
        tot["steady_gap"][md][b] += xs
    for key in ("manual_accel_lead_away", "manual_brake_onsets", "manual_coast_onsets", "gas_overrides",
                "brake_takeovers", "personality_changes"):
      tot[key] += [dict(x, route=r["route"]) if isinstance(x, dict) else x for x in r[key]]
    for md, s in r["mode_seconds"].items():
      tot["mode_seconds"][md] += s
  return tot


def summary(tot):
  L = [f"routes {len(tot['routes'])}, {tot['seconds'] / 60:.0f} min; time by mode (min): "
       + ", ".join(f"{m} {s / 60:.0f}" for m, s in sorted(tot["mode_seconds"].items()))]
  L.append("steady following gap as sunnypilot t_follow (s): median [p25-p75] (samples)")
  for md in ("manual", "engaged"):
    bins = tot["steady_gap"].get(md, {})
    row = []
    for lo, hi in SPEED_BINS:
      b = f"{lo}-{hi if hi < 200 else '+'}"
      xs = bins.get(b, [])
      row.append(f"{b}mph {pct(xs, 50)} [{pct(xs, 25)}-{pct(xs, 75)}] ({len(xs)})" if len(xs) >= 10 else f"{b}mph -")
    L.append(f"  {md:8s} " + " | ".join(row))
  acc = tot["manual_accel_lead_away"]
  if acc:
    L.append(f"manual accel when lead pulls away (m/s^2): median {pct(acc, 50)}, p90 {pct(acc, 90)} ({len(acc)})")
  for key, label in (("manual_brake_onsets", "manual brake onset"), ("manual_coast_onsets", "manual coast (lift) onset")):
    ev = tot[key]
    if ev:
      L.append(f"{label}: {len(ev)} events, TTC median {pct([e['ttc'] for e in ev if e['ttc']], 50)} s, "
               f"gap median {pct([e['gap_s'] for e in ev], 50)} s"
               + (f", hardest decel median {pct([e['min_a'] for e in ev], 50)}" if "min_a" in ev[0] else ""))
  for key, label in (("gas_overrides", "gas overrides"), ("brake_takeovers", "brake takeovers")):
    ev = tot[key]
    ok = [e for e in ev if e["same_lead"]]
    L.append(f"{label}: {len(ev)} total, {len(ok)} with the same in-lane lead (candidates to confirm)")
    for e in ok[:8]:
      L.append("  " + ", ".join(f"{k}={v}" for k, v in e.items() if k not in ("same_lead",)))
  pc = tot["personality_changes"]
  if pc:
    L.append("distance button changes: " + "; ".join(f"{p['mph']}mph->{p['to']}" for p in pc[:10]))
  return "\n".join(L)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="*")
  ap.add_argument("--root", default="/data/media/0/realdata")
  ap.add_argument("--out", default="/tmp/driver_profile.json")
  ap.add_argument("--jobs", type=int, default=os.cpu_count())
  a = ap.parse_args()
  if onroad():
    sys.exit("car is onroad; run this with the car off")
  by_route = defaultdict(list)
  for d in os.listdir(a.root):
    if "--" in d:
      r, s = d.rsplit("--", 1)
      if s.isdigit() and (not a.routes or r in a.routes):
        by_route[r].append(int(s))
  # one job per segment so every core stays busy; segments are stitched back in order per route
  jobs = [(a.root, r, [s]) for r, segs in sorted(by_route.items()) for s in sorted(segs)]
  print(f"{len(by_route)} routes, {len(jobs)} segments, {a.jobs} workers", flush=True)
  per_seg = defaultdict(dict)
  notes = defaultdict(list)
  with Pool(a.jobs) as pool:
    for (route, ticks, note), (_, _, (s,)) in zip(pool.imap(extract_route, jobs), jobs):
      per_seg[route][s] = ticks
      if note:
        notes[route].append(note)
  results = []
  for route in sorted(per_seg):
    ticks = [k for s in sorted(per_seg[route]) for k in per_seg[route][s]]
    print(f"  {route}: {len(ticks) * TICK / 60:.1f} min {'; '.join(notes[route][:2])}", flush=True)
    results.append(analyze(route, ticks))
  tot = merge(results)
  with open(a.out, "w") as f:
    json.dump(tot, f, indent=1)
  print(summary(tot))
  print(f"full detail: {a.out}")


if __name__ == "__main__":
  main()
