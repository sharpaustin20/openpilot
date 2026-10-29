# Read-only: does the driving model notice the lead car braking (brake lights, nose dip) before the radar does?
# For every drive, compares the model's predicted lead deceleration (modelV2.leadsV3[0].a) with the radar
# Kalman deceleration the planner actually uses (radarState.leadOne.aLeadK), and prints:
#   - real braking events (radar aLeadK < -1.0 m/s^2): how many the model caught, and how many seconds earlier
#   - false alarms: model says "lead braking" but the radar never sees it within 3 s, per hour of following,
#     and how many of those happened while sunnypilot was actually giving gas (the only time a "no gas" cap acts)
# for several model thresholds, so the trigger can be chosen from real drives.
#
# Run with the car OFF (refuses to start, and stops, if the comma goes onroad), from the Mac:
#   ssh comma@<ip> python3 /data/openpilot/system/hardware/test_job.py env PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 - [route ...] < brake_light_study.py
# Options: --root DIR (default /data/media/0/realdata), --out FILE (default /tmp/brake_light_study.json), --jobs N,
#          --last N (only the newest N routes, default 20), --all (every route on the device)
import argparse, json, os, sys
from collections import defaultdict
from multiprocessing import Pool

sys.path.insert(0, "/data/openpilot")

TICK = 0.05                    # s, modelV2 / radarState are 20 Hz
RADAR_BRAKE = -1.0             # m/s^2, radar aLeadK below this = the lead is really braking
RADAR_CONFIRM = -0.75          # m/s^2, a model alarm counts as real if the radar gets this low within CONFIRM_S
CONFIRM_S = 3.0
LOOKBACK_S = 3.0               # s before a radar event in which a model alarm counts as "caught early"
HOLDS_S = (0.25, 0.5)          # model must stay below the threshold this long to raise an alarm
THRESHOLDS = (-0.5, -0.75, -1.0, -1.5)
MIN_SPEED = 5.0                # m/s
GAS_ACCEL = 0.2                # m/s^2, carControl accel above this = sunnypilot was giving gas
ONROAD_PARAM = "/data/params/d/IsOnroad"


def onroad():
  try:
    with open(ONROAD_PARAM, "rb") as f:
      return f.read().strip() == b"1"
  except OSError:
    return False


def seg_file(root, route, s):
  for name in ("rlog.zst", "rlog.bz2", "rlog"):
    p = os.path.join(root, f"{route}--{s}", name)
    if os.path.exists(p):
      return p
  return None


# ---------- extraction: one segment -> 20 Hz ticks ----------

def extract_seg(args):
  root, route, s = args
  if onroad():
    return route, s, [], "stopped: car went onroad"
  f = seg_file(root, route, s)
  if f is None:
    return route, s, [], ""
  from openpilot.tools.lib.logreader import LogReader
  cur = dict(v=0.0, long=False, accel=0.0, lead=False, radar=False, alead=0.0, d=0.0)
  ticks = []
  try:
    for m in LogReader(f):
      w = m.which()
      if w == "carState":
        cur["v"] = m.carState.vEgo
      elif w == "carControl":
        cur["long"] = m.carControl.longActive
        cur["accel"] = m.carControl.actuators.accel
      elif w == "radarState":
        l = m.radarState.leadOne
        cur.update(lead=bool(l.status), radar=bool(l.radar), alead=l.aLeadK, d=l.dRel)
      elif w == "modelV2":
        ld = m.modelV2.leadsV3
        if len(ld) == 0:
          continue
        a = list(ld[0].a)
        ticks.append((cur["v"], cur["long"], cur["accel"], cur["lead"], cur["radar"], cur["alead"], cur["d"],
                      ld[0].prob, a[0] if a else 0.0, a[1] if len(a) > 1 else 0.0))
  except Exception as e:
    return route, s, ticks, f"read error: {e}"
  return route, s, ticks, ""


# ---------- analysis: one route ----------

def runs(flags, hold):
  """Start indexes of runs of True at least `hold` ticks long."""
  out, start = [], None
  for i, f in enumerate(flags + [False]):
    if f and start is None:
      start = i
    elif not f and start is not None:
      if i - start >= hold:
        out.append(start + hold - 1)        # alarm fires once the hold is met
      start = None
  return out


def analyze(ticks):
  V, LONG, ACC, LEAD, RADAR, ALEAD, D, PROB, A0, A1 = range(10)
  ok = [t[LEAD] and t[RADAR] and t[V] > MIN_SPEED and t[PROB] > 0.5 for t in ticks]
  follow_s = sum(ok) * TICK
  # radar braking events: first tick of each stretch with aLeadK < RADAR_BRAKE
  ev, prev = [], False
  for i, t in enumerate(ticks):
    b = ok[i] and t[ALEAD] < RADAR_BRAKE
    if b and not prev:
      ev.append(i)
    prev = b
  look, conf = int(LOOKBACK_S / TICK), int(CONFIRM_S / TICK)
  res = {"follow_s": round(follow_s, 1), "radar_events": len(ev), "signals": {}}
  for sig, col in (("a_now", A0), ("a_2s", A1)):
    for thr in THRESHOLDS:
      for hold_s in HOLDS_S:
        alarms = runs([ok[i] and t[col] < thr for i, t in enumerate(ticks)], int(hold_s / TICK))
        lead_times, caught = [], 0
        for e in ev:
          early = [a for a in alarms if e - look <= a <= e]
          if early:
            caught += 1
            lead_times.append((e - early[0]) * TICK)
        false = [a for a in alarms if not any(ticks[j][ALEAD] < RADAR_CONFIRM for j in range(a, min(len(ticks), a + conf)))]
        false_gas = [a for a in false if ticks[a][LONG] and ticks[a][ACC] > GAS_ACCEL]
        res["signals"][f"{sig} {thr} hold {hold_s}"] = dict(alarms=len(alarms), caught=caught, lead_times=lead_times,
                                                          false=len(false), false_gas=len(false_gas))
  return res


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="*")
  ap.add_argument("--root", default="/data/media/0/realdata")
  ap.add_argument("--out", default="/tmp/brake_light_study.json")
  ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
  ap.add_argument("--last", type=int, default=20)
  ap.add_argument("--all", action="store_true")
  args = ap.parse_args()
  if onroad():
    sys.exit("car is onroad: not starting")

  segs = defaultdict(list)
  for name in os.listdir(args.root):
    if "--" in name:
      route, _, s = name.rpartition("--")
      if s.isdigit():
        segs[route].append(int(s))
  routes = args.routes or sorted(segs)
  if not args.routes and not args.all:
    routes = routes[-args.last:]
  work = [(args.root, r, s) for r in routes for s in sorted(segs.get(r, []))]
  print(f"{len(routes)} routes, {len(work)} segments, {args.jobs} workers", flush=True)

  by_route, notes = defaultdict(dict), []
  with Pool(args.jobs) as pool:
    for route, s, ticks, note in pool.imap_unordered(extract_seg, work):
      by_route[route][s] = ticks
      if note:
        notes.append(f"{route}--{s}: {note}")
        if note.startswith("stopped"):
          pool.terminate()
          break

  total = dict(follow_s=0.0, radar_events=0, signals=defaultdict(lambda: dict(alarms=0, caught=0, lead_times=[], false=0, false_gas=0)))
  per_route = {}
  for route in sorted(by_route):
    ticks = [t for s in sorted(by_route[route]) for t in by_route[route][s]]   # segments joined in order
    r = analyze(ticks)
    per_route[route] = r
    total["follow_s"] += r["follow_s"]
    total["radar_events"] += r["radar_events"]
    for k, v in r["signals"].items():
      for f in ("alarms", "caught", "false", "false_gas"):
        total["signals"][k][f] += v[f]
      total["signals"][k]["lead_times"] += v["lead_times"]

  hrs = max(total["follow_s"] / 3600, 1e-6)
  print(f"following a radar lead: {total['follow_s'] / 60:.0f} min, radar braking events (< {RADAR_BRAKE} m/s^2): {total['radar_events']}")
  print("signal / threshold / hold      caught  median-early  false/h  false-while-gas/h")
  summary = {}
  for k, v in total["signals"].items():
    lt = sorted(v["lead_times"])
    med = lt[len(lt) // 2] if lt else 0.0
    row = dict(caught=f"{v['caught']}/{total['radar_events']}", median_early_s=round(med, 2),
               false_per_h=round(v["false"] / hrs, 1), false_gas_per_h=round(v["false_gas"] / hrs, 1))
    summary[k] = row
    print(f"{k:30s} {row['caught']:>7s}  {med:9.2f} s  {row['false_per_h']:7.1f}  {row['false_gas_per_h']:9.1f}")
  for n in notes[:10]:
    print("note:", n)
  with open(args.out, "w") as f:
    json.dump(dict(summary=summary, per_route=per_route, notes=notes), f, indent=1)
  print(f"full results: {args.out}")


if __name__ == "__main__":
  main()
