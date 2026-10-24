# Read-only: learn Austin's following style from recorded drives.
# Finds (1) manual following (long control off), (2) gas overrides while engaged, (3) brake takeovers,
# (4) distance-button / personality changes, (5) coasts: slowing with no pedal pressed behind a lead, followed to
# their end, (6) how hard sunnypilot asks before the PCM actually brakes (0x3B7 BRAKE_LIGHTS_ACC vs the request;
# probe_3b7.py found no such bit on this Camry on 2026-10-24, so (6) stays empty until another signal is found).
# Prints a short summary; full detail goes to a JSON file (tools/camry/suggest_long.py turns it into suggestions).
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
try:   # the Personalized profile (camry_personalized.py) takes the Aggressive slot when switched on
  from openpilot.selfdrive.controls.lib.camry_personalized import read_settings as _read_long_settings
  if _read_long_settings().personalized:
    PERSONALITY[0] = "personalized"
except Exception:
  pass
COAST_MAX_S = 20.0       # s, a coast is followed at most this long
PCM_BINS = [round(-1.5 + 0.1 * i, 1) for i in range(0, 18)]   # m/s^2 request bins for the brake-light check
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
             long_active=False, lat_active=False, sd_state="disabled", enabled=False, personality=None, a_target=None,
             plan_fcw=False, lead=False, d=0.0, vrel=0.0, vlead=0.0, alead=0.0, yrel=0.0, prob=0.0,
             radar=False, gap_btn=False, req_a=None, pcm_brake=None)
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
        elif w == "initData" and "git" not in note:
          note = (note + " " if note else "") + f"git {m.initData.gitCommit[:7]}"
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
          cur["lat_active"] = m.carControl.latActive
          cur["req_a"] = m.carControl.actuators.accel
        elif w == "can":
          for c in m.can:
            # ACC brake lights per the Toyota DBC: 1 bit (Motorola 18 -> byte 2, bit 2). Never set on this Camry
            # (probe_3b7.py, 2026-10-24: no 0x3B7 bit tracks braking on any bus), kept for when a real signal is found.
            if c.address == 0x3B7 and c.src == 0 and len(c.dat) >= 3:
              cur["pcm_brake"] = bool(c.dat[2] & 0x04)
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
      note = f"{note} {errors} segment read errors, last in seg {s}: {e}".strip()
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
  # cruise (PCM) engaged is what "long engaged" means on this car; MADS can keep lateral on without it
  if k["cruise"] and k["gas"]:
    return "gas_override"
  if not k["cruise"]:
    return "manual_lat" if k["lat_active"] else "manual"   # manual_lat: sunnypilot steering, driver on the pedals
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
             gas_overrides=[], brake_takeovers=[], personality_changes=[], mode_seconds=defaultdict(float),
             coasts=[], pcm_brake=defaultdict(lambda: [0, 0]))
  n = len(ticks)
  coast_end = -1   # index where the coast being followed ended, so lift-offs inside it aren't counted twice
  W = int(3.0 / TICK)   # 3 s windows
  for i, k in enumerate(ticks):
    md = mode(k)
    out["mode_seconds"][md] += TICK
    if md in ("engaged", "gas_override"):
      out["mode_seconds"][f"engaged_{PERSONALITY.get(k['personality'], 'unknown')}"] += TICK
    prev = ticks[i - 1] if i else k

    # does the PCM brake for this request level? (engaged, no pedal, request known, bit seen)
    if md == "engaged" and k["req_a"] is not None and k["pcm_brake"] is not None and k["v"] > 4.0 and i % 2 == 0:
      b = min(PCM_BINS, key=lambda x: abs(x - k["req_a"]))
      if abs(b - k["req_a"]) <= 0.05:
        out["pcm_brake"][str(b)][0 if k["pcm_brake"] else 1] += 1

    # coast: lift off the gas behind a lead with no pedal, followed to its end (brake, gas, lead lost/away, timeout)
    if md in ("manual", "manual_lat") and prev["gas"] and not k["gas"] and not k["brake"] and good_lead(k) and i > coast_end:
      j = i
      end = "timeout"
      while j < n - 1 and (j - i) * TICK < COAST_MAX_S:
        j += 1
        x = ticks[j]
        if x["brake"]:
          end = "brake"; break
        if x["gas"]:
          end = "gas"; break
        if not good_lead(x) or x["blinker"] or abs(x["d"] - ticks[j - 1]["d"]) > 4.0:
          end = "lead lost"; break
        if x["vrel"] > 0.5 and (j - i) * TICK > 1.0:
          end = "lead away"; break
      coast_end = j
      dur = (j - i) * TICK
      if dur >= 1.0:
        gaps = [t_equiv(x) for x in ticks[i:j + 1]]
        out["coasts"].append(dict(t=round(k["t"], 1), seg=k["seg"], mph=round(k["v"] * MPH), dur=round(dur, 1),
                                  gap_s=round(gaps[0], 2), min_gap_s=round(min(gaps), 2), end_gap_s=round(gaps[-1], 2),
                                  vrel=round(k["vrel"], 1), ttc=round(ttc(k), 1) if ttc(k) else None,
                                  decel=round((ticks[j]["v"] - k["v"]) / dur, 2), end=end,
                                  end_mph=round(ticks[j]["v"] * MPH)))

    # steady following: same lead, matched speed, not accelerating hard
    if good_lead(k) and abs(k["vrel"]) < 1.0 and abs(k["a"]) < 0.4 and not k["blinker"] and md in ("manual", "manual_lat", "engaged"):
      b = bin_of(k["v"])
      if b and i % 4 == 0 and same_lead_window(ticks, i - 20, i):   # 5 Hz samples, lead stable for 1 s
        key = md if md.startswith("manual") else f"engaged_{PERSONALITY.get(k['personality'], 'unknown')}"
        out["steady_gap"][key][b].append(round(t_equiv(k), 3))

    if md in ("manual", "manual_lat") and good_lead(k):
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
    if k["cruise"] and k["gas"] and not prev["gas"]:
      j = i
      while j < n - 1 and ticks[j]["gas"] and ticks[j]["cruise"]:
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
    if prev["cruise"] and not k["cruise"] and k["brake"]:
      after = [x["a"] for x in ticks[i:i + W]]
      ok = good_lead(k) and same_lead_window(ticks, i - W, i)
      out["brake_takeovers"].append(dict(
        t=round(k["t"], 1), seg=k["seg"], mph=round(k["v"] * MPH),
        gap_s=round(t_equiv(k), 2) if good_lead(k) else None, vrel=round(k["vrel"], 1) if good_lead(k) else None,
        ttc=round(ttc(k), 1) if good_lead(k) and ttc(k) else None,
        planner_a=round(prev["a_target"], 2) if prev["a_target"] is not None else None,
        driver_min_a=round(min(after), 2), fcw_before=any(x["plan_fcw"] for x in ticks[max(0, i - W):i]), same_lead=ok,
        personality=prev["personality"]))

    if k["personality"] is not None and prev["personality"] is not None and k["personality"] != prev["personality"]:
      out["personality_changes"].append(dict(t=round(k["t"], 1), mph=round(k["v"] * MPH),
                                             to=PERSONALITY.get(k["personality"], k["personality"]),
                                             gap_s=round(t_equiv(k), 2) if good_lead(k) else None))
  return out


def merge(results):
  tot = dict(routes=[], seconds=0, steady_gap=defaultdict(lambda: defaultdict(list)), manual_accel_lead_away=[],
             manual_brake_onsets=[], manual_coast_onsets=[], gas_overrides=[], brake_takeovers=[],
             personality_changes=[], mode_seconds=defaultdict(float), coasts=[], pcm_brake=defaultdict(lambda: [0, 0]),
             per_route={})
  for r in results:
    tot["routes"].append(r["route"])
    tot["seconds"] += r["seconds"]
    eng = r["mode_seconds"].get("engaged", 0) + r["mode_seconds"].get("gas_override", 0)
    tot["per_route"][r["route"]] = dict(engaged_s=round(eng), gas=sum(e["same_lead"] for e in r["gas_overrides"]),
                                        brake=sum(e["same_lead"] for e in r["brake_takeovers"]), coasts=len(r["coasts"]))
    for b, (on, off) in r["pcm_brake"].items():
      tot["pcm_brake"][b][0] += on
      tot["pcm_brake"][b][1] += off
    for md, bins in r["steady_gap"].items():
      for b, xs in bins.items():
        tot["steady_gap"][md][b] += xs
    for key in ("manual_accel_lead_away", "manual_brake_onsets", "manual_coast_onsets", "gas_overrides",
                "brake_takeovers", "personality_changes", "coasts"):
      tot[key] += [dict(x, route=r["route"]) if isinstance(x, dict) else x for x in r[key]]
    for md, s in r["mode_seconds"].items():
      tot["mode_seconds"][md] += s
  return tot


def pcm_brake_threshold(pcm_brake):
  """Highest request level (m/s^2) at which the PCM has its ACC brake lights on at least half the time, or None."""
  best = None
  for b, (on, off) in pcm_brake.items():
    if on + off >= 20 and on >= off:
      best = float(b) if best is None else max(best, float(b))
  return best


def summary(tot):
  L = [f"routes {len(tot['routes'])}, {tot['seconds'] / 60:.0f} min; time by mode (min): "
       + ", ".join(f"{m} {s / 60:.0f}" for m, s in sorted(tot["mode_seconds"].items()))]
  L.append("steady following gap as sunnypilot t_follow (s): median [p25-p75] (samples)")
  for md in ["manual", "manual_lat"] + sorted(m for m in tot["steady_gap"] if m.startswith("engaged")):
    bins = tot["steady_gap"].get(md, {})
    row = []
    for lo, hi in SPEED_BINS:
      b = f"{lo}-{hi if hi < 200 else '+'}"
      xs = bins.get(b, [])
      row.append(f"{b}mph {pct(xs, 50)} [{pct(xs, 25)}-{pct(xs, 75)}] ({len(xs)})" if len(xs) >= 10 else f"{b}mph -")
    L.append(f"  {md:19s} " + " | ".join(row))
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
  eng_h = (tot["mode_seconds"].get("engaged", 0) + tot["mode_seconds"].get("gas_override", 0)) / 3600
  if eng_h > 0:
    g = sum(e["same_lead"] for e in tot["gas_overrides"]); b = sum(e["same_lead"] for e in tot["brake_takeovers"])
    L.append(f"override rate (same-lead only): gas {g / eng_h:.1f}/h, brake {b / eng_h:.1f}/h over {eng_h * 60:.0f} engaged min")
  co = tot["coasts"]
  if co:
    ends = defaultdict(int)
    for c in co:
      ends[c["end"]] += 1
    L.append(f"coasts (no pedal behind a lead): {len(co)}, lift-off gap median {pct([c['gap_s'] for c in co], 50)} s, "
             f"lowest gap median {pct([c['min_gap_s'] for c in co], 50)} s, decel median {pct([c['decel'] for c in co], 50)} m/s^2, "
             f"ends: " + ", ".join(f"{k} {v}" for k, v in sorted(ends.items())))
    br = [c for c in co if c["end"] == "brake"]
    if br:
      L.append(f"  coasts ending in braking: {len(br)}, gap at brake median {pct([c['end_gap_s'] for c in br], 50)} s")
  pb = pcm_brake_threshold(tot["pcm_brake"])
  if pb is not None:
    L.append(f"PCM brakes (0x3B7) for requests at or below {pb:.1f} m/s^2 ({sum(sum(v) for v in tot['pcm_brake'].values())} samples)")
  elif tot["pcm_brake"]:
    L.append("PCM brake-light bit (0x3B7): never set on this car, so no PCM coast floor (probe_3b7.py 2026-10-24: no bit tracks braking)")
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
