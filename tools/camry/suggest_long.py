# Turns a driver_profile.py result into suggested Personalized settings, with the evidence for each change.
# Writes the suggestion file (and a small per-route history); nothing changes on the car until the driver taps
# Apply on the Personalized page, which limits every step (camry_personalized.SUGGESTABLE). With the page's
# "Apply Automatically" on, the same limited step is applied here, and the previous settings are kept for Undo.
# Every suggestion is benched first (follow_bench.py --gate, strict: no extra FCW frame, crash, hard braking or close
# gap anywhere). If the whole set fails, the glide/soft plan changes and the gap table change are benched separately
# and the group that passes is offered; the rest is written under "dropped" with the bench's reason.
#
#   PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 tools/camry/suggest_long.py /tmp/driver_profile.json
# Options: --settings FILE, --out FILE (suggestion), --history FILE (per-route stats), --print-only
import argparse, json, os, subprocess, sys, tempfile, time
from collections import defaultdict

import numpy as np

from openpilot.selfdrive.controls.lib import camry_personalized as cp

MPH = 2.237
MIN_EVENTS = 3           # matching events before a change is suggested
MIN_EVENTS_SAFETY = 2    # brake takeovers at/under the target gap: suggest sooner, since they mean "too close"
TTC_LATE_S = 15.0        # brake takeovers with a longer time to collision than this aren't about closing speed
MIN_STEADY = 30          # steady-gap samples per speed bin before the manual gap counts
STEP_S = 0.05            # s, one step on the follow table (Apply allows up to 0.1)
# driver_profile speed bins -> the follow table point they inform (mph)
BIN_TO_BP = {"20-35": 35, "35-50": 40, "50-65": 60, "65-+": 60}
# soft plan strength (soft_distance_factor): firmer after brake takeovers inside the glide band, softer after a long
# stretch engaged without one; the learner never goes below SOFT_FLOOR
STOCK_T_FOLLOW = {1: 1.45, 2: 1.75}   # personality raw -> stock gap (standard, relaxed); 0 = the Personalized table
SOFT_STEP = 0.1
SOFT_FLOOR = 0.2
SOFT_RELAX_ENGAGED_S = 3600
GATE_TIMEOUT_S = 1200    # the full bench of current vs candidate settings takes 1-4 min on the comma's 8 cores


def pct(xs, p):
  if not xs:
    return None
  xs = sorted(xs)
  return xs[min(len(xs) - 1, int(p / 100 * len(xs)))]


def bin_of(mph):
  for lo, hi in ((0, 20), (20, 35), (35, 50), (50, 65), (65, 999)):
    if lo <= mph < hi:
      return f"{lo}-{hi if hi < 999 else '+'}"
  return None


def follow_table(settings):
  bp, vals = cp.follow_table_ms(cp.PersonalizedSettings(**{**settings.__dict__, "personalized": True}))
  return [round(b * MPH) for b in bp], list(vals)


def suggest_follow(tot, settings):
  """One change for the follow table, from gas overrides (closer), brake takeovers (farther) and manual steady gaps."""
  bp_mph, vals = follow_table(settings)
  new = list(vals)
  why = []
  gas = defaultdict(list)
  brake = defaultdict(list)
  for e in tot.get("gas_overrides", []):
    if e.get("same_lead") and e.get("gap_s") is not None and e.get("vrel", 0) is not None and e["vrel"] > -1.0:
      gas[bin_of(e["mph"])].append(e["gap_s"])
  for e in tot.get("brake_takeovers", []):
    # only takeovers at or under the table's gap count as "too close"; farther ones are late braking (see min_ttc_s)
    if e.get("same_lead") and e.get("gap_s") is not None and e["gap_s"] <= float(np.interp(e["mph"], bp_mph, vals)) + 0.1:
      brake[bin_of(e["mph"])].append(e["gap_s"])
  manual = tot.get("steady_gap", {}).get("manual", {})
  for b, bp in BIN_TO_BP.items():
    if bp not in bp_mph:
      continue
    i = bp_mph.index(bp)
    g, br, man = gas.get(b, []), brake.get(b, []), manual.get(b, [])
    if len(br) >= MIN_EVENTS_SAFETY:
      new[i] = vals[i] + (2 * STEP_S if len(br) >= 2 * MIN_EVENTS_SAFETY else STEP_S)
      why.append(f"{len(br)} brake takeover(s) at {b} mph, gap {pct(br, 50):.2f} s: farther")
    elif len(g) >= MIN_EVENTS:
      new[i] = vals[i] - (2 * STEP_S if len(g) >= 2 * MIN_EVENTS else STEP_S)
      why.append(f"{len(g)} gas press(es) at {b} mph, gap {pct(g, 50):.2f} s: closer")
    elif len(man) >= MIN_STEADY:
      med = pct(man, 50)
      if abs(med - vals[i]) >= STEP_S:
        new[i] = vals[i] + (STEP_S if med > vals[i] else -STEP_S)
        why.append(f"you follow at {med:.2f} s manually at {b} mph ({len(man)} samples)")
  new = [round(min(max(v, cp.MIN_FOLLOW_S), cp.MAX_FOLLOW_S), 2) for v in new]
  if new == [round(v, 2) for v in vals]:
    return None
  return dict(field="follow_s", bp_mph=bp_mph, to=new, why="; ".join(why), **{"from": [round(v, 2) for v in vals]})


def suggest_glide(tot, settings):
  """Glide band and coast decel from the driver's own coasts and the PCM brake-light check."""
  out = []
  coasts = [c for c in tot.get("coasts", []) if c["mph"] >= 20]
  if len(coasts) >= MIN_EVENTS and settings.band == cp.BAND_RELATIVE:
    bp_mph, vals = follow_table(settings)
    # how far under the profile's target gap the driver lifts off, and where he finally brakes
    under = []
    for c in coasts:
      target = float(np.interp(c["mph"], bp_mph, vals))
      under.append(target - c["gap_s"])
    margin = round(min(max(pct(under, 50), 0.0), 0.5), 2)
    if abs(margin - settings.coast_margin_s) >= 0.05:
      out.append(dict(field="coast_margin_s", to=margin, why=f"{len(coasts)} coasts, lift-off {pct(under, 50):.2f} s under target",
                      **{"from": settings.coast_margin_s}))
    brakes = [c for c in coasts if c["end"] == "brake"]
    if len(brakes) >= MIN_EVENTS:
      width = round(min(max(pct([c["gap_s"] - c["end_gap_s"] for c in brakes], 50), 0.1), 0.5), 2)
      if abs(width - settings.light_width_s) >= 0.05:
        out.append(dict(field="light_width_s", to=width, why=f"{len(brakes)} coasts ended in braking after {width:.2f} s of gap",
                        **{"from": settings.light_width_s}))
    dec = pct([c["decel"] for c in coasts], 50)
    if dec is not None:
      coast_a = round(min(max(dec, -0.6), 0.0), 1)
      if abs(coast_a - settings.coast_accel) >= 0.1:
        out.append(dict(field="coast_accel", to=coast_a, why=f"your coasts slow at {dec:.2f} m/s^2", **{"from": settings.coast_accel}))
  thr = None
  pb = tot.get("pcm_brake", {})
  if pb:
    from openpilot.tools.camry.driver_profile import pcm_brake_threshold
    thr = pcm_brake_threshold(pb)
  if thr is not None and settings.coast_accel <= thr:
    # coasting must stay above the level where the PCM starts braking
    coast_a = round(min(thr + 0.1, 0.0), 1)
    out = [o for o in out if o["field"] != "coast_accel"]
    out.append(dict(field="coast_accel", to=coast_a, why=f"the car brakes for requests at or below {thr:.1f} m/s^2 (0x3B7)",
                    **{"from": settings.coast_accel}))
  # takeovers while closing on the lead: glide should stay out until a longer time to collision
  tk = [e["ttc"] for e in tot.get("brake_takeovers", []) if e.get("same_lead") and e.get("ttc") and e["ttc"] <= TTC_LATE_S]
  if len(tk) >= MIN_EVENTS:
    ttc = float(round(min(max(pct(tk, 50) + 1.0, settings.min_ttc_s), 10.0)))
    if ttc > settings.min_ttc_s:
      out.append(dict(field="min_ttc_s", to=ttc, why=f"{len(tk)} brake takeover(s) while closing, TTC median {pct(tk, 50):.1f} s",
                      **{"from": settings.min_ttc_s}))
  return out


def suggest_soft(tot, settings):
  """Soft plan strength: one step firmer after brake takeovers inside the glide band, one step softer after an hour
  or more engaged (in the drives being learned from) without one."""
  if not settings.soft_plan or settings.band != cp.BAND_RELATIVE:
    return []
  bp_mph, vals = follow_table(settings)
  in_band = []
  for e in tot.get("brake_takeovers", []):
    if not (e.get("same_lead") and e.get("gap_s") is not None):
      continue
    tf = STOCK_T_FOLLOW.get(e.get("personality")) or float(np.interp(e["mph"], bp_mph, vals))
    coast_to = tf - settings.coast_margin_s
    if coast_to - settings.light_width_s - 0.05 <= e["gap_s"] <= coast_to + 0.05:
      in_band.append(e["gap_s"])
  cur = settings.soft_distance_factor
  engaged = sum(r.get("engaged_s", 0) for r in tot.get("per_route", {}).values())
  if len(in_band) >= MIN_EVENTS_SAFETY:
    new = round(min(cur + SOFT_STEP, 1.0), 2)
    why = f"{len(in_band)} brake takeover(s) inside the glide band, gap {pct(in_band, 50):.2f} s: firmer plan"
  elif not in_band and engaged >= SOFT_RELAX_ENGAGED_S:
    new = round(max(cur - SOFT_STEP, SOFT_FLOOR), 2)
    why = f"{engaged / 60:.0f} min engaged with no brake takeover inside the glide band: softer plan"
  else:
    return []
  if abs(new - cur) < 0.05:
    return []
  return [dict(field="soft_distance_factor", to=new, why=why, **{"from": cur})]


def bench_gate(settings, candidate, all_modes):
  """follow_bench.py --gate of the candidate settings against the current ones (full scenario set; all three distance
  modes when all_modes), as a child of this process so it ends with the learning job. Returns the GATE_JSON dict, or
  {"verdict": "skipped", "why": ...}."""
  bench = os.path.join(os.path.dirname(os.path.abspath(__file__)), "follow_bench.py")
  root = os.path.dirname(os.path.dirname(os.path.dirname(bench)))
  try:
    with tempfile.TemporaryDirectory() as d:
      cur, cand = os.path.join(d, "current.json"), os.path.join(d, "candidate.json")
      cp.write_settings(settings, cur)
      cp.write_settings(candidate, cand)
      cmd = [sys.executable, bench, "--gate", cur, cand] + (["--modes", "all"] if all_modes else [])
      r = subprocess.run(cmd, capture_output=True, text=True, timeout=GATE_TIMEOUT_S, env={**os.environ, "OPENPILOT_PREFIX": "lbench"}, cwd=root)
    for line in r.stdout.splitlines():
      if line.startswith("GATE_JSON "):
        return json.loads(line[len("GATE_JSON "):])
    return dict(verdict="skipped", why=f"bench gave no verdict (exit {r.returncode}): {(r.stderr or r.stdout)[-300:].strip()}")
  except Exception as e:
    return dict(verdict="skipped", why=f"bench did not run: {e}")


def _why(g):
  return "; ".join(g["worse"][:2]) if g.get("worse") else str(g.get("why", "no verdict"))


def gate_in_groups(settings, changes, full_gate, gate_fn=None):
  """The whole candidate failed the bench. Bench two groups on their own: the glide/soft plan changes without the gap
  table change, then the gap table change alone. Offer the first group that passes, never both (together is what
  failed), and list the rest as dropped with the bench's reason. Returns (offered, dropped, gate of the offered group
  or the full gate, group gates). At most two more bench runs."""
  gate_fn = gate_fn or bench_gate
  groups = [("glide and soft plan", [c for c in changes if c["field"] != "follow_s"]),
            ("gap table", [c for c in changes if c["field"] == "follow_s"])]
  full_why = "with all changes together the bench showed: " + _why(full_gate)
  if any(not grp for _, grp in groups):   # one kind of change only: the group is the whole set, which failed
    return [], [dict(c, dropped=full_why) for c in changes], full_gate, {}
  offered, offered_gate, dropped, gates = [], None, [], {}
  for name, grp in groups:
    g = gate_fn(settings, cp.apply_suggested({"changes": grp}, settings), any(c["field"] != "follow_s" for c in grp))
    gates[name] = g
    if g.get("verdict") == "pass" and not offered:
      offered, offered_gate = grp, g
    elif g.get("verdict") == "pass":
      dropped += [dict(c, dropped="passes the bench alone but not with the change(s) offered above; suggested again once those are applied") for c in grp]
    else:
      dropped += [dict(c, dropped="the bench showed: " + _why(g)) for c in grp]
  return offered, dropped, offered_gate or full_gate, gates


def _totals(g):
  return "; ".join(f"{n}: crashes {t['crashes']}, FCW {t['fcw']}, min gap {t['min_gap_s']} s, braking {t['brake_s']} s"
                   for n, t in g.get("totals", {}).items())


def update_history(tot, path):
  hist = cp.read_json(path) or {}
  routes = hist.get("routes", {})
  for r, st in tot.get("per_route", {}).items():
    routes[r] = dict(st, t=time.time())
  hist["routes"] = routes
  tmp = f"{path}.tmp"
  with open(tmp, "w") as f:
    json.dump(hist, f, indent=1)
  os.replace(tmp, path)
  return routes


def rates(routes):
  """Interventions (same-lead gas + brake) per engaged hour: newest route, and the newest 5 together."""
  order = sorted(routes)   # route names are date-time strings, so this is chronological
  def rate(names):
    eng = sum(routes[n]["engaged_s"] for n in names)
    return None if eng < 120 else 3600.0 * sum(routes[n]["gas"] + routes[n]["brake"] for n in names) / eng
  return dict(per_hour_last=rate(order[-1:]) if order else None, per_hour_5=rate(order[-5:]) if order else None,
              routes_seen=len(order))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("profile", help="driver_profile.py JSON output")
  ap.add_argument("--settings", default=cp.settings_path())
  ap.add_argument("--out", default=cp.suggested_path())
  ap.add_argument("--history", default=os.path.join(os.path.dirname(cp.suggested_path()), "camry_long_learned.json"))
  ap.add_argument("--print-only", action="store_true")
  ap.add_argument("--no-gate", action="store_true", help="skip the quick bench of the candidate settings")
  a = ap.parse_args()
  with open(a.profile) as f:
    tot = json.load(f)
  settings = cp.read_settings(a.settings)
  changes = []
  fo = suggest_follow(tot, settings)
  if fo:
    changes.append(fo)
  changes += suggest_glide(tot, settings)
  changes += suggest_soft(tot, settings)
  changes = list({c["field"]: c for c in changes}.values())   # one change per field
  routes = update_history(tot, a.history) if not a.print_only else {**(cp.read_json(a.history) or {}).get("routes", {}), **tot.get("per_route", {})}
  sug = dict(made=time.time(), routes=tot.get("routes", []), changes=changes, stats=rates(routes))
  print(f"{len(tot.get('routes', []))} route(s); interventions/h: last {sug['stats']['per_hour_last']}, last 5 {sug['stats']['per_hour_5']}")
  for c in changes:
    print(f"  {c['field']}: {c['from']} -> {c['to']}  ({c['why']})")
  if not changes:
    print("  no change suggested")
  if changes and not a.no_gate:
    # the candidate (what Apply would produce) must bench no worse than the current settings before it is offered;
    # glide and soft plan settings act in every distance mode, so those changes are benched in all three
    all_modes = any(c["field"] != "follow_s" for c in changes)
    gate = bench_gate(settings, cp.apply_suggested(sug, settings), all_modes)
    sug["gate"] = gate
    if gate["verdict"] == "fail":
      print(f"bench gate: FAIL with all changes together: {gate.get('worse')}")
      changes, sug["dropped"], sug["gate"], sug["gates"] = gate_in_groups(settings, changes, gate)
      sug["changes"], sug["gate_full"] = changes, gate
      for c in sug["dropped"]:
        print(f"  dropped {c['field']}: {c['dropped']}")
      if changes:
        print("bench gate: PASS for " + ", ".join(c["field"] for c in changes) + " on their own (" + _totals(sug["gate"]) + ")")
      else:
        print("bench gate: every suggestion dropped")
    elif gate["verdict"] == "pass":
      print("bench gate: PASS (" + _totals(gate) + ")")
    else:
      print(f"bench gate: not run ({gate.get('why')}); suggestions kept but not applied automatically")
  if not a.print_only:
    if settings.auto_apply and changes and sug.get("gate", {}).get("verdict") == "pass":
      # "Apply Automatically" is on: same limited step as the page's Apply button, previous file kept for Undo
      new = cp.apply_suggested(sug, settings)
      if new != settings:
        cp.save_with_history(new, a.settings)
        sug["applied"] = time.time()
        print("applied automatically (Apply Automatically is on); the previous settings are kept for Undo")
    tmp = f"{a.out}.tmp"
    with open(tmp, "w") as f:
      json.dump(sug, f, indent=1)
    os.replace(tmp, a.out)
    print(f"written: {a.out}")


if __name__ == "__main__":
  sys.exit(main())
