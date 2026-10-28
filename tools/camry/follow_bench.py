# Closed-loop following test bench: runs sunnypilot's real LongitudinalPlanner (MPC) against simulated leads
# and compares follow-gap tunes for the Aggressive / Personalized slot, with and without glide and soft plan,
# plus glide on and off in the Standard and Relaxed modes (their stock gaps).
# Nothing is published or written: no msgq sockets are opened, real-time pacing is disabled, and the gap table
# and Personalized settings are swapped in-process only (the settings file on the device is never read here).
#
# Run with the car OFF (refuses if onroad), works on the stock branch (no install needed):
#   cd /data/openpilot && OPENPILOT_PREFIX=lbench PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 /tmp/follow_bench.py
# Options: --table FILE   also bench the Personalized settings file FILE (e.g. before tapping Apply)
#          --quick        only the glide-relevant scenarios
#          --gate CURRENT CANDIDATE [--modes all]   full bench of two settings files, PASS/FAIL for the candidate
#                                                   (used by suggest_long.py; --modes all adds Standard and Relaxed)
import os
import sys
from dataclasses import replace
os.environ.setdefault("OPENPILOT_PREFIX", "lbench")   # isolate params/msgq namespace from the real system

try:
  with open("/data/params/d/IsOnroad", "rb") as f:
    if f.read().strip() == b"1":
      sys.exit("car is onroad; run this with the car off")
except OSError:
  pass

import numpy as np
import cereal.messaging as messaging
from cereal import log
import openpilot.selfdrive.test.longitudinal_maneuvers.plant as plant_mod
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib import camry_personalized as cp

MPH = 0.44704
DT = 0.05
AGG = log.LongitudinalPersonality.aggressive

# candidate tunes for the Aggressive / Personalized slot: (speed breakpoints in mph, t_follow values)
TUNES = {
  "stock 1.25":     ([0], [1.25]),
  "PR7 1.0":        ([40, 60], [1.0, 1.45]),
  "0.8 to 35":      ([35, 40, 60], [0.8, 1.0, 1.45]),
  "flat 1.1":       ([35, 40, 60], [1.1, 1.1, 1.1]),   # the PR #9 default table (three points, as the learner keeps them)
}
# settings per tune (glide/soft plan); tunes not listed run with both OFF (the file's defaults have both on)
OFF = cp.PersonalizedSettings(glide=False, soft_plan=False, approach=False, overspeed_glide_mph=0.0)   # the plain planner
GLIDE = cp.PersonalizedSettings(personalized=True, glide=True, soft_plan=False)
SOFT = cp.PersonalizedSettings(personalized=True, glide=True, soft_plan=True)
DEFAULTS = cp.PersonalizedSettings()   # what the car runs after the update with no settings file
PR8 = replace(SOFT, overspeed_glide_mph=0.0, approach=False)   # glide + soft plan as PR #8 behaves (no overspeed glide, no approach taper)
SETTINGS = {
  "0.8 + glide":    (TUNES["0.8 to 35"], replace(GLIDE, overspeed_glide_mph=0.0, approach=False)),
  "0.8 + glide+soft": (TUNES["0.8 to 35"], PR8),
  "0.8 + all":      (TUNES["0.8 to 35"], DEFAULTS),   # PR #8 plus glide over set speed and the approach taper (the defaults)
  "1.1 + glide+soft": (TUNES["flat 1.1"], PR8),        # PR #9 alone: the 1.1 s table with PR #8 behavior
  "1.1 + all":      (TUNES["flat 1.1"], DEFAULTS),     # PR #9 + PR #10: what the car runs after both, with no settings file
  "standard + defaults": (TUNES["stock 1.25"], DEFAULTS),   # table unused: Standard and Relaxed keep their stock gaps
  "relaxed + defaults":  (TUNES["stock 1.25"], DEFAULTS),
}
# glide in the other distance modes: the Plant's personality per tune (default Aggressive)
PERSONALITIES = {"standard": log.LongitudinalPersonality.standard, "standard + defaults": log.LongitudinalPersonality.standard,
                 "relaxed": log.LongitudinalPersonality.relaxed, "relaxed + defaults": log.LongitudinalPersonality.relaxed}
TUNES["standard"] = TUNES["stock 1.25"]
TUNES["relaxed"] = TUNES["stock 1.25"]
for _name, (_tune, _st) in SETTINGS.items():
  TUNES[_name] = _tune
# GLIDE CHECK: each glide tune against the same gaps without glide
BASE_OF = {"0.8 + glide": "0.8 to 35", "0.8 + glide+soft": "0.8 to 35", "0.8 + all": "0.8 + glide+soft",
           "1.1 + glide+soft": "flat 1.1", "1.1 + all": "1.1 + glide+soft", "standard + defaults": "standard",
           "relaxed + defaults": "relaxed", "file": "0.8 to 35"}
CUR_SETTINGS = [OFF]   # what make_planner gives the planner (per worker process)
CUR_PERSONALITY = [AGG]
BRAKE_A = -0.4   # m/s^2, car decel counted as "braking" in the brake-time metric


def add_table_tune(path, name="file"):
  """--table FILE: the Personalized settings file as its own tune, with its glide/soft plan settings (also --gate)."""
  st = cp.read_settings(path)
  bp, vals = cp.follow_table_ms(cp.PersonalizedSettings(**{**st.__dict__, "personalized": True}))
  TUNES[name] = ([b / MPH for b in bp], list(vals))
  SETTINGS[name] = (TUNES[name], st)


class FakeRk:
  def __init__(self, *a, **k):
    self.frame = 0
  def monitor_time(self):
    self.frame += 1
    return False


class NoSock:
  def __init__(self, *a, **k):
    pass


plant_mod.Ratekeeper = FakeRk
plant_mod.Plant.messaging_initialized = True   # skip pub/sub socket creation
plant_mod.time.sleep = lambda s: None
messaging.SubMaster = NoSock


# The stock branch calls get_T_FOLLOW(personality) without speed, so record the MPC's current v_ego here
CUR_V = [0.0]
_mpc_update = long_mpc.LongitudinalMpc.update
def _update(self, *a, **k):
  CUR_V[0] = float(self.x0[1])
  return _mpc_update(self, *a, **k)
long_mpc.LongitudinalMpc.update = _update


def set_tune(bp_mph, vals, settings=None, personality=AGG):
  bp = [b * MPH for b in bp_mph]
  stock = long_mpc.get_T_FOLLOW
  def t_follow(personality=log.LongitudinalPersonality.standard, v_ego=None, personalized=None):
    if personality == AGG:
      return float(np.interp(CUR_V[0] if v_ego is None else v_ego, bp, vals))
    return stock(personality)
  long_mpc.get_T_FOLLOW = t_follow
  # the planner's own copy of the table (long_mpc.update reads self.personalized) and the glide settings
  st = settings if settings is not None else OFF
  CUR_SETTINGS[0] = cp.PersonalizedSettings(**{**st.__dict__, "follow_bp_mph": tuple(bp_mph), "follow_s": tuple(vals)})
  CUR_PERSONALITY[0] = personality
  return stock


def personalize(planner):
  st = CUR_SETTINGS[0]
  planner.glide = cp.SafeGlide(cp.GlideGovernor(dt=DT, live=False, settings=st))   # explicit settings, never the device's file
  planner.mpc.personalized = cp.follow_table_ms(st)   # None unless the tune's settings say Personalized
  return planner


CAR_MODE = os.environ.get("BENCH_CAR", "toyota")   # "honda" = the Plant's own CarParams, as in openpilot CI


CP_VARIANT = ""   # diag A/B: "honda_swap", or "+radar", "+delay", "+flags", "+all" on top of the Camry CP


FIELD_GROUPS = {
  "id": ["brand", "carFingerprint"],
  "geom": ["wheelbase", "steerRatio", "centerToFront", "mass", "rotationalInertia", "tireStiffnessFront", "tireStiffnessRear"],
  "rest": ["longitudinalTuning", "transmissionType", "maxLateralAccel", "lateralTuning", "lateralParams", "steerActuatorDelay",
           "steerLimitTimer", "steerControlType", "minSteerSpeed", "safetyConfigs", "tireStiffnessFactor", "wheelSpeedFactor"],
}


def mixed_cp(base, group):
  """base CP with the fields of one group copied from the other car ('allbut_id' = everything except identity)."""
  from opendbc.car.toyota.values import CAR as TC
  from opendbc.car.toyota.interface import CarInterface as TI
  from opendbc.car.honda.values import CAR as HC
  from opendbc.car.honda.interface import CarInterface as HI
  t, h = TI.get_non_essential_params(TC.TOYOTA_CAMRY), HI.get_non_essential_params(HC.HONDA_CIVIC)
  t.openpilotLongitudinalControl = True
  CP, other = (h, t) if base == "honda" else (t, h)
  if group == "allbut_id":
    fields = [f for f in other.to_dict() if f not in FIELD_GROUPS["id"]]
  else:
    fields = FIELD_GROUPS[group]
  od = other.to_dict()
  for f in fields:
    if f in od:
      try:
        setattr(CP, f, getattr(other, f))   # capnp builder: struct/list assignment copies
      except Exception as e:
        print(f"mixed_cp: could not copy {f}: {e}")
  SP = (HI.get_non_essential_params_sp(CP, HC.HONDA_CIVIC) if base == "honda" else TI.get_non_essential_params_sp(CP, TC.TOYOTA_CAMRY))
  CP.openpilotLongitudinalControl = True   # get_non_essential_params_sp resets it
  return CP, SP


def make_planner(v0):
  if CP_VARIANT.startswith("mix:"):   # e.g. mix:honda:id = Honda CP with the Camry's brand/fingerprint
    _, base, group = CP_VARIANT.split(":")
    CP, SP = mixed_cp(base, group)
    return personalize(LongitudinalPlanner(CP, SP, init_v=v0))
  if CP_VARIANT == "honda_swap":   # Honda CP, but built through the same swap path as the Camry
    from opendbc.car.honda.values import CAR
    from opendbc.car.honda.interface import CarInterface
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    return personalize(LongitudinalPlanner(CP, CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC), init_v=v0))
  from opendbc.car.toyota.values import CAR
  from opendbc.car.toyota.interface import CarInterface
  CP = CarInterface.get_non_essential_params(CAR.TOYOTA_CAMRY)
  # the Camry runs openpilot longitudinal (Alpha Long); without this the planner resets every step
  # because the bench never sets selfdriveState.enabled
  CP.openpilotLongitudinalControl = True
  if CP_VARIANT in ("+radar", "+all"):
    CP.radarUnavailable = False
  if CP_VARIANT in ("+delay", "+all"):
    CP.radarDelay = 0.1
  if CP_VARIANT in ("+flags", "+all"):
    CP.flags = 272
    CP.autoResumeSng = True
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.TOYOTA_CAMRY)
  CP.openpilotLongitudinalControl = True   # get_non_essential_params_sp resets this to False, so set it after
  return personalize(LongitudinalPlanner(CP, CP_SP, init_v=v0))


def use_camry_planner():
  # Plant builds its planner inside __init__; swap the class it uses so only ONE planner (one acados solver)
  # exists per run. A second solver instance in the same process corrupts the first ("solution_status 4" resets).
  plant_mod.LongitudinalPlanner = lambda CP, CP_SP, init_v=0.0: make_planner(init_v)


def make_plant(v0, d0, e2e):
  if CAR_MODE != "honda":
    use_camry_planner()
  return plant_mod.Plant(lead_relevancy=True, speed=v0, distance_lead=d0, e2e=e2e, personality=CUR_PERSONALITY[0])


def run(lead_profile, v0, d0, duration, e2e=False, cruise_mph=80):
  """lead_profile(t) -> lead speed. Returns metrics after a 30 s settle at constant lead speed."""
  if d0 is None:
    d0 = long_mpc.get_T_FOLLOW(CUR_PERSONALITY[0], v0) * v0 + long_mpc.STOP_DISTANCE
  p = make_plant(v0, d0, e2e)
  p.v_lead_prev = v0
  settle = 40.0
  rows, fcw, glide_s = [], 0, 0.0
  while p.current_time < settle + duration:
    t = p.current_time
    vl = float(lead_profile(t - settle)) if t >= settle else v0
    out = p.step(vl, 1.0, cruise_mph * MPH, 0.0, 1.0)
    if t >= settle:
      d = out["distance_lead"] - out["distance"]
      rows.append((d, out["speed"], vl, out["acceleration"]))
      fcw += bool(out["fcw"])
      glide_s += DT * bool(getattr(getattr(p.planner, "glide", None), "active", False))
  a = np.array(rows)
  d, v, vl, acc = a.T
  closing = vl - v
  ttc = np.where(closing < -0.3, d / np.maximum(-closing, 1e-3), np.inf)
  expect = long_mpc.get_T_FOLLOW(CUR_PERSONALITY[0], v[0]) * v[0] + long_mpc.STOP_DISTANCE
  return dict(gap_m=d[0], expect_m=expect, gap_s=d[0] / max(v[0], 0.1), min_gap_m=d.min(), min_ttc=ttc.min(),
              min_a=acc.min(), max_jerk=np.abs(np.diff(acc)).max() / DT, fcw=fcw, crash=bool(d.min() < 0.4),
              brake_s=float((acc < BRAKE_A).sum() * DT), glide_s=glide_s, end_gap_s=d[-1] / max(v[-1], 0.1),
              min_gap_s=d.min() / max(v[0], 0.1))


def scenarios():
  S = []
  for mph in (30, 45, 60, 70):
    v = mph * MPH
    S.append((f"steady {mph} mph", lambda t, v=v: v, v, None, 5.0))
  for mph in (30, 45, 60):
    v = mph * MPH
    for dec in (2.0, 3.5, 5.0):
      S.append((f"lead brakes {dec} m/s2 from {mph} mph to stop", lambda t, v=v, dec=dec: max(0.0, v - dec * t), v, None, 25.0))
  v60 = 60 * MPH
  S.append(("lead slows 60->40 mph at 1.5 m/s2", lambda t: max(40 * MPH, v60 - 1.5 * t), v60, None, 25.0))
  S += glide_scenarios()
  return S


def glide_scenarios():
  """Where glide should show: small speed differences and gentle lead changes behind the same lead."""
  S = []
  for mph in (30, 45, 65):
    v = mph * MPH
    for dv in (2, 5):   # lead settles dv mph slower: we close slowly, then should glide into the new gap
      S.append((f"lead drops {dv} mph from {mph} mph and holds", lambda t, v=v, dv=dv: v - min(dv * MPH, 0.7 * t), v, None, 30.0))
    # lead speed wobbles +-2 mph with a 20 s period: the gap hunting that makes stock brake all the time
    S.append((f"lead wobbles +-2 mph at {mph} mph", lambda t, v=v: v + 2 * MPH * np.sin(2 * np.pi * t / 20.0), v, None, 40.0))
    # lead brakes moderately for 3 s then holds: glide must hand over to real braking
    S.append((f"lead brakes 1.5 m/s2 for 3 s at {mph} mph", lambda t, v=v: v - 1.5 * min(t, 3.0), v, None, 25.0))
  return S


def cut_in(v_ego_mph, d_cut, dv):
  """Lead suddenly appears d_cut m ahead, dv m/s slower, then holds speed."""
  v = v_ego_mph * MPH
  p = make_plant(v, 200.0, False)
  p.lead_relevancy = False
  for _ in range(int(20 / DT)):
    p.step(v, 0.0, v, 0.0, 1.0)   # cruise = ego speed, so the ego holds speed until the cut-in
  v_cut = p.speed
  p.lead_relevancy = True
  p.distance_lead = p.distance + d_cut
  p.v_lead_prev = v_cut - dv
  rows, fcw = [], 0
  for _ in range(int(20 / DT)):
    out = p.step(v_cut - dv, 1.0, v, 0.0, 1.0)
    rows.append((out["distance_lead"] - out["distance"], out["speed"], out["acceleration"]))
    fcw += bool(out["fcw"])
  a = np.array(rows)
  return dict(min_gap_m=a[:, 0].min(), min_a=a[:, 2].min(), fcw=fcw, crash=bool(a[:, 0].min() < 0.4))


# ---- overspeed glide: ego over the set speed with nothing else limiting the plan ----
def run_overspeed(v0_mph, cruise_profile, duration, pitch=0.0, lead=None):
  """cruise_profile(t) -> set speed (m/s); lead = (distance m, speed m/s) of a slower lead far ahead, else none.
  Metrics: braking time, hardest decel, seconds over the set speed, when it got back to it, glide time; with a lead
  also the min gap and TTC as in run()."""
  v0 = v0_mph * MPH
  p = make_plant(v0, 200.0 if lead is None else lead[0], False)
  p.lead_relevancy = lead is not None
  vl = v0 if lead is None else lead[1]
  p.v_lead_prev = vl
  rows, fcw, glide_s = [], 0, 0.0
  while p.current_time < duration:
    vc = float(cruise_profile(p.current_time))
    out = p.step(vl, 1.0 if lead is not None else 0.0, vc, pitch, 1.0)
    rows.append((out["speed"], out["acceleration"], vc, out["distance_lead"] - out["distance"]))
    fcw += bool(out["fcw"])
    glide_s += DT * bool(getattr(getattr(p.planner, "glide", None), "active", False))
  v, acc, vc, d = np.array(rows).T
  over = v - vc
  back = np.flatnonzero(over <= 0.5 * MPH)
  res = dict(min_a=acc.min(), max_jerk=np.abs(np.diff(acc)).max() / DT, fcw=fcw, crash=bool(lead is not None and d.min() < 0.4),
             brake_s=float((acc < BRAKE_A).sum() * DT), glide_s=glide_s, over_s=float((over > 0.5 * MPH).sum() * DT),
             max_over_mph=float(over.max() / MPH), back_s=float(back[0] * DT) if len(back) else None, pitch=pitch)
  if lead is not None:
    closing = vl - v
    ttc = np.where(closing < -0.3, d / np.maximum(-closing, 1e-3), np.inf)
    res.update(min_gap_m=float(d.min()), min_gap_s=float(d.min() / max(v.max(), 0.1)), min_ttc=float(ttc.min()))
  return res


def overspeed_scenarios():
  """(title, v0 mph, cruise_profile, duration, pitch rad, lead). Titles start with 'over set speed' (flat/hill) or 'set speed'."""
  S = []
  for mph, over in ((30, 3), (45, 2), (45, 4), (60, 4), (55, 10)):
    S.append((f"over set speed by {over} mph at {mph} mph", mph + over, lambda t, v=mph * MPH: v, 30.0, 0.0, None))
  S.append(("set speed drops 55->35 mph after 5 s", 55, lambda t: (55.0 if t < 5.0 else 35.0) * MPH, 40.0, 0.0, None))
  S.append(("over set speed by 3 mph at 45 mph, 3 deg downhill", 48, lambda t: 45 * MPH, 30.0, -0.052, None))
  S.append(("over set speed by 3 mph at 45 mph, 3 deg uphill", 48, lambda t: 45 * MPH, 30.0, 0.052, None))
  S.append(("over set speed by 3 mph at 45 mph, slower lead 70 m ahead at 40 mph", 48, lambda t: 45 * MPH, 40.0, 0.0, (70.0, 40 * MPH)))
  S.append(("over set speed by 5 mph at 60 mph, slower lead 90 m ahead at 50 mph", 65, lambda t: 60 * MPH, 40.0, 0.0, (90.0, 50 * MPH)))
  return S


OVERSPEED = overspeed_scenarios()
OVERSPEED_TITLES = [t for t, *_ in OVERSPEED]


# ---- approach: ego well behind a slower lead and faster than it, set speed higher still (the run-up case) ----
def run_approach(v_ego_mph, v_lead_mph, d0, cruise_mph, duration=60.0):
  """Closes in from d0 m. Metrics: peak accel, peak jerk, braking time, hardest decel, how far under the target gap it
  got (overshoot), min gap and TTC, FCW, crash, when the gap settled within 15% of the target, glide time."""
  v0, vl = v_ego_mph * MPH, v_lead_mph * MPH
  p = make_plant(v0, d0, False)
  p.v_lead_prev = vl
  rows, fcw, glide_s = [], 0, 0.0
  while p.current_time < duration:
    out = p.step(vl, 1.0, cruise_mph * MPH, 0.0, 1.0)
    rows.append((out["distance_lead"] - out["distance"], out["speed"], out["acceleration"]))
    fcw += bool(out["fcw"])
    glide_s += DT * bool(getattr(getattr(p.planner, "glide", None), "active", False))
  d, v, acc = np.array(rows).T
  expect = long_mpc.get_T_FOLLOW(CUR_PERSONALITY[0], vl) * vl + long_mpc.STOP_DISTANCE   # target gap at the lead's speed
  closing = vl - v
  ttc = np.where(closing < -0.3, d / np.maximum(-closing, 1e-3), np.inf)
  outside = np.flatnonzero(np.abs(d - expect) > 0.15 * expect + 1)
  under = max(0.0, expect - d.min())
  return dict(min_gap_m=d.min(), min_gap_s=d.min() / max(v.max(), 0.1), expect_m=expect, under_m=under, under_s=under / max(vl, 0.1),
              peak_a=acc.max(), min_a=acc.min(), max_jerk=np.abs(np.diff(acc)).max() / DT, brake_s=float((acc < BRAKE_A).sum() * DT),
              fcw=fcw, crash=bool(d.min() < 0.4), min_ttc=ttc.min(), glide_s=glide_s,
              settle_s=float((outside[-1] + 1) * DT) if len(outside) else 0.0)


# (title, ego mph, lead mph, start gap m, set speed mph)
APPROACH = [
  ("approach: lead 30 mph, from 40 mph with 45 set, 120 m back", 40, 30, 120.0, 45),
  ("approach: lead 50 mph, from 60 mph with 70 set, 150 m back", 60, 50, 150.0, 70),
  ("approach: lead 45 mph, from 45 mph with 60 set, 100 m back", 45, 45, 100.0, 60),
  ("approach: lead 20 mph, from 30 mph with 35 set, 80 m back", 30, 20, 80.0, 35),
]
APPROACH_TITLES = [t for t, *_ in APPROACH]


def job(args):
  name, idx = args
  bp, vals = TUNES[name]
  # each worker process gets its own copy of the patched module
  set_tune(bp, vals, SETTINGS[name][1] if name in SETTINGS else None, PERSONALITIES.get(name, AGG))
  sc = scenarios()
  if idx < len(sc):
    title, prof, v0, d0, dur = sc[idx]
    return name, title, run(prof, v0, d0, dur)
  if idx < len(sc) + len(CUT_INS):
    mph, dcut, dv = CUT_INS[idx - len(sc)]
    return name, f"cut-in {dcut} m ahead, {dv} m/s slower at {mph} mph", cut_in(mph, dcut, dv)
  k = idx - len(sc) - len(CUT_INS)
  if k < len(OVERSPEED):
    title, v0, prof, dur, pitch, ld = OVERSPEED[k]
    return name, title, run_overspeed(v0, prof, dur, pitch, ld)
  title, ve, vl, d0, vc = APPROACH[k - len(OVERSPEED)]
  return name, title, run_approach(ve, vl, d0, vc)


CUT_INS = ((40, 15, 3), (60, 25, 4), (45, 28, 1), (45, 20, 2))   # the last two: ~1.1 s and ~0.7 s gaps at 45 mph
GLIDE_ONLY = [t for t, *_ in glide_scenarios()]


def job_list(names, quick):
  """(tune, scenario index) pairs; quick = steady, glide and cut-in scenarios only."""
  sc = scenarios()
  n_jobs = len(sc) + len(CUT_INS) + len(OVERSPEED) + len(APPROACH)
  return [(name, i) for name in names for i in range(n_jobs)
          if not quick or i >= len(sc) or sc[i][0] in GLIDE_ONLY or sc[i][0].startswith("steady")]


def run_jobs(jobs, names):
  from multiprocessing import Pool
  res = {name: {} for name in names}
  order = [t for t, *_ in scenarios()] + [f"cut-in {d} m ahead, {dv} m/s slower at {m} mph" for m, d, dv in CUT_INS] + OVERSPEED_TITLES + APPROACH_TITLES
  print(f"{len(jobs)} runs on {os.cpu_count()} cores", flush=True)
  with Pool(os.cpu_count(), maxtasksperchild=1) as pool:   # fresh process per job: no patches leak between jobs
    for k, (name, title, m) in enumerate(pool.imap_unordered(job, jobs), 1):
      res[name][title] = m
      if k % 10 == 0:
        print(f"  {k}/{len(jobs)}", flush=True)
  return {n: {t: r[t] for t in order if t in r} for n, r in res.items()}


GATE_MODES = {"personalized": AGG, "standard": log.LongitudinalPersonality.standard, "relaxed": log.LongitudinalPersonality.relaxed}


def gate(cur_path, cand_path, modes=("personalized",)):
  """--gate CURRENT CANDIDATE [--modes all] (two settings files): the full scenario set for both, in the Personalized
  slot and, with --modes all, in Standard and Relaxed too (glide and soft plan act there as well). PASS unless in some
  scenario of some mode the candidate has more FCW frames than the current (strict: the bench is deterministic, so even
  one more frame is an effect of the change, e.g. a closer gap, not noise), crashes where the current doesn't, brakes
  harder than -2.5 where the current didn't, gets under HARD_MIN_GAP_S where the current didn't, or misses its steady
  gap. suggest_long.py then benches the glide/soft plan changes and the gap table change separately on a fail. Prints
  a verdict line and a GATE_JSON line (read by suggest_long.py before a suggestion is offered or applied)."""
  import json
  names = []
  for mode in modes:
    for which, path in (("current", cur_path), ("candidate", cand_path)):
      name = f"{which}/{mode}"
      add_table_tune(path, name)
      PERSONALITIES[name] = GATE_MODES[mode]
      names.append(name)
  res = run_jobs(job_list(names, False), names)
  worse, tot = [], {}
  for mode in modes:
    cur, cand = res[f"current/{mode}"], res[f"candidate/{mode}"]
    for k, m in cand.items():
      b = cur.get(k)
      if b is None:
        continue
      if m["fcw"] > b["fcw"]:
        worse.append(f"{mode}, {k}: FCW {m['fcw']} > {b['fcw']}")
      if m["crash"] and not b["crash"]:
        worse.append(f"{mode}, {k}: crash")
      if m["min_a"] < -2.5 <= b["min_a"]:
        worse.append(f"{mode}, {k}: decel {m['min_a']:.2f}")
      if "min_gap_s" in m and m["min_gap_s"] < cp.HARD_MIN_GAP_S <= b.get("min_gap_s", 9.0):
        worse.append(f"{mode}, {k}: min gap {m['min_gap_s']:.2f} s")
      if k.startswith("steady") and abs(m["gap_m"] - m["expect_m"]) > 0.15 * m["expect_m"] + 1:
        worse.append(f"{mode}, {k}: steady gap {m['gap_m']:.0f} m, table {m['expect_m']:.0f} m")
      if k in OVERSPEED_TITLES and m.get("pitch", 0.0) == 0.0 and m.get("back_s") is None and b.get("back_s") is not None:
        worse.append(f"{mode}, {k}: never got back to the set speed")
    for which, r in (("current", cur), ("candidate", cand)):
      tot[f"{which}/{mode}"] = dict(fcw=sum(m["fcw"] for m in r.values()), crashes=sum(m["crash"] for m in r.values()),
                                    brake_s=round(sum(m.get("brake_s", 0.0) for m in r.values()), 1),
                                    min_gap_s=round(min(m["min_gap_s"] for m in r.values() if "min_gap_s" in m), 2))
  verdict = "pass" if not worse else "fail"
  print(f"GATE {verdict.upper()}: " + " | ".join(f"{n}: FCW {t['fcw']}, crashes {t['crashes']}, min gap {t['min_gap_s']} s, braking {t['brake_s']} s"
                                                 for n, t in tot.items()) + (f"  {worse}" if worse else ""))
  print("GATE_JSON " + json.dumps(dict(verdict=verdict, worse=worse, modes=list(modes), totals=tot)))


def main():
  if "--table" in sys.argv:
    add_table_tune(sys.argv[sys.argv.index("--table") + 1])
  names = list(TUNES)
  res = run_jobs(job_list(names, "--quick" in sys.argv), names)
  print("\nsteady gap at speed (m / s):")
  for mph in (30, 45, 60, 70):
    k = f"steady {mph} mph"
    print(f"  {mph} mph: " + " | ".join(f"{n} {res[n][k]['gap_m']:.0f} m (table {res[n][k]['expect_m']:.0f}) {res[n][k]['gap_s']:.2f} s" for n in names))
  bad = [(n, k) for n in names for k, m in res[n].items() if k.startswith("steady") and abs(m["gap_m"] - m["expect_m"]) > 0.15 * m["expect_m"] + 1]
  sane = not res[names[0]]["lead brakes 2.0 m/s2 from 30 mph to stop"]["crash"] and not bad
  print(f"FIDELITY CHECK: {'OK' if sane else 'FAILED (do not trust results)'}" + (f" steady-gap mismatches: {bad}" if bad else ""))
  print("\nevents (min gap m, min TTC s, hardest decel m/s2, FCW frames, crash):")
  for k in res[names[0]]:
    if k.startswith("steady") or k in OVERSPEED_TITLES or k in APPROACH_TITLES:
      continue
    print(f"  {k}")
    for n in names:
      m = res[n][k]
      ttc = f"{m['min_ttc']:.1f}" if "min_ttc" in m and np.isfinite(m["min_ttc"]) else "-"
      print(f"    {n:14s} gap {m['min_gap_m']:5.1f}  ttc {ttc:>5s}  decel {m['min_a']:5.2f}  fcw {m['fcw']:3d}  crash {m['crash']}")
  worst = {n: dict(min_a=min(m["min_a"] for m in res[n].values()), fcw=sum(m["fcw"] for m in res[n].values()),
                   crashes=sum(m["crash"] for m in res[n].values())) for n in names}
  print("\nsummary: " + " | ".join(f"{n}: hardest {w['min_a']:.2f}, FCW frames {w['fcw']}, crashes {w['crashes']}" for n, w in worst.items()))

  # glide: braking time and gap behind the same lead, glide profiles vs their table without glide
  print(f"\nglide scenarios (seconds braking harder than {BRAKE_A:.1f} m/s2 / seconds gliding / gap at end s / min gap s):")
  for k in GLIDE_ONLY:
    if k not in res[names[0]]:
      continue
    print(f"  {k}")
    for n in names:
      m = res[n].get(k)
      if m:
        print(f"    {n:16s} brake {m['brake_s']:5.1f} s  glide {m['glide_s']:5.1f} s  end gap {m['end_gap_s']:.2f}  min gap {m['min_gap_s']:.2f}")
  print(f"\noverspeed scenarios (braking harder than {BRAKE_A:.1f} s / hardest decel / over the set speed s / back at it after s / glide s):")
  for k in OVERSPEED_TITLES:
    if k not in res[names[0]]:
      continue
    print(f"  {k}")
    for n in names:
      m = res[n].get(k)
      if m:
        back = f"{m['back_s']:.1f}" if m["back_s"] is not None else "never"
        extra = f"  min gap {m['min_gap_s']:.2f} s  fcw {m['fcw']}" if "min_gap_s" in m else ""
        print(f"    {n:16s} brake {m['brake_s']:5.1f} s  decel {m['min_a']:5.2f}  over {m['over_s']:5.1f} s"
              + f"  back {back:>5s}  glide {m['glide_s']:5.1f} s{extra}")
  print("\napproach scenarios (peak accel / peak jerk / braking s / hardest decel / under the target gap by s / min gap s / settled after s / fcw):")
  for k in APPROACH_TITLES:
    if k not in res[names[0]]:
      continue
    print(f"  {k}")
    for n in names:
      m = res[n].get(k)
      if m:
        print(f"    {n:16s} accel {m['peak_a']:4.2f}  jerk {m['max_jerk']:4.1f}  brake {m['brake_s']:5.1f} s  decel {m['min_a']:5.2f}"
              + f"  under {m['under_s']:4.2f} s  min gap {m['min_gap_s']:.2f} s  settled {m['settle_s']:5.1f} s  fcw {m['fcw']}")
  print("\nGLIDE CHECK, each glide tune vs the same gaps without glide (no new FCW frames, no new crashes, "
        + "braking no harder than -2.5 where the base stayed above it):")
  for n in SETTINGS:
    base = BASE_OF.get(n, names[0])
    if n not in res or base not in res:
      continue
    worse = []
    for k, m in res[n].items():
      b = res[base].get(k)
      if b is None:
        continue
      if m["fcw"] > b["fcw"]:
        worse.append(f"{k}: FCW {m['fcw']} > {b['fcw']}")
      if m["crash"] and not b["crash"]:
        worse.append(f"{k}: crash")
      if m["min_a"] < -2.5 <= b["min_a"]:
        worse.append(f"{k}: decel {m['min_a']:.2f}")
    brake_base = sum(res[base][k]["brake_s"] for k in GLIDE_ONLY if k in res[base])
    brake_n = sum(res[n][k]["brake_s"] for k in GLIDE_ONLY if k in res[n])
    print(f"  {n:16s} {'PASS' if not worse else 'FAIL'}  vs '{base}': braking in glide scenarios {brake_n:.0f} s vs {brake_base:.0f} s"
          + (f"  {worse}" if worse else ""))


def diag_job(args):
  mode, idx = args
  global CAR_MODE, CP_VARIANT
  if mode == "honda":
    CAR_MODE, CP_VARIANT = "honda", ""
  elif mode == "honda_swap":
    CAR_MODE, CP_VARIANT = "toyota", "honda_swap"   # Honda CP through the Camry swap path
  elif mode.startswith("mix:"):
    CAR_MODE, CP_VARIANT = "toyota", mode
  else:
    CAR_MODE, CP_VARIANT = "toyota", mode[len("toyota"):]
  if idx >= 0:
    from openpilot.selfdrive.test.longitudinal_maneuvers.test_longitudinal import create_maneuvers
    man = create_maneuvers({"e2e": False, "force_decel": False})[idx]
    if CAR_MODE == "toyota":   # same maneuver, Camry CarParams (or a diag variant)
      use_camry_planner()
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
      valid, out = man.evaluate()
    return mode, man.title, dict(valid=valid, min_gap=float(out[:, 6].min()))
  set_tune(*TUNES["stock 1.25"])
  v = 45 * MPH
  return mode, "lead brakes 2.0 from 45 mph (stock)", run(lambda t: max(0.0, v - 2.0 * t), v, None, 25.0)


def diag():
  """Checks the bench against openpilot's own CI maneuvers, with the CI Honda CarParams and with the Camry's."""
  from multiprocessing import Pool
  from openpilot.selfdrive.test.longitudinal_maneuvers.test_longitudinal import create_maneuvers
  n = len(create_maneuvers({"e2e": False, "force_decel": False}))
  modes = ("honda", "toyota")
  if "--ab" in sys.argv:   # A/B: which Camry CarParams field makes it differ from Honda
    modes = ("honda", "toyota", "mix:honda:id", "mix:honda:geom", "mix:honda:rest", "mix:toyota:allbut_id", "mix:toyota:id")
    keep = [i for i, m in enumerate(create_maneuvers({"e2e": False, "force_decel": False}))
            if "decel" in m.title or m.title.startswith("approach stopped car at 20m/s, initial distance 90m")]
  else:
    keep = list(range(n))
  jobs = [(m, i) for m in modes for i in keep + [-1]]
  with Pool(os.cpu_count(), maxtasksperchild=1) as pool:   # fresh process per job: no patches leak between jobs
    for mode, title, r in sorted(pool.imap_unordered(diag_job, jobs), key=lambda x: (x[0], x[1])):
      if "valid" in r:
        print(f"  {mode:20s} {'PASS' if r['valid'] else 'FAIL'}  min gap {r['min_gap']:6.1f} m  {title}")
      else:
        print(f"  {mode:20s} {'CRASH' if r['crash'] else 'ok   '}  min gap {r['min_gap_m']:6.1f} m  decel {r['min_a']:.2f}  {title}")


if __name__ == "__main__":
  if "--gate" in sys.argv:
    i = sys.argv.index("--gate")
    all_modes = "--modes" in sys.argv and sys.argv[sys.argv.index("--modes") + 1] == "all"
    gate(sys.argv[i + 1], sys.argv[i + 2], tuple(GATE_MODES) if all_modes else ("personalized",))
  elif "--diag" in sys.argv:
    diag()
  else:
    main()
