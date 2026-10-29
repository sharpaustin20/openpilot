"""
Camry following: the Personalized profile in the Aggressive slot of the distance button, and glide in
every distance mode.

Everything is set in /data/camry_long.json (the Personalized page) and tuned over time from the
learning job's suggestions:
- Personalized (on by default): a speed-based table of REAL gaps (DEFAULT_FOLLOW_* below) for the
  Aggressive slot: bumper-to-bumper time to the lead, distance / speed, with the MPC's 6 m stop distance
  included (t_follow_for_gap converts it); off = stock Aggressive (flat 1.25 s t_follow). Standard and
  Relaxed keep their stock gaps.
- glide (option A, on by default, every mode): the MPC penalizes being too close as much as being too
  far, so it brakes to win back every bit of gap. Behind a lead in the same lane, glide raises small
  braking requests to the car's coast decel while the gap is in a coast zone under the current mode's
  target, caps braking in a light zone below it, and passes the MPC output through unchanged below the
  band, when closing fast, when the lead brakes, right after a cut-in, or for anything that isn't the
  lead (curves, stops, e2e). It only ever makes a braking request less firm.
- soft plan (option C1, on by default): while glide may act, the MPC's weight on holding the exact
  distance is lowered, so the plan itself glides. Its one-sided danger-zone penalty is unchanged. The
  learning job tunes the strength (firmer after brake takeovers inside the band, softer without them).
- glide over set speed (overspeed_glide_mph, default 3, 0 = off): with nothing else limiting the plan (no lead,
  curve, stop or e2e request), braking only to get back down to the set speed or the speed limit target is lifted
  to the car's own coast on that grade while within the allowance, then ramps in linearly over OVERSPEED_RAMP_MPH
  and is capped at OVERSPEED_BRAKE: speed alone never brings hard braking. Never a throttle request.
- approach (on by default): closing on a slower lead, the acceleration request is tapered off once coasting plus light
  braking at approach_decel would be needed to arrive at the lead's speed right at the target gap (closing^2 / 2 x the
  distance left), down to no acceleration at all, so the car stops gaining speed early and eases into the gap instead
  of running up and braking. It only ever lowers a positive acceleration request, never below zero; braking requests
  are untouched. The closing speed feeding it is filtered (1 s): it is a noisy estimate on this car.
FCW is computed by the MPC and is untouched by all of it.

Settings live in a file, not Params: this branch is a prebuilt release, so Params keys are compiled
into params_pyx.so and unknown keys are deleted at boot (same as the fan page). A missing, unreadable
or invalid file means the defaults; every value is validated and clamped.
Settings are read once when plannerd starts (each drive), so nothing changes mid-drive, except that
switching glide or soft plan off takes effect within a second.
"""
import json
import math
import os
import shutil
import time
from dataclasses import asdict, dataclass, fields, replace

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths

MPH = 0.44704
# default Personalized follow-gap table (mph, s); long_mpc.py uses it in m/s. The values are REAL gaps (the driver's
# choice, 2026-10-29: "I want to set the real gap"): distance to the lead / speed, the MPC's 6 m stop distance included,
# so the planner's t_follow is the gap minus 6 m / speed (t_follow_for_gap). 1.1 s up to 40 mph (so cars don't cut in),
# blending to 1.2 s at 60 mph and up (2026-10-28: more held back on the highway and made them press the gas). Each
# point can still be moved by the learner.
DEFAULT_FOLLOW_BP_MPH = (35.0, 40.0, 60.0)
DEFAULT_FOLLOW_S = (1.1, 1.1, 1.2)
STOP_DISTANCE = 6.0         # m, long_mpc.py: steady-state gap = t_follow * v + STOP_DISTANCE
MIN_T_FOLLOW = 0.7          # s, the planner's t_follow never goes under this: below ~34 mph a 1.1 s real gap would need
                            # less, so there the real gap ends up a little longer than the table (1.15 s at 30 mph)
MIN_FOLLOW_S = 0.8          # s, follow-gap table limits (real gaps)
MAX_FOLLOW_S = 1.75
MAX_FOLLOW_BP_MPH = 40.0 / MPH  # 40 m/s
MAX_FOLLOW_POINTS = 6
MIN_GLIDE_SPEED = 5.0       # m/s (~11 mph), below this stop-and-go is left to the MPC
HARD_MIN_GAP_S = 0.6        # s, never glide closer than this, whatever the settings say
RELEASE_S = 0.15            # s of gap below the light zone over which braking is handed back to the MPC
LEAD_BRAKE_LIMIT = -1.0     # m/s^2, no glide while the lead itself brakes harder than this
BRAKE_ASSIST_A = -1.5       # m/s^2, brake assist acts while the lead brakes harder than this (radard's filtered estimate)
BRAKE_ASSIST_HOLD = 1.0     # s brake assist stays on after the lead's braking eases off
# brake assist stays off in Relaxed (log.LongitudinalPersonality.relaxed): its stock gap is long, and on the 2026-10-29
# bench (radard lead, lead braking 2.0 m/s2 from 45 mph to a stop) assist there braked at -2.60 m/s2 where the stock
# planner stayed above -2.5, which the strict gate fails
BRAKE_ASSIST_OFF_IN = (2, "relaxed")
LEAD_BRAKING_A = -0.5       # m/s^2, a lead decelerating more than this counts as still braking (radard's own threshold)
CLOSING_MIN = 0.3           # m/s, slower closing than this counts as not closing (TTC = inf)
CUT_IN_JUMP = 4.0           # m, a lead distance jump this big in one frame is a different car
STABLE_TIME = 1.0           # s the same lead must be seen before glide acts
BLOCK_HOLD_TIME = 1.0       # s glide stays off after a TTC or lead-braking gate trips
OVERSPEED_RAMP_MPH = 5.0    # mph past the overspeed allowance over which braking for speed alone ramps in, linearly
OVERSPEED_BRAKE = -1.0      # m/s^2, the most braking ever asked for because of speed alone (moderate, never hard)
OVERSPEED_COAST_MIN = -0.5  # m/s^2, the overspeed coast floor never asks for more decel than this (uphill)
APPROACH_START = 0.25       # the taper starts when the decel needed to arrive at the target gap is this fraction of approach_decel
APPROACH_ACCEL_HI = 1.5     # m/s^2, the acceleration cap where the taper starts (above anything the MPC asks for over 11 mph)
APPROACH_FILTER_S = 1.0     # s, first-order filter on the closing speed the taper uses (a Kalman estimate from distance here)
SETTINGS_CHECK_INTERVAL = 1.0  # s
STATE_WRITE_INTERVAL = 1.0     # s

BAND_RELATIVE = "relative"  # zones measured from the profile's target gap
BAND_FIXED = "fixed"        # zones at fixed gaps in seconds

BOOLS = ("personalized", "glide", "soft_plan", "approach", "brake_assist", "learn", "auto_apply")


@dataclass
class PersonalizedSettings:
  personalized: bool = True       # your gap table in the Aggressive slot (off = stock Aggressive, flat 1.25 s)
  follow_bp_mph: tuple = ()       # follow-gap table override (empty = the DEFAULT_FOLLOW_* table)
  follow_s: tuple = ()
  glide: bool = True              # in every distance mode
  band: str = BAND_RELATIVE
  coast_margin_s: float = 0.2     # relative: coast until the gap is this far under target
  light_width_s: float = 0.25     # relative: width of the light-brake zone below that
  fixed_coast_to_s: float = 1.25  # fixed: coast down to this gap
  fixed_light_to_s: float = 1.0   # fixed: brake lightly down to this gap
  light_brake: float = -0.8       # m/s^2, strongest braking asked for in the light zone
  min_ttc_s: float = 6.0          # s, glide only while time to collision is above this
  coast_accel: float = -0.3       # m/s^2, request while coasting (Camry's own coast on flat road)
  overspeed_glide_mph: float = 3.0  # coast instead of braking while within this many mph over the set speed or speed limit (0 = off)
  approach: bool = True           # ease off while closing on a slower lead and coast into the gap (no run-up)
  approach_decel: float = 0.5     # m/s^2, planned decel for closing in: lift off once arriving at the target gap needs this much
  soft_plan: bool = True
  soft_distance_factor: float = 0.3  # weight on holding the exact distance while in the glide band (1.0 = stock)
  brake_assist: bool = True       # react sooner when the car ahead brakes hard (in every distance mode; bench 2026-10-29)
  lead_brake_tau: float = 0.3     # brake assist: how fast the planner expects the lead's braking to fade (stock radar lead 1.5, vision lead 0.3)
  brake_ramp: float = 0.25        # brake assist: scale on the planner's accel-change and jerk weights (1.0 = stock ramp)
  assist_comfort_brake: float = 2.0  # brake assist: the planner's comfort brake while it acts (2.5 = stock; bench 2026-10-29)
  # planner shaping (in every distance mode; needs the rebuilt planner solver, which this branch ships); stock defaults
  mpc_comfort_brake: float = 2.5  # m/s^2 the desired distance plans to brake with: lower = starts slowing earlier and gentler
  mpc_glide_band_s: float = 0.0   # s under the target gap the planner itself plans to coast (0 = off)
  mpc_glide_weight: float = 1.0   # weight on the gap error inside that band (1 = stock)
  mpc_far_weight: float = 1.0     # weight on a gap longer than the target (lower = closes a big gap more gently)
  learn: bool = True              # run the learning job while parked (writes suggestions)
  auto_apply: bool = False        # let the learning job apply its suggestions itself (one step each, history kept)


# field -> (min, max). The glide ranges stay inside what the bench has covered (defaults and the learner's values):
# coasting always asks for some decel, the TTC gate never drops under 5 s, light braking is never weaker than -0.5.
LIMITS = {
  "coast_margin_s": (0.05, 0.5),
  "light_width_s": (0.1, 0.5),
  "fixed_coast_to_s": (0.8, 2.0),
  "fixed_light_to_s": (HARD_MIN_GAP_S, 1.8),
  "light_brake": (-1.5, -0.5),
  "min_ttc_s": (5.0, 10.0),
  "coast_accel": (-0.6, -0.1),
  "overspeed_glide_mph": (0.0, 10.0),
  "approach_decel": (0.3, 1.0),
  "soft_distance_factor": (0.1, 1.0),
  "lead_brake_tau": (0.1, 1.5),
  "brake_ramp": (0.25, 1.0),
  "mpc_comfort_brake": (1.5, 3.0),
  "assist_comfort_brake": (1.5, 3.0),
  "mpc_glide_band_s": (0.0, 0.5),
  "mpc_glide_weight": (0.05, 1.0),
  "mpc_far_weight": (0.1, 1.0),
}
MPC_SHAPING_STOCK = (2.5, 0.0, 1.0, 1.0)   # (comfort brake, glide band, glide weight, far weight): long_mpc's stock cost


def _default_dir() -> str:
  return Paths.comma_home() if PC else "/data"


def settings_path() -> str:
  return os.path.join(_default_dir(), "camry_long.json")


def suggested_path() -> str:
  return os.path.join(_default_dir(), "camry_long_suggested.json")


def history_dir() -> str:
  return os.path.join(_default_dir(), "camry_long_history")


def state_path() -> str:
  return "/dev/shm/camry_long_state.json" if os.path.isdir("/dev/shm") else os.path.join(_default_dir(), "camry_long_state.json")


def _num(v) -> bool:
  return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def parse_follow_table(bp, vals) -> tuple[tuple, tuple]:
  """Speeds (mph, increasing, 0-40 m/s) and gaps (s, clamped to MIN/MAX_FOLLOW_S); anything invalid means no override."""
  if not isinstance(bp, (list, tuple)) or not isinstance(vals, (list, tuple)):
    return (), ()
  if not (1 <= len(bp) == len(vals) <= MAX_FOLLOW_POINTS) or not all(_num(x) for x in (*bp, *vals)):
    return (), ()
  if any(not 0.0 <= x <= MAX_FOLLOW_BP_MPH for x in bp) or any(b <= a for a, b in zip(bp, bp[1:], strict=False)):
    return (), ()
  return tuple(float(x) for x in bp), tuple(float(min(max(v, MIN_FOLLOW_S), MAX_FOLLOW_S)) for v in vals)


def parse_settings(raw) -> PersonalizedSettings:
  """Anything not a valid value falls back to its default; numbers are clamped to LIMITS."""
  s = PersonalizedSettings()
  if not isinstance(raw, dict):
    return s
  for f in fields(PersonalizedSettings):
    if f.name not in raw or f.name in ("follow_bp_mph", "follow_s"):
      continue
    v = raw[f.name]
    if f.name in BOOLS:
      if isinstance(v, bool):
        setattr(s, f.name, v)
    elif f.name == "band":
      if v in (BAND_RELATIVE, BAND_FIXED):
        s.band = v
    elif _num(v):
      lo, hi = LIMITS[f.name]
      setattr(s, f.name, float(min(max(v, lo), hi)))
  s.follow_bp_mph, s.follow_s = parse_follow_table(raw.get("follow_bp_mph"), raw.get("follow_s"))
  # the light zone must sit below the coast zone, and coasting can't brake harder than light braking
  s.fixed_light_to_s = min(s.fixed_light_to_s, s.fixed_coast_to_s - 0.05)
  s.coast_accel = max(s.coast_accel, s.light_brake)
  return s


def read_settings(path: str | None = None) -> PersonalizedSettings:
  try:
    with open(path or settings_path()) as f:
      return parse_settings(json.loads(f.read(8192)))
  except Exception:
    return PersonalizedSettings()


def write_settings(s: PersonalizedSettings, path: str | None = None) -> None:
  s = parse_settings(asdict(s))
  path = path or settings_path()
  os.makedirs(os.path.dirname(path), exist_ok=True)
  tmp = f"{path}.tmp"
  with open(tmp, "w") as f:
    json.dump(asdict(s), f, indent=1)
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)


def read_json(path: str, limit: int = 65536):
  try:
    with open(path) as f:
      return json.loads(f.read(limit))
  except Exception:
    return None


# ---------- suggestions, apply and revert (used by the Personalized page and the learning job) ----------

HISTORY_KEEP = 5
# fields the learning job may suggest, with the largest change allowed per Apply
SUGGESTABLE = {"follow_s": 0.1, "coast_margin_s": 0.1, "light_width_s": 0.1, "light_brake": 0.2, "min_ttc_s": 1.0, "coast_accel": 0.2,
               "soft_distance_factor": 0.1}


def read_suggested(path: str | None = None) -> dict | None:
  """{"made": epoch, "routes": [...], "changes": [{"field", "from", "to", "why"}...], "stats": {...}} or None."""
  raw = read_json(path or suggested_path())
  return raw if isinstance(raw, dict) and isinstance(raw.get("changes"), list) else None


def _limited(field: str, cur, new):
  """new value moved at most the allowed step from cur (per point for the follow table)."""
  step = SUGGESTABLE[field]
  if field == "follow_s":
    if not (isinstance(new, (list, tuple)) and cur and len(cur) == len(new)):
      return cur
    return tuple(round(float(min(max(n, c - step), c + step)), 3) for c, n in zip(cur, new, strict=True))
  if not _num(new):
    return cur
  return round(float(min(max(new, cur - step), cur + step)), 3)


def apply_suggested(sug: dict, cur: PersonalizedSettings) -> PersonalizedSettings:
  """cur with the suggestion's changes, each limited to its step; the follow table needs matching breakpoints."""
  new = asdict(cur)
  if not cur.follow_bp_mph:
    bp, vals = follow_table_ms(replace(cur, personalized=True))
    new["follow_bp_mph"], new["follow_s"] = tuple(float(round(b / MPH)) for b in bp), tuple(vals)
  for ch in sug.get("changes", []):
    f = ch.get("field") if isinstance(ch, dict) else None
    if f not in SUGGESTABLE:
      continue
    if f == "follow_s":
      if tuple(ch.get("bp_mph", ())) != tuple(new["follow_bp_mph"]):
        continue
      new[f] = _limited(f, tuple(new["follow_s"]), ch.get("to"))
    else:
      new[f] = _limited(f, new[f], ch.get("to"))
  return parse_settings(new)


def save_with_history(s: PersonalizedSettings, path: str | None = None, hist: str | None = None) -> None:
  """Keeps the previous settings file (last HISTORY_KEEP) so the page can revert."""
  path = path or settings_path()
  hist = hist or history_dir()
  try:
    os.makedirs(hist, exist_ok=True)
    if os.path.exists(path):
      # copy, not move: the live file stays in place until write_settings replaces it atomically
      shutil.copy2(path, os.path.join(hist, f"{time.time_ns()}.json"))
    old = sorted(os.listdir(hist))
    for name in old[:-HISTORY_KEEP]:
      os.remove(os.path.join(hist, name))
  except Exception:
    cloudlog.exception("camry personalized: history")
  write_settings(s, path)


def revert(path: str | None = None, hist: str | None = None) -> PersonalizedSettings | None:
  """Restores the newest history file; returns the restored settings or None if there is none."""
  path = path or settings_path()
  hist = hist or history_dir()
  try:
    names = sorted(os.listdir(hist))
  except OSError:
    return None
  if not names:
    return None
  restored = read_settings(os.path.join(hist, names[-1]))
  write_settings(restored, path)
  os.remove(os.path.join(hist, names[-1]))
  return restored


def history_count(hist: str | None = None) -> int:
  try:
    return len(os.listdir(hist or history_dir()))
  except OSError:
    return 0


def t_follow_for_gap(gap_s: float, v_ego: float) -> float:
  """The MPC's t_follow that holds a REAL gap of gap_s (distance / speed) at v_ego: the MPC keeps
  t_follow * v + STOP_DISTANCE, so t_follow = gap - STOP_DISTANCE / v, never under MIN_T_FOLLOW."""
  return max(gap_s - STOP_DISTANCE / max(v_ego, 0.1), MIN_T_FOLLOW)


def mode_name(personality, path: str | None = None) -> str:
  """Display name of a longitudinal personality ("aggressive", "standard", "relaxed"): "Personalized" for the
  Aggressive slot while the Personalized profile is on. Never raises (selfdrived's distance-button alert uses it)."""
  name = str(personality).title()
  return "Personalized" if name == "Aggressive" and read_settings(path).personalized else name


def follow_table_ms(s: PersonalizedSettings):
  """(breakpoints m/s, real gaps s) for long_mpc.get_T_FOLLOW, which converts them with t_follow_for_gap. None =
  Personalized off (stock Aggressive); the default table when no override is set."""
  if not s.personalized:
    return None
  bp, vals = (s.follow_bp_mph, s.follow_s) if s.follow_bp_mph else (DEFAULT_FOLLOW_BP_MPH, DEFAULT_FOLLOW_S)
  return [x * MPH for x in bp], list(vals)


def zone_edges(s: PersonalizedSettings, t_follow: float) -> tuple[float, float]:
  """(coast_to, light_to) gap in seconds: coast above coast_to, light braking down to light_to."""
  if s.band == BAND_FIXED:
    return s.fixed_coast_to_s, s.fixed_light_to_s
  coast_to = t_follow - s.coast_margin_s
  return coast_to, coast_to - s.light_width_s


def glide_floor(s: PersonalizedSettings, gap_s: float, t_follow: float, accel_min: float) -> float | None:
  """Least braking allowed at this gap, or None to pass the MPC through."""
  if gap_s < HARD_MIN_GAP_S:
    return None
  coast_to, light_to = zone_edges(s, t_follow)
  coast = min(s.coast_accel, 0.0)
  if gap_s >= coast_to:
    return coast
  if gap_s >= light_to:
    frac = (coast_to - gap_s) / max(coast_to - light_to, 1e-3)
    return coast + frac * (s.light_brake - coast)
  if gap_s >= light_to - RELEASE_S:
    frac = (light_to - gap_s) / RELEASE_S
    return s.light_brake + frac * (accel_min - s.light_brake)
  return None


def overspeed_floor(s: PersonalizedSettings, over_ms: float, accel_coast: float) -> float | None:
  """Least braking allowed for speed alone when over the target by over_ms (m/s), or None to pass the MPC through.
  accel_coast: the car's own coast accel on the current grade (the planner's estimate). Within the allowance the floor
  is that coast, never above 0 (no throttle) and never below OVERSPEED_COAST_MIN; past it the floor drops linearly
  to OVERSPEED_BRAKE over OVERSPEED_RAMP_MPH and stays there: braking for speed alone is never hard."""
  allow = s.overspeed_glide_mph * MPH
  if allow <= 0.0 or over_ms <= 0.0:
    return None
  coast = min(0.0, max(accel_coast, OVERSPEED_COAST_MIN))
  if over_ms <= allow:
    return coast
  frac = min((over_ms - allow) / (OVERSPEED_RAMP_MPH * MPH), 1.0)
  return coast + frac * (OVERSPEED_BRAKE - coast)


def approach_cap(s: PersonalizedSettings, gap_m: float, closing: float, v_ego: float, t_follow: float) -> float | None:
  """Most acceleration allowed while closing on a lead (never below 0), or None for no cap. gap_m: distance to the
  lead; closing: m/s, positive when gaining on it. The decel needed to arrive at the lead's speed right at the target
  gap (the gap for the lead's speed) is closing^2 / (2 x the distance left to it). Under APPROACH_START of
  approach_decel there is no cap; from there the cap tapers linearly from APPROACH_ACCEL_HI down to 0 at approach_decel
  (pedal lifted, no more gaining speed) and stays at 0 beyond it; any slowing down is the MPC's. Inside the target gap
  the glide zones take over: no cap."""
  if not s.approach or closing <= CLOSING_MIN:
    return None
  room = gap_m - (t_follow * max(v_ego - closing, 0.0) + STOP_DISTANCE)
  if room <= 0.0:
    return None
  frac = closing * closing / (2.0 * room) / max(s.approach_decel, 1e-3)
  if frac <= APPROACH_START:
    return None
  if frac >= 1.0:
    return 0.0
  return APPROACH_ACCEL_HI * (1.0 - (frac - APPROACH_START) / (1.0 - APPROACH_START))


class GlideGovernor:
  """Per planner cycle: observe() before the MPC runs, distance_cost_factor() for its weights,
  apply() on its output. live=False (test bench) skips the settings and status files; pass settings."""
  def __init__(self, path: str | None = None, dt: float = DT_MDL, live: bool = True, settings: PersonalizedSettings | None = None):
    self.path = path
    self.dt = dt
    self.live = live
    self.settings = settings if settings is not None else (read_settings(path) if live else PersonalizedSettings())
    self._mtime = self._stat()
    self._next_check = 0.0
    self._next_state_write = 0.0
    self.pending_change = False
    if live:
      cloudlog.info(f"camry personalized settings: {asdict(self.settings)}")
    self.reset()

  def reset(self):
    self._reset_glide()
    self._assist_frames = 0
    self.assist = False

  def _reset_glide(self):
    self._stable_frames = 0
    self._block_frames = 0
    self._last_d = None
    self._ready = False
    self.zone = "off"
    self.gap_s = None
    self.t_follow = 0.0
    self.active = False
    self.approach_cap_a = None
    self._closing_f = 0.0

  def _stat(self):
    # write_settings replaces the file, so a new inode marks a change even when two writes share a coarse mtime tick
    try:
      st = os.stat(self.path or settings_path())
      return st.st_ino, st.st_mtime_ns, st.st_size
    except OSError:
      return None

  def refresh_settings(self, now: float | None = None):
    """Mid-drive only switching glide, soft plan, the approach taper or brake assist off is applied; everything else
    waits for the next drive."""
    if not self.live:
      return
    now = time.monotonic() if now is None else now
    if now < self._next_check:
      return
    self._next_check = now + SETTINGS_CHECK_INTERVAL
    mtime = self._stat()
    if mtime == self._mtime:
      return
    self._mtime = mtime
    new = read_settings(self.path) if mtime is not None else PersonalizedSettings()
    off = {k: False for k in ("glide", "soft_plan", "approach", "brake_assist") if getattr(self.settings, k) and not getattr(new, k)}
    if off:
      self.settings = replace(self.settings, **off)
      cloudlog.info(f"camry personalized: switched off mid-drive: {list(off)}")
    self.pending_change = new != self.settings

  def observe(self, v_ego: float, lead, t_follow: float, personality=None):
    """lead: radarState.leadOne; t_follow: the current distance mode's target gap; personality: the distance mode
    (selfdriveState.personality). Tracks the lead and the safety gates; decides if glide may act this cycle."""
    s = self.settings
    self._ready = False
    self.active = False
    # brake assist: while the lead brakes hard, and BRAKE_ASSIST_HOLD after, the MPC expects that braking to last
    # (lead_tau_cap) and may ramp its own braking up faster (mpc_ramp_factor)
    braking = s.brake_assist and personality not in BRAKE_ASSIST_OFF_IN and str(personality) not in BRAKE_ASSIST_OFF_IN and \
      lead.status and lead.modelProb > 0.5 and lead.aLeadK < BRAKE_ASSIST_A
    self._assist_frames = int(BRAKE_ASSIST_HOLD / self.dt) if braking else max(0, self._assist_frames - 1)
    self.assist = self._assist_frames > 0
    if not (s.glide or s.soft_plan or s.approach):
      self._reset_glide()
      return
    self.t_follow = t_follow

    lead_ok = lead.status and lead.modelProb > 0.5 and lead.dRel > 0.0
    if lead_ok and self._last_d is not None and abs(lead.dRel - self._last_d) > CUT_IN_JUMP:
      self._stable_frames = 0
    self._last_d = lead.dRel if lead_ok else None
    self._stable_frames = self._stable_frames + 1 if lead_ok else 0
    self.gap_s = (lead.dRel - STOP_DISTANCE) / max(v_ego, 0.1) if lead_ok else None

    ttc = lead.dRel / -lead.vRel if lead_ok and lead.vRel < -CLOSING_MIN else math.inf
    if lead_ok and (ttc < s.min_ttc_s or lead.aLeadK < LEAD_BRAKE_LIMIT):
      self._block_frames = int(BLOCK_HOLD_TIME / self.dt)

    # approach taper: needs a stable lead at speed, but not the glide gates (it only ever lowers acceleration).
    # The closing speed is filtered (restarted for a new lead) so a noisy estimate can't make the cap flicker.
    closing = -lead.vRel if lead_ok else 0.0
    if self._stable_frames <= 1:
      self._closing_f = closing
    else:
      self._closing_f += (closing - self._closing_f) * min(1.0, self.dt / APPROACH_FILTER_S)
    self.approach_cap_a = None
    if lead_ok and v_ego >= MIN_GLIDE_SPEED and self._stable_frames >= int(STABLE_TIME / self.dt):
      self.approach_cap_a = approach_cap(s, lead.dRel, self._closing_f, v_ego, t_follow)

    if not lead_ok or v_ego < MIN_GLIDE_SPEED:
      self.zone = "no lead" if not lead_ok else "slow"
      self._block_frames = max(0, self._block_frames - 1)   # a hold from a lead that is gone runs out too
    elif self._block_frames > 0:
      self._block_frames -= 1
      self.zone = "blocked"
    elif self._stable_frames < int(STABLE_TIME / self.dt):
      self.zone = "new lead"
    else:
      self._ready = True
      self.zone = self._zone_name()

  def _zone_name(self) -> str:
    coast_to, light_to = zone_edges(self.settings, self.t_follow)
    if self.gap_s < HARD_MIN_GAP_S:
      return "brake"
    if self.gap_s >= coast_to:
      return "coast"
    if self.gap_s >= light_to:
      return "light"
    return "release" if self.gap_s >= light_to - RELEASE_S else "brake"

  def distance_cost_factor(self, source_is_lead: bool) -> float:
    """Soft plan: scale on the MPC's exact-distance weight. 1.0 (stock) unless soft plan is on and glide
    may act; then soft_distance_factor in the coast zone, back to 1.0 across the light zone."""
    s = self.settings
    if not (self._ready and s.soft_plan and source_is_lead):
      return 1.0
    coast_to, light_to = zone_edges(s, self.t_follow)
    if self.gap_s >= coast_to:
      return s.soft_distance_factor
    if self.gap_s >= light_to:
      frac = (coast_to - self.gap_s) / max(coast_to - light_to, 1e-3)
      return s.soft_distance_factor + frac * (1.0 - s.soft_distance_factor)
    return 1.0

  def mpc_ramp_factor(self) -> float:
    """Brake assist: scale on the MPC's accel-change and jerk weights this cycle (1.0 = stock)."""
    return self.settings.brake_ramp if self.assist else 1.0

  def lead_tau_cap(self) -> float | None:
    """Brake assist: the most the MPC may assume for the decay of a braking lead's decel (None = radard's value)."""
    return self.settings.lead_brake_tau if self.assist else None

  def mpc_shaping(self) -> tuple:
    """(comfort brake, glide band s, glide weight, far weight) for the rebuilt planner solver. While brake assist acts
    the comfort brake is at most assist_comfort_brake: the planner plans to start braking earlier for a hard-braking lead
    without changing how it follows or closes in otherwise."""
    s = self.settings
    comfort_brake = min(s.mpc_comfort_brake, s.assist_comfort_brake) if self.assist else s.mpc_comfort_brake
    return comfort_brake, s.mpc_glide_band_s, s.mpc_glide_weight, s.mpc_far_weight

  def apply(self, a_target: float, source_is_lead: bool, accel_min: float) -> float:
    """Returns the adjusted accel target: the approach taper can only lower a positive acceleration request (never
    below 0), glide can only make a braking request less firm. Never adds throttle, never adds braking."""
    self.active = False
    out = a_target
    if self.approach_cap_a is not None and self.settings.approach and out > self.approach_cap_a:
      out = self.approach_cap_a
      self.zone = "approach"
      self.active = True
    if not (self._ready and self.settings.glide):
      return out
    if not source_is_lead:
      if not self.active:
        self.zone = "not lead"
      return out
    floor = glide_floor(self.settings, self.gap_s, self.t_follow, accel_min)
    if floor is None:
      return out
    self.active = self.active or out < floor
    return max(out, floor)

  def apply_overspeed(self, a_target: float, v_ego: float, v_target: float, accel_coast: float, speed_is_the_reason: bool) -> float:
    """Glide over set speed. speed_is_the_reason: the MPC's source is cruise (no lead limiting) and the target is the set
    speed or the speed limit (not a curve), so any braking is only to get back down to v_target. Then the request is
    lifted to the overspeed floor (coast within the allowance, linearly firmer past it, capped). Never adds throttle,
    nothing while a lead gate (TTC, lead braking) is holding glide off, nothing under MIN_GLIDE_SPEED."""
    s = self.settings
    if not (s.glide and s.overspeed_glide_mph > 0.0 and speed_is_the_reason) or v_ego < MIN_GLIDE_SPEED or self._block_frames > 0:
      return a_target
    floor = overspeed_floor(s, v_ego - v_target, accel_coast)
    if floor is None:
      return a_target
    self.zone = "overspeed"
    self.active = a_target < floor
    return max(a_target, floor)

  def update(self, a_target: float, v_ego: float, lead, t_follow: float, source_is_lead: bool, accel_min: float) -> float:
    self.observe(v_ego, lead, t_follow)
    return self.apply(a_target, source_is_lead, accel_min)

  def write_state(self, now: float | None = None):
    """Small status file for the Personalized page, at most once a second."""
    if not self.live:
      return
    now = time.monotonic() if now is None else now
    if now < self._next_state_write:
      return
    self._next_state_write = now + STATE_WRITE_INTERVAL
    try:
      path = state_path()
      tmp = f"{path}.tmp"
      with open(tmp, "w") as f:
        json.dump({"t": time.time(), "glide": self.settings.glide, "zone": self.zone, "active": self.active,
                   "gap_s": None if self.gap_s is None else round(self.gap_s, 2), "pending_change": self.pending_change}, f)
      os.replace(tmp, path)
    except Exception:
      pass


class SafeGlide:
  """Fail-safe wrapper used by the planner: any exception in the governor is logged once, glide, soft plan and brake
  assist are then off for the rest of the drive, and the planner gets the plain MPC output (factors 1.0, no tau cap,
  a_target as is).
  Attributes (settings, zone, active, ...) read through to the governor."""
  def __init__(self, governor):
    self.governor = governor
    self.failed = False

  def __getattr__(self, name):
    return getattr(self.governor, name)

  def _fail(self, where: str):
    if not self.failed:
      cloudlog.exception(f"camry personalized: {where} failed, glide and brake assist off for this drive")
    self.failed = True
    try:
      self.governor.reset()
    except Exception:
      pass

  def refresh_settings(self, now: float | None = None):
    if self.failed:
      return
    try:
      self.governor.refresh_settings(now)
    except Exception:
      self._fail("refresh_settings")

  def reset(self):
    try:
      self.governor.reset()
    except Exception:
      self._fail("reset")

  def observe(self, v_ego: float, lead, t_follow: float, personality=None):
    if self.failed:
      return
    try:
      self.governor.observe(v_ego, lead, t_follow, personality)
    except Exception:
      self._fail("observe")

  def mpc_ramp_factor(self) -> float:
    if self.failed:
      return 1.0
    try:
      f = float(self.governor.mpc_ramp_factor())
      lo, hi = LIMITS["brake_ramp"]
      return f if lo <= f <= hi else 1.0
    except Exception:
      self._fail("mpc_ramp_factor")
      return 1.0

  def lead_tau_cap(self) -> float | None:
    if self.failed:
      return None
    try:
      cap = self.governor.lead_tau_cap()
      if cap is None:
        return None
      lo, hi = LIMITS["lead_brake_tau"]
      return float(cap) if lo <= float(cap) <= hi else None
    except Exception:
      self._fail("lead_tau_cap")
      return None

  def mpc_shaping(self) -> tuple:
    if self.failed:
      return MPC_SHAPING_STOCK
    try:
      vals = tuple(float(v) for v in self.governor.mpc_shaping())
      keys = ("mpc_comfort_brake", "mpc_glide_band_s", "mpc_glide_weight", "mpc_far_weight")
      ok = len(vals) == len(keys) and all(LIMITS[k][0] <= v <= LIMITS[k][1] for k, v in zip(keys, vals, strict=False))
      return vals if ok else MPC_SHAPING_STOCK
    except Exception:
      self._fail("mpc_shaping")
      return MPC_SHAPING_STOCK

  def distance_cost_factor(self, source_is_lead: bool) -> float:
    if self.failed:
      return 1.0
    try:
      f = float(self.governor.distance_cost_factor(source_is_lead))
      return f if 0.1 <= f <= 1.0 else 1.0
    except Exception:
      self._fail("distance_cost_factor")
      return 1.0

  def apply(self, a_target: float, source_is_lead: bool, accel_min: float) -> float:
    if self.failed:
      return a_target
    try:
      out = float(self.governor.apply(a_target, source_is_lead, accel_min))
      # never throttle the MPC didn't ask for, never more braking than it asked for: an acceleration request may only
      # come down (to 0 at the least), a braking request may only come up (to 0 at the most)
      return out if math.isfinite(out) and min(a_target, 0.0) <= out <= max(a_target, 0.0) else a_target
    except Exception:
      self._fail("apply")
      return a_target

  def apply_overspeed(self, a_target: float, v_ego: float, v_target: float, accel_coast: float, speed_is_the_reason: bool) -> float:
    if self.failed:
      return a_target
    try:
      out = float(self.governor.apply_overspeed(a_target, v_ego, v_target, accel_coast, speed_is_the_reason))
      return out if math.isfinite(out) and a_target <= out <= max(a_target, 0.0) else a_target
    except Exception:
      self._fail("apply_overspeed")
      return a_target

  def update(self, a_target: float, v_ego: float, lead, t_follow: float, source_is_lead: bool, accel_min: float) -> float:
    self.observe(v_ego, lead, t_follow)
    return self.apply(a_target, source_is_lead, accel_min)

  def write_state(self, now: float | None = None):
    if self.failed:
      return
    try:
      self.governor.write_state(now)
    except Exception:
      self._fail("write_state")


def read_state(path: str | None = None, max_age: float = 3.0) -> dict | None:
  st = read_json(path or state_path(), 1024)
  return st if isinstance(st, dict) and time.time() - st.get("t", 0) < max_age else None
