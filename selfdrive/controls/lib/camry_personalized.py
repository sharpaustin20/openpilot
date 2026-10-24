"""
Camry following: the Personalized profile in the Aggressive slot of the distance button, and glide in
every distance mode.

Everything is set in /data/camry_long.json (the Personalized page) and tuned over time from the
learning job's suggestions:
- Personalized (on by default): a speed-based t_follow table (DEFAULT_FOLLOW_* below) for the
  Aggressive slot; off = stock Aggressive (flat 1.25 s). Standard and Relaxed keep their stock gaps.
- glide (option A, on by default, every mode): the MPC penalizes being too close as much as being too
  far, so it brakes to win back every bit of gap. Behind a lead in the same lane, glide raises small
  braking requests to the car's coast decel while the gap is in a coast zone under the current mode's
  target, caps braking in a light zone below it, and passes the MPC output through unchanged below the
  band, when closing fast, when the lead brakes, right after a cut-in, or for anything that isn't the
  lead (curves, stops, e2e). It only ever makes a braking request less firm.
- soft plan (option C1, on by default): while glide may act, the MPC's weight on holding the exact
  distance is lowered, so the plan itself glides. Its one-sided danger-zone penalty is unchanged. The
  learning job tunes the strength (firmer after brake takeovers inside the band, softer without them).
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
# default Personalized follow-gap table (mph, s), fitted to the driver's own manual following; long_mpc.py uses it in m/s
DEFAULT_FOLLOW_BP_MPH = (35.0, 40.0, 60.0)
DEFAULT_FOLLOW_S = (0.8, 1.0, 1.45)
STOP_DISTANCE = 6.0         # m, long_mpc.py: steady-state gap = t_follow * v + STOP_DISTANCE
MIN_FOLLOW_S = 0.8          # s, follow-gap table limits
MAX_FOLLOW_S = 1.75
MAX_FOLLOW_BP_MPH = 40.0 / MPH  # 40 m/s
MAX_FOLLOW_POINTS = 6
MIN_GLIDE_SPEED = 5.0       # m/s (~11 mph), below this stop-and-go is left to the MPC
HARD_MIN_GAP_S = 0.6        # s, never glide closer than this, whatever the settings say
RELEASE_S = 0.15            # s of gap below the light zone over which braking is handed back to the MPC
LEAD_BRAKE_LIMIT = -1.0     # m/s^2, no glide while the lead itself brakes harder than this
CLOSING_MIN = 0.3           # m/s, slower closing than this counts as not closing (TTC = inf)
CUT_IN_JUMP = 4.0           # m, a lead distance jump this big in one frame is a different car
STABLE_TIME = 1.0           # s the same lead must be seen before glide acts
BLOCK_HOLD_TIME = 1.0       # s glide stays off after a TTC or lead-braking gate trips
SETTINGS_CHECK_INTERVAL = 1.0  # s
STATE_WRITE_INTERVAL = 1.0     # s

BAND_RELATIVE = "relative"  # zones measured from the profile's target gap
BAND_FIXED = "fixed"        # zones at fixed gaps in seconds

BOOLS = ("personalized", "glide", "soft_plan", "learn", "auto_apply")


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
  soft_plan: bool = True
  soft_distance_factor: float = 0.3  # weight on holding the exact distance while in the glide band (1.0 = stock)
  learn: bool = True              # run the learning job while parked (writes suggestions)
  auto_apply: bool = False        # let the learning job apply its suggestions itself (one step each, history kept)


# field -> (min, max)
LIMITS = {
  "coast_margin_s": (0.0, 0.5),
  "light_width_s": (0.1, 0.5),
  "fixed_coast_to_s": (0.8, 2.0),
  "fixed_light_to_s": (HARD_MIN_GAP_S, 1.8),
  "light_brake": (-1.5, -0.3),
  "min_ttc_s": (3.0, 10.0),
  "coast_accel": (-0.6, 0.0),
  "soft_distance_factor": (0.1, 1.0),
}


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


def follow_table_ms(s: PersonalizedSettings):
  """(breakpoints m/s, gaps s) for long_mpc.get_T_FOLLOW. None = Personalized off (stock Aggressive);
  the default table when no override is set."""
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
    self._stable_frames = 0
    self._block_frames = 0
    self._last_d = None
    self._ready = False
    self.zone = "off"
    self.gap_s = None
    self.t_follow = 0.0
    self.active = False

  def _stat(self):
    try:
      return os.stat(self.path or settings_path()).st_mtime_ns
    except OSError:
      return None

  def refresh_settings(self, now: float | None = None):
    """Mid-drive only switching glide or soft plan off is applied; everything else waits for the next drive."""
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
    off = {k: False for k in ("glide", "soft_plan") if getattr(self.settings, k) and not getattr(new, k)}
    if off:
      self.settings = replace(self.settings, **off)
      cloudlog.info(f"camry personalized: switched off mid-drive: {list(off)}")
    self.pending_change = new != self.settings

  def observe(self, v_ego: float, lead, t_follow: float):
    """lead: radarState.leadOne; t_follow: the current distance mode's target gap. Tracks the lead and the
    safety gates; decides if glide may act this cycle."""
    s = self.settings
    self._ready = False
    self.active = False
    if not (s.glide or s.soft_plan):
      self.reset()
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

    if not lead_ok or v_ego < MIN_GLIDE_SPEED:
      self.zone = "no lead" if not lead_ok else "slow"
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

  def apply(self, a_target: float, source_is_lead: bool, accel_min: float) -> float:
    """Returns the (possibly less firm) accel target. Never adds throttle, never makes braking firmer."""
    self.active = False
    if not (self._ready and self.settings.glide):
      return a_target
    if not source_is_lead:
      self.zone = "not lead"
      return a_target
    floor = glide_floor(self.settings, self.gap_s, self.t_follow, accel_min)
    if floor is None:
      return a_target
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
  """Fail-safe wrapper used by the planner: any exception in the governor is logged once, glide and soft plan
  are then off for the rest of the drive, and the planner gets the plain MPC output (factor 1.0, a_target as is).
  Attributes (settings, zone, active, ...) read through to the governor."""
  def __init__(self, governor):
    self.governor = governor
    self.failed = False

  def __getattr__(self, name):
    return getattr(self.governor, name)

  def _fail(self, where: str):
    if not self.failed:
      cloudlog.exception(f"camry personalized: {where} failed, glide off for this drive")
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

  def observe(self, v_ego: float, lead, t_follow: float):
    if self.failed:
      return
    try:
      self.governor.observe(v_ego, lead, t_follow)
    except Exception:
      self._fail("observe")

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
      # never more braking than the MPC asked for, never throttle it didn't ask for
      return out if math.isfinite(out) and a_target <= out <= max(a_target, 0.0) else a_target
    except Exception:
      self._fail("apply")
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
