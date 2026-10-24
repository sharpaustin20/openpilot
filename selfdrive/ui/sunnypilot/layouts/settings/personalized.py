"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
from dataclasses import asdict

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import camry_personalized as cp
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import ListItemSP, button_item_sp, toggle_item_sp
from openpilot.system.ui.sunnypilot.widgets.option_control import OptionControlSP, LABEL_WIDTH
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import ItemAction, text_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
import pyray as rl

MPH = 2.237

# numeric settings shown as -/+ controls: field -> (choices, label)
NUMBER_OPTIONS = {
  "coast_margin_s": ([round(0.05 * i, 2) for i in range(11)], lambda v: f"{v:.2f} s"),
  "light_width_s": ([round(0.1 + 0.05 * i, 2) for i in range(9)], lambda v: f"{v:.2f} s"),
  "fixed_coast_to_s": ([round(0.8 + 0.05 * i, 2) for i in range(25)], lambda v: f"{v:.2f} s"),
  "fixed_light_to_s": ([round(0.6 + 0.05 * i, 2) for i in range(25)], lambda v: f"{v:.2f} s"),
  "light_brake": ([round(-1.5 + 0.1 * i, 1) for i in range(13)], lambda v: f"{v:.1f} m/s²"),
  "min_ttc_s": ([float(i) for i in range(3, 11)], lambda v: f"{v:.0f} s"),
  "coast_accel": ([round(-0.6 + 0.1 * i, 1) for i in range(7)], lambda v: f"{v:.1f} m/s²"),
  "soft_distance_factor": ([round(0.1 * i, 1) for i in range(1, 11)], lambda v: f"{v:.1f}"),
}

FIELD_NAMES = {
  "follow_s": "follow gap", "coast_margin_s": "coast zone starts", "light_width_s": "light-brake zone width",
  "light_brake": "light braking limit", "min_ttc_s": "glide only above TTC", "coast_accel": "coast decel",
  "soft_distance_factor": "soft plan strength",
}


class SettingsStore:
  """The one copy of the settings file the page edits. Every write goes through save()."""
  def __init__(self):
    self.s = cp.read_settings()

  def refresh(self):
    self.s = cp.read_settings()

  def save(self, **changes):
    try:
      self.s = cp.parse_settings({**asdict(self.s), **changes})
      cp.write_settings(self.s)
    except Exception:
      cloudlog.exception("personalized: failed to save settings")


class NumberControl(OptionControlSP):
  """OptionControlSP over one numeric field of the settings file instead of a Params key."""

  def __init__(self, store: SettingsStore, field: str):
    ItemAction.__init__(self, enabled=True)
    self.store = store
    self.field = field
    self.choices, self.fmt = NUMBER_OPTIONS[field]
    self.param_key = ""
    self.min_value = 0
    self.max_value = len(self.choices) - 1
    self.value_change_step = 1
    self._minus_enabled = True
    self._plus_enabled = True
    self.on_value_changed = None
    self.value_map = None
    self.label_width = LABEL_WIDTH
    self.use_float_scaling = False
    self.label_callback = lambda idx: self.fmt(self.choices[idx])
    self.current_value = 0
    self._font = gui_app.font(FontWeight.MEDIUM)
    self.minus_btn_rect = rl.Rectangle(0, 0, 0, 0)
    self.plus_btn_rect = rl.Rectangle(0, 0, 0, 0)
    self.refresh()

  def refresh(self):
    v = getattr(self.store.s, self.field)
    self.current_value = min(range(len(self.choices)), key=lambda i: abs(self.choices[i] - v))

  def set_value(self, value: int):
    if not (self.min_value <= value <= self.max_value) or value == self.current_value:
      return
    self.store.save(**{self.field: self.choices[value]})
    self.refresh()


def _fmt_table(bp_mph, vals) -> str:
  return ", ".join(f"{v:.2f} s @ {b:.0f} mph" for b, v in zip(bp_mph, vals, strict=True))


def _per_hour(x) -> str:
  return "--" if x is None else f"{x:.1f}/h"


class PersonalizedLayout(Widget):
  def __init__(self):
    super().__init__()
    self.store = SettingsStore()
    self._sug = None
    self._sug_ts = 0.
    self._status = ""
    self._scroller = Scroller(self._initialize_items(), line_separator=True, spacing=0)

  def _toggle(self, field: str, title: str, description: str) -> ListItemSP:
    item = toggle_item_sp(lambda: tr(title), lambda: tr(description), initial_state=getattr(self.store.s, field),
                          callback=lambda state, f=field: self.store.save(**{f: state}))
    return item

  def _number(self, field: str, title: str, description: str) -> ListItemSP:
    ctrl = NumberControl(self.store, field)
    self._numbers.append(ctrl)
    return ListItemSP(title=lambda: tr(title), description=lambda: tr(description), action_item=ctrl, inline=True)

  def _initialize_items(self):
    self._numbers: list[NumberControl] = []
    self._toggles: dict[str, ListItemSP] = {}
    for field, title, desc in (
      ("personalized", "Personalized Profile",
       "Your gap table in the Aggressive position of the distance button: Personalized, Standard, Relaxed. On by default; " +
       "off = stock Aggressive (flat 1.25 s gap). Changes here apply at the next drive; only switching Glide or Soft Plan off applies right away."),
      ("glide", "Glide",
       "In every distance mode: behind a lead, coast instead of braking while the gap is in the coast zone under that mode's target, " +
       "brake lightly in the zone below it, and brake normally below that. Off when closing fast, when the lead brakes hard, " +
       "right after a cut-in, or below 11 mph. Only ever asks for less braking; forward collision warning is unchanged."),
      ("soft_plan", "Soft Plan",
       "The planner itself cares less about holding the exact distance while in the glide band, so it plans the glide " +
       "instead of braking to win the gap back. The too-close penalty is unchanged. On by default; the learner tunes its " +
       "strength: firmer after brake takeovers inside the glide band, softer after an hour engaged without one."),
      ("learn", "Learn After Drives",
       "While parked (10 min after the car is off, battery above 12.2 V, device below 60 °C), look at your gas presses, brake takeovers, " +
       "distance-button changes and manual driving, and suggest small changes below."),
      ("auto_apply", "Apply Automatically",
       "Let the learner apply its suggestions itself after each parked run: one small step per setting, with the previous " +
       "settings kept so Undo works. Off = the page shows them and you tap Apply."),
    ):
      self._toggles[field] = self._toggle(field, title, desc)

    items = [
      self._toggles["personalized"],
      text_item(lambda: tr("Follow Gap"), self._follow_text,
                description=lambda: tr("Gap in seconds by speed. The default is fitted to your manual following; Apply below can tune it.")),
      self._toggles["glide"],
      self._number("coast_margin_s", "Coast Zone Starts", "How far under the target gap coasting is allowed to start (relative band)."),
      self._number("light_width_s", "Light-Brake Zone Width",
                   "Below the coast zone, braking ramps up to the light braking limit over this many seconds of gap."),
      self._number("light_brake", "Light Braking Limit", "Strongest braking asked for in the light-brake zone."),
      self._number("coast_accel", "Coast Decel", "What to ask for while coasting. -0.3 m/s² is about the Camry's own coast on a flat road."),
      self._number("min_ttc_s", "Glide Only Above TTC", "No glide when time to collision with the lead is under this."),
      self._toggles["soft_plan"],
      self._number("soft_distance_factor", "Soft Plan Strength", "Weight on holding the exact distance while in the glide band. 1.0 = stock, lower = softer."),
      text_item(lambda: tr("Live"), self._live_text),
      self._toggles["learn"],
      text_item(lambda: tr("Interventions"), self._interventions_text,
                description=lambda: tr("Gas presses and brake takeovers per engaged hour, last drive and last 5 drives.")),
      self._toggles["auto_apply"],
      button_item_sp(lambda: tr("Suggested From Your Drives"), lambda: tr("Apply"), self._suggested_text, callback=self._apply),
      button_item_sp(lambda: tr("Undo Last Apply"), lambda: tr("Revert"), self._history_text, callback=self._revert),
    ]
    return items

  # ---- texts ----
  def _follow_text(self) -> str:
    s = self.store.s
    if not s.personalized:
      return tr("stock 1.25 s (Personalized off)")
    bp, vals = cp.follow_table_ms(s)
    return _fmt_table([b * MPH for b in bp], vals) + ("" if s.follow_bp_mph else tr(" (default)"))

  def _live_text(self) -> str:
    st = cp.read_state()
    if st is None:
      return tr("not driving")
    gap = f"{st['gap_s']:.2f} s" if st.get("gap_s") is not None else "--"
    txt = f"{tr('gap')} {gap}, {st.get('zone', '')}" + (tr(", gliding") if st.get("active") else "")
    if st.get("pending_change"):
      txt += tr(" (new settings at next drive)")
    return txt

  def _refresh_sug(self):
    now = time.monotonic()
    if now - self._sug_ts < 2.0:
      return
    self._sug_ts = now
    self._sug = cp.read_suggested()

  def _interventions_text(self) -> str:
    self._refresh_sug()
    stats = (self._sug or {}).get("stats") or {}
    last, five = stats.get("per_hour_last"), stats.get("per_hour_5")
    if last is None and five is None:
      return "--"
    return f"{tr('last')} {_per_hour(last)}, {tr('last 5')} {_per_hour(five)}"

  def _suggested_text(self) -> str:
    self._refresh_sug()
    if self._status:
      return self._status
    if not self._sug:
      return tr("none yet")
    ch = self._sug.get("changes", [])
    gate = self._sug.get("gate") or {}
    if not ch:
      if self._sug.get("dropped") and gate.get("verdict") == "fail":
        return tr("dropped: the bench showed more FCW, crashes or a smaller gap: ") + "; ".join(gate.get("worse", [])[:2])
      return tr("no change suggested")
    parts = [tr("applied automatically after your last drive (Undo below):")] if self._sug.get("applied") else []
    if gate.get("verdict") != "pass":
      parts.append(tr("not benched yet"))
    for c in ch[:3]:
      name = FIELD_NAMES.get(c.get("field"), c.get("field"))
      if c.get("field") == "follow_s":
        parts.append(f"{name}: {_fmt_table(c.get('bp_mph', ()), c.get('to', ()))}")
      else:
        parts.append(f"{name}: {c.get('from')} -> {c.get('to')}")
      if c.get("why"):
        parts[-1] += f" ({c['why']})"
    return "<br>".join(parts)

  def _history_text(self) -> str:
    n = cp.history_count()
    return tr("nothing to undo") if n == 0 else f"{n} " + tr("earlier version(s) kept")

  # ---- actions ----
  def _apply(self):
    self._refresh_sug()
    if not self._sug or not self._sug.get("changes") or self._sug.get("applied"):
      return
    try:
      new = cp.apply_suggested(self._sug, self.store.s)
      cp.save_with_history(new)
      self.store.refresh()
      for n in self._numbers:
        n.refresh()
      self._status = tr("applied; takes effect at the next drive")
      self._sug_ts = 0.
    except Exception:
      cloudlog.exception("personalized: apply failed")
      self._status = tr("apply failed")

  def _revert(self):
    try:
      if cp.revert() is not None:
        self.store.refresh()
        for n in self._numbers:
          n.refresh()
        self._sync_toggles()
        self._status = tr("reverted; takes effect at the next drive")
    except Exception:
      cloudlog.exception("personalized: revert failed")

  def _sync_toggles(self):
    for field, item in self._toggles.items():
      item.action_item.set_state(getattr(self.store.s, field))

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self.store.refresh()
    self._status = ""
    for n in self._numbers:
      n.refresh()
    self._sync_toggles()
    self._scroller.show_event()
