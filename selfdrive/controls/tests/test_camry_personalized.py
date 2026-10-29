import json
import os
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import camry_personalized as cp
from openpilot.selfdrive.controls.lib.camry_personalized import (BAND_FIXED, GlideGovernor, HARD_MIN_GAP_S, MAX_FOLLOW_S, MIN_FOLLOW_S,
                                                                  PersonalizedSettings, glide_floor, parse_settings, read_settings,
                                                                  write_settings)

ACCEL_MIN = -3.5
T_FOLLOW = 1.45


def lead(gap_s, v_ego=26.8, vrel=-1.0, a_lead=0.0, prob=0.9):
  return SimpleNamespace(status=True, modelProb=prob, dRel=gap_s * v_ego + 6.0, vRel=vrel, aLeadK=a_lead)


def governor(**kw):
  return GlideGovernor(live=False, settings=PersonalizedSettings(personalized=True, glide=True, **kw))


def run(g, frames, a_target, ld, v_ego=26.8, source_is_lead=True, t_follow=T_FOLLOW):
  out = a_target
  for _ in range(frames):
    out = g.update(a_target, v_ego, ld, t_follow, source_is_lead, ACCEL_MIN)
  return out


class TestSettings:
  def test_missing_or_bad_is_default(self, tmp_path):
    assert read_settings(str(tmp_path / "missing.json")) == PersonalizedSettings()
    p = tmp_path / "bad.json"
    p.write_text("{not json")
    assert read_settings(str(p)) == PersonalizedSettings()
    d = PersonalizedSettings()
    assert d.personalized and d.glide and d.soft_plan and d.learn and not d.auto_apply   # Personalized replaces Aggressive; glide everywhere
    assert parse_settings([1, 2]) == PersonalizedSettings()

  def test_clamped(self):
    s = parse_settings({"glide": True, "light_brake": -9.0, "min_ttc_s": 0.5, "coast_accel": 2.0, "band": "weird",
                        "coast_margin_s": float("nan"), "fixed_coast_to_s": 1.0, "fixed_light_to_s": 1.5})
    assert s.glide and s.light_brake == -1.5 and s.min_ttc_s == 5.0 and s.coast_accel == -0.1
    assert s.band == "relative" and s.coast_margin_s == PersonalizedSettings().coast_margin_s
    loose = parse_settings({"coast_margin_s": 0.0, "light_brake": -0.3, "min_ttc_s": 3.0, "coast_accel": 0.0})
    assert (loose.coast_margin_s, loose.light_brake, loose.min_ttc_s, loose.coast_accel) == (0.05, -0.5, 5.0, -0.1)   # the loosest allowed
    assert s.fixed_light_to_s < s.fixed_coast_to_s
    assert parse_settings({"glide": 1, "soft_plan": 1}) == PersonalizedSettings()  # a non-bool falls back to the default

  def test_follow_table(self):
    s = parse_settings({"follow_bp_mph": [35, 40, 60], "follow_s": [0.5, 1.0, 3.0]})
    assert s.follow_bp_mph == (35.0, 40.0, 60.0) and s.follow_s == (MIN_FOLLOW_S, 1.0, MAX_FOLLOW_S)
    for bp, vals in (([40, 35], [1.0, 1.0]), ([35], [1.0, 1.1]), ([35, 200], [1.0, 1.0]), ([35, float("nan")], [1.0, 1.0]),
                     ([1, 2, 3, 4, 5, 6, 7], [1.0] * 7), ("35", "1.0")):
      assert parse_settings({"follow_bp_mph": bp, "follow_s": vals}).follow_bp_mph == ()

  def test_round_trip(self, tmp_path):
    p = str(tmp_path / "camry_glide.json")
    s = PersonalizedSettings(personalized=True, glide=True, band=BAND_FIXED, light_brake=-0.6, follow_bp_mph=(40.0,), follow_s=(1.1,))
    write_settings(s, p)
    assert read_settings(p) == s
    with open(p) as f:
      assert json.load(f)["glide"] is True


class TestFloor:
  def test_zones_relative(self):
    s = PersonalizedSettings(glide=True)
    assert glide_floor(s, 1.6, T_FOLLOW, ACCEL_MIN) == s.coast_accel           # above target: coast
    assert glide_floor(s, 1.3, T_FOLLOW, ACCEL_MIN) == s.coast_accel           # 0.15 s under target: still coast
    mid = glide_floor(s, 1.125, T_FOLLOW, ACCEL_MIN)
    assert s.light_brake < mid < s.coast_accel                                  # light zone blends
    assert glide_floor(s, 0.95, T_FOLLOW, ACCEL_MIN) < s.light_brake           # handing back to the MPC
    assert glide_floor(s, 0.8, T_FOLLOW, ACCEL_MIN) is None                    # below the band
    assert glide_floor(s, HARD_MIN_GAP_S - 0.01, 0.8, ACCEL_MIN) is None       # hard minimum gap

  def test_zones_fixed(self):
    s = PersonalizedSettings(glide=True, band=BAND_FIXED)
    assert glide_floor(s, 1.3, T_FOLLOW, ACCEL_MIN) == s.coast_accel
    assert glide_floor(s, 0.99, T_FOLLOW, ACCEL_MIN) <= s.light_brake
    assert glide_floor(s, 0.8, T_FOLLOW, ACCEL_MIN) is None


class TestGovernor:
  def test_off_passes_through(self):
    g = GlideGovernor(live=False, settings=PersonalizedSettings(glide=False))
    assert run(g, 40, -1.0, lead(1.3)) == -1.0

  def test_every_mode_uses_its_own_gap(self):
    g = governor()
    assert run(g, 40, -1.0, lead(1.4), t_follow=1.25) == g.settings.coast_accel          # Personalized gap: coast zone
    assert run(g, 40, -1.0, lead(1.4), t_follow=1.75) == pytest.approx(-0.6, abs=0.02)   # Relaxed gap: light zone

  def test_coasts_instead_of_braking(self):
    g = governor()
    assert run(g, 40, -1.0, lead(1.3)) == g.settings.coast_accel
    assert g.zone == "coast" and g.active

  def test_never_adds_accel(self):
    g = governor()
    assert run(g, 40, -0.1, lead(1.3)) == -0.1
    assert run(g, 40, 0.5, lead(1.3)) == 0.5

  def test_firm_braking_below_band(self):
    g = governor()
    assert run(g, 40, -2.0, lead(0.8)) == -2.0

  def test_waits_for_stable_lead(self):
    g = governor()
    assert run(g, 5, -1.0, lead(1.3)) == -1.0
    assert g.zone == "new lead"

  def test_cut_in_resets(self):
    g = governor()
    run(g, 40, -1.0, lead(1.5))
    assert run(g, 1, -1.0, lead(1.3)) == -1.0  # 5 m jump: new car
    assert g.zone == "new lead"

  def test_fast_closing_blocks(self):
    g = governor()
    assert run(g, 40, -1.0, lead(1.3, vrel=-8.0)) == -1.0  # TTC ~5 s
    assert g.zone == "blocked"

  def test_lead_braking_blocks_and_holds(self):
    g = governor()
    run(g, 40, -1.0, lead(1.3))
    assert run(g, 1, -1.0, lead(1.3, a_lead=-2.0)) == -1.0
    assert run(g, 10, -1.0, lead(1.3)) == -1.0  # still held off
    assert run(g, 20, -1.0, lead(1.3)) == g.settings.coast_accel

  def test_only_for_lead_source(self):
    g = governor()
    assert run(g, 40, -1.0, lead(1.3), source_is_lead=False) == -1.0

  def test_no_lead_or_slow(self):
    g = governor()
    assert run(g, 40, -1.0, lead(1.3, prob=0.1)) == -1.0
    assert run(g, 40, -1.0, lead(1.3, v_ego=3.0), v_ego=3.0) == -1.0


class TestSoftPlan:
  def test_factor(self):
    g = GlideGovernor(live=False, settings=PersonalizedSettings(glide=False, soft_plan=True))
    for _ in range(40):
      g.observe(26.8, lead(1.3), T_FOLLOW)
    assert g.distance_cost_factor(True) == g.settings.soft_distance_factor
    assert g.distance_cost_factor(False) == 1.0
    assert g.apply(-1.0, True, ACCEL_MIN) == -1.0  # glide itself off
    for _ in range(5):
      g.observe(26.8, lead(0.8), T_FOLLOW)
    assert g.distance_cost_factor(True) == 1.0

  def test_mid_drive_only_switch_off(self, tmp_path):
    p = str(tmp_path / "camry_long.json")
    write_settings(PersonalizedSettings(personalized=True, glide=True, soft_plan=True), p)
    g = GlideGovernor(path=p)
    write_settings(PersonalizedSettings(personalized=True, glide=True, soft_plan=False, light_brake=-1.2), p)
    import os
    os.utime(p, ns=(1, 1))
    g.refresh_settings(now=1e9)
    assert g.settings.glide and not g.settings.soft_plan and g.settings.light_brake == -0.8 and g.pending_change


class TestApplyRevert:
  def test_apply_limits_steps(self, tmp_path):
    cur = PersonalizedSettings(personalized=True)
    sug = {"changes": [{"field": "follow_s", "bp_mph": [35, 40, 60], "to": [0.5, 1.3, 1.5]},
                       {"field": "light_brake", "to": -1.5}, {"field": "coast_accel", "to": 5.0},
                       {"field": "min_ttc_s", "to": "x"}, {"field": "personalized", "to": False}]}
    new = cp.apply_suggested(sug, cur)
    assert new.follow_bp_mph == (35.0, 40.0, 60.0)
    assert new.follow_s == (1.0, 1.2, 1.3)            # at most 0.1 per point from the default table
    assert new.light_brake == -1.0 and new.coast_accel == -0.1 and new.min_ttc_s == 6.0 and new.personalized

  def test_apply_ignores_mismatched_breakpoints(self):
    cur = PersonalizedSettings(personalized=True, follow_bp_mph=(40.0,), follow_s=(1.0,))
    new = cp.apply_suggested({"changes": [{"field": "follow_s", "bp_mph": [35, 40], "to": [0.9, 0.9]}]}, cur)
    assert new.follow_s == (1.0,)

  def test_history_and_revert(self, tmp_path):
    p, h = str(tmp_path / "camry_long.json"), str(tmp_path / "hist")
    a = PersonalizedSettings(personalized=True, glide=True)
    write_settings(a, p)
    for i in range(7):
      cp.save_with_history(PersonalizedSettings(personalized=True, glide=True, light_brake=-0.5 - 0.1 * i), p, h)
      import time
      time.sleep(0.001)
    assert cp.history_count(h) == cp.HISTORY_KEEP
    before = read_settings(p)
    restored = cp.revert(p, h)
    assert restored is not None and restored.light_brake == round(before.light_brake + 0.1, 1)
    assert read_settings(p) == restored and cp.history_count(h) == cp.HISTORY_KEEP - 1
    for _ in range(10):
      cp.revert(p, h)
    assert cp.revert(p, h) is None


class TestSaveKeepsLiveFile:
  def test_live_file_is_copied_not_moved(self, tmp_path):
    p, h = str(tmp_path / "camry_long.json"), str(tmp_path / "hist")
    write_settings(PersonalizedSettings(light_brake=-0.5), p)
    cp.save_with_history(PersonalizedSettings(light_brake=-0.7), p, h)
    assert read_settings(p).light_brake == -0.7 and cp.history_count(h) == 1
    assert read_settings(str(tmp_path / "hist" / sorted((tmp_path / "hist").iterdir())[0].name)).light_brake == -0.5


class TestSafeGlide:
  def test_passes_through_and_disables_after_error(self):
    inner = governor()
    g = cp.SafeGlide(inner)
    assert run(g, 40, -1.0, lead(1.3)) == inner.settings.coast_accel and g.active   # works like the governor
    inner.apply = lambda *a, **k: 1 / 0
    assert g.apply(-1.0, True, ACCEL_MIN) == -1.0 and g.failed
    assert g.distance_cost_factor(True) == 1.0
    assert run(g, 5, -1.0, lead(1.3)) == -1.0   # stays off for the drive, even though the governor is fine again
    assert g.zone == "off"

  def test_output_bounds(self):
    inner = governor()
    g = cp.SafeGlide(inner)
    run(g, 40, -1.0, lead(1.3))
    inner.apply = lambda a, *_: a - 1.0          # a buggy governor asking for more braking
    assert g.apply(-1.0, True, ACCEL_MIN) == -1.0
    inner.apply = lambda a, *_: 0.5              # or for throttle
    assert g.apply(-1.0, True, ACCEL_MIN) == -1.0
    inner.apply = lambda a, *_: float("nan")
    assert g.apply(-1.0, True, ACCEL_MIN) == -1.0
    inner.distance_cost_factor = lambda *_: 0.0
    assert g.distance_cost_factor(True) == 1.0
    assert not g.failed


class TestSuggestionGate:
  """suggest_long.gate_in_groups: after the whole candidate fails the bench, offer the group that passes on its own."""
  CHANGES = [{"field": "follow_s", "bp_mph": [35, 40, 60], "to": [0.8, 0.95, 1.45], "from": [0.8, 1.0, 1.45]},
             {"field": "coast_margin_s", "from": 0.2, "to": 0.1},
             {"field": "min_ttc_s", "from": 6.0, "to": 7.0}]
  FULL = {"verdict": "fail", "worse": ["personalized, lead brakes 5.0 from 45: FCW 256 > 255"]}
  CUR = PersonalizedSettings(follow_bp_mph=(35.0, 40.0, 60.0), follow_s=(0.8, 1.0, 1.45))

  @staticmethod
  def _sl():
    from openpilot.tools.camry import suggest_long
    return suggest_long

  def test_glide_group_offered_gap_table_dropped(self):
    sl = self._sl()
    calls = []

    def fake_gate(cur, cand, all_modes):
      calls.append((cand.follow_s, cand.coast_margin_s, cand.min_ttc_s, all_modes))
      return {"verdict": "pass", "totals": {}} if cand.follow_s == cur.follow_s else self.FULL

    offered, dropped, gate, gates = sl.gate_in_groups(self.CUR, self.CHANGES, self.FULL, fake_gate)
    assert [c["field"] for c in offered] == ["coast_margin_s", "min_ttc_s"] and gate["verdict"] == "pass"
    assert [c["field"] for c in dropped] == ["follow_s"] and "FCW 256 > 255" in dropped[0]["dropped"]
    assert len(calls) == 2 and calls[0][3] is True and calls[1][3] is False   # glide group in all modes, table alone in one
    assert calls[0][0] == (0.8, 1.0, 1.45) and calls[0][1] == 0.1 and calls[1][0] == (0.8, 0.95, 1.45) and calls[1][1] == 0.2

  def test_never_offers_both_groups(self):
    sl = self._sl()
    offered, dropped, gate, _ = sl.gate_in_groups(self.CUR, self.CHANGES, self.FULL, lambda *a: {"verdict": "pass"})
    assert [c["field"] for c in offered] == ["coast_margin_s", "min_ttc_s"]
    assert [c["field"] for c in dropped] == ["follow_s"] and "alone" in dropped[0]["dropped"]

  def test_single_kind_is_not_rebenched(self):
    sl = self._sl()
    offered, dropped, gate, gates = sl.gate_in_groups(self.CUR, self.CHANGES[1:], self.FULL, lambda *a: pytest.fail("no rerun"))
    assert offered == [] and gate is self.FULL and gates == {} and len(dropped) == 2 and "together" in dropped[0]["dropped"]

  def test_all_fail(self):
    sl = self._sl()
    offered, dropped, gate, _ = sl.gate_in_groups(self.CUR, self.CHANGES, self.FULL, lambda *a: {"verdict": "fail", "worse": ["x: crash"]})
    assert offered == [] and gate is self.FULL and [c["field"] for c in dropped] == ["coast_margin_s", "min_ttc_s", "follow_s"]
    assert all(c["dropped"] == "the bench showed: x: crash" for c in dropped)


class TestFollowFloor:
  """suggest_long.suggest_follow: the learner moves the 1.1 s town default, but never suggests closer than 1.0 s."""
  GAS = {"gas_overrides": [{"mph": 25, "gap_s": 0.9, "vrel": 0.0, "same_lead": True}] * 6}

  def test_default_table(self):
    assert cp.DEFAULT_FOLLOW_BP_MPH == (35.0, 40.0, 60.0) and cp.DEFAULT_FOLLOW_S == (1.1, 1.1, 1.2)

  def test_gas_presses_stop_at_the_floor(self):
    from openpilot.tools.camry import suggest_long as sl
    ch = sl.suggest_follow(self.GAS, PersonalizedSettings())
    assert ch["to"] == [1.0, 1.1, 1.2] and ch["from"] == [1.1, 1.1, 1.2]
    assert sl.suggest_follow(self.GAS, PersonalizedSettings(follow_bp_mph=(35.0, 40.0, 60.0), follow_s=(1.0, 1.1, 1.2))) is None

  def test_below_the_floor_is_raised(self):
    from openpilot.tools.camry import suggest_long as sl
    ch = sl.suggest_follow({}, PersonalizedSettings(follow_bp_mph=(35.0, 40.0, 60.0), follow_s=(0.8, 1.0, 1.45)))
    assert ch["to"] == [1.0, 1.0, 1.45] and "floor" in ch["why"]


class TestOverspeed:
  """Glide over set speed: coast within the allowance, linearly firmer past it, capped; only when speed is the reason."""
  def gov(self, mph=3.0, glide=True):
    return GlideGovernor(live=False, settings=PersonalizedSettings(glide=glide, overspeed_glide_mph=mph))

  def test_default_on_and_clamped(self):
    assert PersonalizedSettings().overspeed_glide_mph == 3.0
    assert parse_settings({"overspeed_glide_mph": 50}).overspeed_glide_mph == 10.0
    assert parse_settings({"overspeed_glide_mph": -1}).overspeed_glide_mph == 0.0

  def test_coasts_within_allowance(self):
    g, v_t = self.gov(), 20.0
    assert g.apply_overspeed(-0.8, v_t + 2 * cp.MPH, v_t, -0.3, True) == -0.3   # flat: the car's own coast
    assert g.zone == "overspeed" and g.active
    assert g.apply_overspeed(-0.8, v_t + 2 * cp.MPH, v_t, 0.2, True) == 0.0    # downhill: hold speed, no throttle
    assert g.apply_overspeed(-0.8, v_t + 2 * cp.MPH, v_t, -0.9, True) == -0.5  # uphill: never below the coast minimum

  def test_linear_ramp_then_capped(self):
    g, v_t = self.gov(), 20.0
    assert g.apply_overspeed(-3.0, v_t + 5.5 * cp.MPH, v_t, -0.3, True) == pytest.approx(-0.65, abs=0.01)  # halfway up the ramp
    assert g.apply_overspeed(-3.0, v_t + 8 * cp.MPH, v_t, -0.3, True) == pytest.approx(cp.OVERSPEED_BRAKE)
    assert g.apply_overspeed(-3.0, v_t + 20 * cp.MPH, v_t, -0.3, True) == pytest.approx(cp.OVERSPEED_BRAKE)  # never harder for speed alone
    outs = [g.apply_overspeed(-3.0, v_t + x * cp.MPH, v_t, -0.3, True) for x in np.arange(0.5, 12.0, 0.25)]
    steps = np.diff(outs)
    assert steps.max() <= 0.0 and -steps.min() < 0.05   # firmer the further over, in small steps, never a jump

  def test_never_adds_throttle_or_acts_under_target(self):
    g = self.gov()
    assert g.apply_overspeed(0.3, 22.0, 20.0, -0.3, True) == 0.3
    assert g.apply_overspeed(-0.1, 22.0, 20.0, -0.3, True) == -0.1
    assert g.apply_overspeed(-0.8, 19.0, 20.0, -0.3, True) == -0.8   # under the target: speed is not why it brakes

  def test_only_when_speed_is_the_reason(self):
    g = self.gov()
    assert g.apply_overspeed(-0.8, 22.0, 20.0, -0.3, False) == -0.8               # a lead, curve, stop or e2e is limiting
    assert self.gov(mph=0.0).apply_overspeed(-0.8, 22.0, 20.0, -0.3, True) == -0.8   # setting off
    assert self.gov(glide=False).apply_overspeed(-0.8, 22.0, 20.0, -0.3, True) == -0.8   # Glide toggle off
    assert g.apply_overspeed(-0.8, 4.0, 3.0, -0.3, True) == -0.8                   # too slow

  def test_blocked_after_fast_closing_lead_then_recovers(self):
    g = self.gov()
    run(g, 40, -1.0, lead(1.3, vrel=-8.0))   # TTC gate trips
    assert g.apply_overspeed(-0.8, 22.0, 20.0, -0.3, True) == -0.8
    for _ in range(25):   # that lead is gone: the hold runs out within about a second
      g.observe(21.0, lead(1.3, prob=0.1), T_FOLLOW)
    assert g.apply_overspeed(-0.8, 21.0, 20.0, -0.3, True) == -0.3   # 2.2 mph over: within the allowance

  def test_safe_wrapper_bounds_and_disables(self):
    class Bad(GlideGovernor):
      def apply_overspeed(self, *a):
        return 5.0
    sg = cp.SafeGlide(Bad(live=False, settings=PersonalizedSettings()))
    assert sg.apply_overspeed(-0.8, 22.0, 20.0, -0.3, True) == -0.8 and not sg.failed

    class Boom(GlideGovernor):
      def apply_overspeed(self, *a):
        raise RuntimeError("x")
    sg = cp.SafeGlide(Boom(live=False, settings=PersonalizedSettings()))
    assert sg.apply_overspeed(-0.8, 22.0, 20.0, -0.3, True) == -0.8 and sg.failed


class TestApproach:
  """Approach taper: ease a positive acceleration request off while closing on a slower lead, never below zero."""
  V = 26.8

  def gap_for(self, room, closing, t_follow=T_FOLLOW):
    """Distance to the lead that leaves `room` m before the target gap at the lead's speed."""
    return t_follow * (self.V - closing) + 6.0 + room

  def test_no_cap_far_or_not_closing(self):
    s = PersonalizedSettings()
    assert cp.approach_cap(s, 150.0, 0.0, self.V, T_FOLLOW) is None       # same speed
    assert cp.approach_cap(s, 150.0, -2.0, self.V, T_FOLLOW) is None      # lead pulling away
    assert cp.approach_cap(s, 300.0, 3.0, self.V, T_FOLLOW) is None       # far back, closing slowly
    assert cp.approach_cap(replace(s, approach=False), 60.0, 8.0, self.V, T_FOLLOW) is None
    assert cp.approach_cap(s, self.gap_for(-5.0, 3.0), 3.0, self.V, T_FOLLOW) is None   # inside the gap: glide's job

  def test_taper_to_zero(self):
    s = PersonalizedSettings()   # approach_decel 0.5; 100 m of room: needed decel = closing^2 / 200
    assert cp.approach_cap(s, self.gap_for(100.0, 10.0), 10.0, self.V, T_FOLLOW) == pytest.approx(0.0)    # needs 0.5: pedal up
    assert cp.approach_cap(s, self.gap_for(100.0, 5.0), 5.0, self.V, T_FOLLOW) is None                    # needs 0.125: no cap yet
    assert cp.approach_cap(s, self.gap_for(100.0, 7.9), 7.9, self.V, T_FOLLOW) == pytest.approx(0.75, abs=0.02)   # halfway
    assert cp.approach_cap(replace(s, approach_decel=1.0), self.gap_for(100.0, 10.0), 10.0, self.V, T_FOLLOW) == pytest.approx(1.0, abs=0.02)
    caps = [cp.approach_cap(s, self.gap_for(100.0, c), c, self.V, T_FOLLOW) for c in np.arange(5.1, 10.0, 0.1)]
    assert all(b <= a + 1e-9 for a, b in zip(caps, caps[1:], strict=False)) and min(caps) >= 0.0   # firmer the faster, never below 0

  def test_governor_caps_positive_requests_only(self):
    g = governor()
    ld = lead(3.0, vrel=-9.0)   # 86 m back, closing 9 m/s: arriving at the gap would need about 0.7 m/s2 -> pedal up
    assert run(g, 40, 0.8, ld, source_is_lead=False) == pytest.approx(0.0)
    assert g.zone == "approach" and g.active
    assert run(g, 1, -1.5, ld, source_is_lead=False) == -1.5          # braking passes through
    assert run(g, 1, -0.2, ld, source_is_lead=False) == -0.2          # so does any request at or below zero
    assert not g.active
    ld2 = lead(2.0, vrel=-12.0)                                        # TTC 5 s: the glide gates block, the cap still applies
    assert run(g, 40, 0.8, ld2, source_is_lead=False) == pytest.approx(0.0)
    assert g.zone == "approach"
    assert run(governor(), 5, 0.8, ld) == 0.8                          # lead not yet stable: no cap

  def test_filter_smooths_a_noisy_closing_speed(self):
    g = governor()
    run(g, 100, 0.8, lead(3.0, vrel=-6.0), source_is_lead=False)      # settled mid-taper (cap about 0.55)
    outs = []
    for i in range(40):                                                # estimate jumping 4.5 <-> 7.5 m/s: raw caps 1.1 <-> 0
      outs.append(run(g, 1, 0.8, lead(3.0, vrel=-7.5 if i % 2 else -4.5), source_is_lead=False))
    assert max(np.abs(np.diff(outs))) < 0.1 and 0.3 < np.mean(outs) < 0.8   # no frame-to-frame flicker
    assert run(g, 100, 0.8, lead(3.0, vrel=-0.2), source_is_lead=False) == 0.8  # released once the lead stops closing

  def test_mid_drive_switch_off(self, tmp_path):
    path = str(tmp_path / "s.json")
    write_settings(PersonalizedSettings(), path)
    g = GlideGovernor(path=path, dt=0.05)
    write_settings(PersonalizedSettings(approach=False), path)
    g.refresh_settings(now=5.0)
    assert not g.settings.approach and g.settings.glide

  def test_safe_wrapper_allows_the_cap_but_not_braking(self):
    class Cap(GlideGovernor):
      def apply(self, a, *_):
        return 0.2
    assert cp.SafeGlide(Cap(live=False, settings=PersonalizedSettings())).apply(0.8, False, ACCEL_MIN) == 0.2

    class Brake(GlideGovernor):
      def apply(self, a, *_):
        return -0.3
    assert cp.SafeGlide(Brake(live=False, settings=PersonalizedSettings())).apply(0.8, False, ACCEL_MIN) == 0.8    # below zero: rejected
    assert cp.SafeGlide(Brake(live=False, settings=PersonalizedSettings())).apply(-0.2, False, ACCEL_MIN) == -0.2  # more braking: rejected


class TestSuggestionRanges:
  """suggest_long.suggest_glide never offers a value outside the settings' valid range (the 2026-10-26 run offered a
  0.04 s coast margin, under the 0.05 s minimum)."""

  @staticmethod
  def _coasts(under_s, decel, end="coast", n=5):
    # Personalized table at 50 mph is 0.8/1.0/1.45 interpolated: about 1.225 s real, so the planner's t_follow (what the
    # coasts' gap_s is measured against) is that minus 6 m / speed
    tf = cp.t_follow_for_gap(1.225, 50 * cp.MPH)
    return [{"mph": 50.0, "gap_s": tf - under_s, "end_gap_s": tf - under_s - 0.02, "decel": decel, "end": end}
            for _ in range(n)]

  def test_values_stay_in_limits(self):
    from openpilot.tools.camry import suggest_long as sl
    cur = PersonalizedSettings(follow_bp_mph=(35.0, 40.0, 60.0), follow_s=(0.8, 1.0, 1.45))
    out = {c["field"]: c["to"] for c in sl.suggest_glide({"coasts": self._coasts(0.04, -0.05, end="brake")}, cur)}
    for field, v in out.items():
      lo, hi = cp.LIMITS[field]
      assert lo <= v <= hi, (field, v)
    assert out["coast_margin_s"] == cp.LIMITS["coast_margin_s"][0]
    assert out["light_width_s"] == cp.LIMITS["light_width_s"][0]
    assert out["coast_accel"] == cp.LIMITS["coast_accel"][1]   # a near-zero coast decel is offered as the weakest allowed


class TestRealGap:
  """The Personalized table is the real gap (distance / speed, the MPC's 6 m stop distance included); the planner's
  t_follow is the gap minus 6 m / speed, never under MIN_T_FOLLOW (the driver, 2026-10-29: "I want to set the real gap")."""

  def test_steady_distance_is_the_real_gap(self):
    for mph, gap in ((45, 1.1), (60, 1.2), (70, 1.2)):
      v = mph * cp.MPH
      t = cp.t_follow_for_gap(gap, v)
      assert t > cp.MIN_T_FOLLOW
      assert (t * v + cp.STOP_DISTANCE) / v == pytest.approx(gap)

  def test_floor_at_low_speed(self):
    v = 30 * cp.MPH
    assert cp.t_follow_for_gap(1.1, v) == cp.MIN_T_FOLLOW
    assert (cp.MIN_T_FOLLOW * v + cp.STOP_DISTANCE) / v > 1.1   # a little farther than the table there, never closer
    assert cp.t_follow_for_gap(1.1, 0.0) == cp.MIN_T_FOLLOW

  def test_default_table_t_follow(self):
    bp, vals = cp.follow_table_ms(PersonalizedSettings())
    got = [cp.t_follow_for_gap(float(np.interp(b, bp, vals)), b) for b in bp]
    assert got == pytest.approx([1.1 - 6 / (35 * cp.MPH), 1.1 - 6 / (40 * cp.MPH), 1.2 - 6 / (60 * cp.MPH)])   # 0.72, 0.76, 0.98

  def test_learner_compares_real_gaps(self):
    from openpilot.tools.camry import suggest_long as sl
    # brake takeovers at 45 mph count as "too close" for the 1.1 s table at 1.15 s real, not at 1.3 s real
    close = {"brake_takeovers": [{"mph": 45, "gap_s": 0.85, "real_s": 1.15, "same_lead": True}] * 2}
    far = {"brake_takeovers": [{"mph": 45, "gap_s": 1.0, "real_s": 1.3, "same_lead": True}] * 2}
    assert sl.suggest_follow(close, PersonalizedSettings())["to"] == [1.1, 1.15, 1.2]
    assert sl.suggest_follow(far, PersonalizedSettings()) is None
    # a profile from before real_s: the real gap is gap_s plus 6 m / speed
    assert sl.real_of({"mph": 45, "gap_s": 0.85}) == pytest.approx(0.85 + 6 / (45 / sl.MPH))

  def test_review_reads_real_gap_commits(self):
    from openpilot.tools.camry import drive_review as dr
    table = ([35 / dr.MPH, 40 / dr.MPH, 60 / dr.MPH], [1.1, 1.1, 1.2])
    v = 60 / dr.MPH
    assert dr.t_follow(0, v, table, real=True, floor=0.7) == pytest.approx(1.2 - 6 / v)
    assert dr.t_follow(0, v, table) == pytest.approx(1.2)   # code from before the real gap: the table was t_follow
    assert dr.t_follow(1, v, table, real=True, floor=0.7) == 1.45   # Standard keeps its stock t_follow


class TestModeName:
  """The distance-button popup ("Driving Personality: ...") names the Aggressive slot like Settings does."""

  def test_names(self, tmp_path):
    on, off = str(tmp_path / "on.json"), str(tmp_path / "off.json")
    write_settings(PersonalizedSettings(personalized=True), on)
    write_settings(PersonalizedSettings(personalized=False), off)
    assert cp.mode_name("aggressive", on) == "Personalized"
    assert cp.mode_name("aggressive", off) == "Aggressive"
    assert cp.mode_name("standard", on) == "Standard"
    assert cp.mode_name("relaxed", on) == "Relaxed"
    assert cp.mode_name("aggressive", str(tmp_path / "missing.json")) == "Personalized"   # no file = the defaults (on)

  def test_broken_file_never_raises(self, tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text("{not json")
    assert cp.mode_name("aggressive", str(bad)) == "Personalized"


@pytest.fixture
def long_mpc_module(monkeypatch):
  """long_mpc for unit tests of its Python side. Where the compiled solver or msgq isn't built (cloud), they are
  stubbed for the import and the stubbed modules are dropped again afterwards."""
  import sys
  import types
  try:
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
    yield long_mpc
    return
  except ImportError:
    pass

  class Stub(types.ModuleType):
    def __getattr__(self, name):
      return object
  before = set(sys.modules)
  stubs = ["cereal.messaging", "openpilot.common.params_pyx",
           "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx"]
  try:
    import casadi  # noqa: F401  (pip-installable: the shaped cost is then checked for real)
  except ImportError:
    stubs.append("casadi")
  for name in stubs:
    monkeypatch.setitem(sys.modules, name, Stub(name))
  import importlib
  yield importlib.import_module("openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc")
  for name in set(sys.modules) - before:   # nothing imported against the stubs outlives the test
    sys.modules.pop(name, None)


class TestBrakeAssist:
  """React sooner when the car ahead brakes hard: the MPC expects the lead's braking to last (aLeadTau capped) and may
  ramp its own braking faster (lower accel-change and jerk weights). Off by default; fails safe to stock."""

  def gov(self, **kw):
    return GlideGovernor(live=False, settings=PersonalizedSettings(**kw))

  def test_on_by_default(self):
    s = PersonalizedSettings()
    assert s.brake_assist and (s.lead_brake_tau, s.brake_ramp) == (0.3, 0.25)   # "firm": passed the 2026-10-29 bench gate
    g = self.gov(brake_assist=False)
    g.observe(26.8, lead(1.2, a_lead=-3.0), 1.0)
    assert not g.assist and g.mpc_ramp_factor() == 1.0 and g.lead_tau_cap() is None   # off: the stock planner

  def test_off_in_relaxed(self):
    from cereal import log
    for relaxed in (log.LongitudinalPersonality.relaxed, "relaxed"):
      g = self.gov()
      g.observe(26.8, lead(1.2, a_lead=-3.0), 1.0, relaxed)
      assert not g.assist and g.lead_tau_cap() is None
    for mode in (log.LongitudinalPersonality.aggressive, log.LongitudinalPersonality.standard, None):
      g = self.gov()
      g.observe(26.8, lead(1.2, a_lead=-3.0), 1.0, mode)
      assert g.assist

  def test_switching_off_mid_drive_applies_right_away(self, tmp_path):
    path = str(tmp_path / "camry_long.json")
    write_settings(PersonalizedSettings(), path)
    g = GlideGovernor(live=True, path=path)
    g.refresh_settings(now=0.0)
    assert g.settings.brake_assist
    write_settings(PersonalizedSettings(brake_assist=False, lead_brake_tau=0.5), path)
    g.refresh_settings(now=100.0)
    assert not g.settings.brake_assist and g.settings.lead_brake_tau == 0.3   # only the switch-off applies mid-drive

  def test_switch_off_seen_when_mtime_unchanged(self, tmp_path):
    # the comma's /tmp keeps coarse mtimes: two quick writes can share one
    path = str(tmp_path / "camry_long.json")
    write_settings(PersonalizedSettings(), path)
    first = os.stat(path).st_mtime_ns
    g = GlideGovernor(live=True, path=path)
    write_settings(PersonalizedSettings(brake_assist=False), path)
    os.utime(path, ns=(first, first))
    g.refresh_settings(now=100.0)
    assert not g.settings.brake_assist

  def test_on_while_lead_brakes_hard_then_holds(self):
    g = self.gov(brake_assist=True)
    g.observe(26.8, lead(1.2, a_lead=-1.0), 1.0)
    assert not g.assist   # ordinary slowing ahead: stock planner
    g.observe(26.8, lead(1.2, a_lead=-2.0), 1.0)
    assert g.assist and g.mpc_ramp_factor() == 0.25 and g.lead_tau_cap() == 0.3
    hold = int(cp.BRAKE_ASSIST_HOLD / g.dt)
    for _ in range(hold - 1):
      g.observe(26.8, lead(1.2, a_lead=0.0), 1.0)
    assert g.assist
    g.observe(26.8, lead(1.2, a_lead=0.0), 1.0)
    assert not g.assist and g.mpc_ramp_factor() == 1.0 and g.lead_tau_cap() is None

  def test_works_with_glide_off_and_needs_a_real_lead(self):
    g = self.gov(brake_assist=True, glide=False, soft_plan=False, approach=False)
    g.observe(26.8, lead(1.2, a_lead=-2.5), 1.0)
    assert g.assist
    g.reset()
    assert not g.assist
    g.observe(26.8, lead(1.2, a_lead=-2.5, prob=0.3), 1.0)
    assert not g.assist

  def test_safe_wrapper(self):
    class Broken:
      def mpc_ramp_factor(self):
        raise RuntimeError
      def lead_tau_cap(self):
        raise RuntimeError
      def reset(self):
        pass
    s = cp.SafeGlide(Broken())
    assert s.mpc_ramp_factor() == 1.0 and s.failed
    assert s.lead_tau_cap() is None
    odd = cp.SafeGlide(SimpleNamespace(mpc_ramp_factor=lambda: 0.01, lead_tau_cap=lambda: 9.0))
    assert odd.mpc_ramp_factor() == 1.0 and odd.lead_tau_cap() is None   # outside LIMITS: stock

  def test_settings_clamped(self):
    s = parse_settings({"brake_assist": True, "lead_brake_tau": 0.0, "brake_ramp": 5})
    assert s.brake_assist and s.lead_brake_tau == cp.LIMITS["lead_brake_tau"][0] and s.brake_ramp == 1.0
    assert parse_settings({"brake_assist": "off"}).brake_assist   # not a bool: the default (on)
    assert not parse_settings({"brake_assist": False}).brake_assist

  def test_mpc_lead_tau_cap(self, long_mpc_module):
    m = long_mpc_module
    mpc = SimpleNamespace(x0=np.array([0.0, 26.8, 0.0]), lead_tau_cap=None, extrapolate_lead=m.LongitudinalMpc.extrapolate_lead)
    ld = SimpleNamespace(status=True, dRel=40.0, vLead=26.8, aLeadK=-3.0, aLeadTau=1.5)
    stock = m.LongitudinalMpc.process_lead(mpc, ld)
    mpc.lead_tau_cap = 0.3
    capped = m.LongitudinalMpc.process_lead(mpc, ld)
    assert capped[-1, 1] < stock[-1, 1] - 3   # expects the lead to keep braking: much slower at the horizon end
    coasting = SimpleNamespace(status=True, dRel=40.0, vLead=26.8, aLeadK=-0.2, aLeadTau=1.5)
    assert np.allclose(m.LongitudinalMpc.process_lead(mpc, coasting), m.LongitudinalMpc.process_lead(
      SimpleNamespace(x0=mpc.x0, lead_tau_cap=None, extrapolate_lead=mpc.extrapolate_lead), coasting))   # not braking: untouched

  def test_mpc_ramp_factor(self, long_mpc_module):
    m = long_mpc_module
    got = {}
    mpc = SimpleNamespace(set_cost_weights=lambda w, c: got.setdefault("w", []).append(list(w)))
    m.LongitudinalMpc.set_weights(mpc, personality=log_personality(), ramp_factor=1.0)
    m.LongitudinalMpc.set_weights(mpc, personality=log_personality(), ramp_factor=0.5)
    stock, fast = got["w"]
    assert fast[4] == pytest.approx(stock[4] * 0.5) and fast[5] == pytest.approx(stock[5] * 0.5)   # accel change, jerk
    assert fast[:4] == stock[:4]


class TestPlannerShaping:
  """The rebuilt planner formula: compiled constants (comfort brake, the shape of the distance cost) as runtime
  parameters. Stock everywhere by default, and sent only to a solver built with them."""

  def test_defaults_are_stock_except_while_assist_acts(self):
    s = PersonalizedSettings()
    assert (s.mpc_comfort_brake, s.mpc_glide_band_s, s.mpc_glide_weight, s.mpc_far_weight) == cp.MPC_SHAPING_STOCK
    assert s.assist_comfort_brake == 2.0
    g = GlideGovernor(live=False, settings=s)
    g.observe(26.8, lead(1.2, a_lead=-0.5), 1.0)
    assert g.mpc_shaping() == cp.MPC_SHAPING_STOCK
    g.observe(26.8, lead(1.2, a_lead=-3.0), 1.0)
    assert g.mpc_shaping() == (2.0, 0.0, 1.0, 1.0)

  def test_assist_comfort_brake_only_while_assist_acts(self):
    g = GlideGovernor(live=False, settings=PersonalizedSettings(assist_comfort_brake=1.5))
    g.observe(26.8, lead(1.2, a_lead=-0.5), 1.0)
    assert g.mpc_shaping() == cp.MPC_SHAPING_STOCK
    g.observe(26.8, lead(1.2, a_lead=-3.0), 1.0)
    assert g.assist and g.mpc_shaping() == (1.5, 0.0, 1.0, 1.0)
    g = GlideGovernor(live=False, settings=PersonalizedSettings(assist_comfort_brake=2.5, mpc_comfort_brake=2.0))
    g.observe(26.8, lead(1.2, a_lead=-3.0), 1.0)
    assert g.mpc_shaping()[0] == 2.0   # never less cautious than without assist

  def test_settings_clamped(self):
    s = parse_settings({"mpc_comfort_brake": 0.5, "mpc_glide_band_s": 2.0, "mpc_glide_weight": 0.0, "mpc_far_weight": 5})
    assert (s.mpc_comfort_brake, s.mpc_glide_band_s, s.mpc_glide_weight, s.mpc_far_weight) == (1.5, 0.5, 0.05, 1.0)

  def test_safe_wrapper(self):
    class Broken:
      def mpc_shaping(self):
        raise RuntimeError
    s = cp.SafeGlide(Broken())
    assert s.mpc_shaping() == cp.MPC_SHAPING_STOCK and s.failed
    odd = cp.SafeGlide(SimpleNamespace(mpc_shaping=lambda: (1.0, 0.0, 1.0, 1.0)))
    assert odd.mpc_shaping() == cp.MPC_SHAPING_STOCK and not odd.failed   # outside LIMITS: stock
    short = cp.SafeGlide(SimpleNamespace(mpc_shaping=lambda: (2.0, 0.2)))
    assert short.mpc_shaping() == cp.MPC_SHAPING_STOCK
    ok = cp.SafeGlide(SimpleNamespace(mpc_shaping=lambda: (2.0, 0.2, 0.3, 0.5)))
    assert ok.mpc_shaping() == (2.0, 0.2, 0.3, 0.5)

  def test_stock_solver_gets_stock_shaping(self, long_mpc_module):
    m = long_mpc_module
    mpc = SimpleNamespace(param_dim=m.STOCK_PARAM_DIM, comfort_brake=1.5, close_band=0.3, band_weight=0.2, far_weight=0.5)
    assert m.LongitudinalMpc.shaping(mpc) == (m.COMFORT_BRAKE, 0.0, 1.0, 1.0)
    mpc.param_dim = m.PARAM_DIM
    assert m.LongitudinalMpc.shaping(mpc) == (1.5, 0.3, 0.2, 0.5)

  def test_cruise_stages_keep_stock_cost(self, long_mpc_module):
    m = long_mpc_module
    x_obstacles = np.array([[50., 80., 60.], [70., 65., 90.], [100., 120., 80.]])   # lead 0, lead 1, cruise closest
    p = m.shaping_params(x_obstacles, 2.0, 0.3, 0.2, 0.5)
    assert p.tolist() == [[2.0, 0.3, 0.2, 0.5], [2.0, 0.3, 0.2, 0.5], [2.0, 0.0, 1.0, 1.0]]
    assert p.shape[1] == m.PARAM_DIM - m.STOCK_PARAM_DIM

  def test_committed_solver_agrees_with_itself(self, long_mpc_module):
    """The json, the C code and the casadi model function of the committed solver give the same np, and the planner
    reads that np (on the comma from the library itself, elsewhere from the json)."""
    import os
    import re
    m = long_mpc_module
    gen = m.EXPORT_DIR
    with open(m.JSON_FILE) as f:
      np_json = json.load(f)["dims"]["np"]
    with open(os.path.join(gen, "acados_solver_long.h")) as f:
      np_h = int(re.search(r"#define LONG_NP\s+(\d+)", f.read()).group(1))
    with open(os.path.join(gen, "acados_solver_long.c")) as f:
      np_c = int(re.search(r"int casadi_np = (\d+);", f.read()).group(1))
    with open(os.path.join(gen, "long_model", "long_expl_ode_fun.c")) as f:
      model = f.read()
    p_in = re.search(r"case 2: return (casadi_s\d+);", model).group(1)
    np_model = int(re.search(p_in + r"\[\d+\] = \{(\d+),", model).group(1))
    assert np_json == np_h == np_c == np_model
    assert np_json in (m.STOCK_PARAM_DIM, m.PARAM_DIM)
    assert m.solver_param_dim() == np_json

  def test_param_dim_fallbacks(self, long_mpc_module, monkeypatch, tmp_path):
    m = long_mpc_module
    monkeypatch.setattr(m, "EXPORT_DIR", str(tmp_path))   # no library there
    monkeypatch.setattr(m, "JSON_FILE", str(tmp_path / "acados_ocp_long.json"))
    assert m.solver_param_dim() == m.STOCK_PARAM_DIM
    (tmp_path / "acados_ocp_long.json").write_text(json.dumps({"dims": {"np": 10}}))
    assert m.solver_param_dim() == 10

  def test_shaped_cost(self, long_mpc_module):
    pytest.importorskip("casadi")
    m = long_mpc_module

    def cost(err, close_band=0.0, band_weight=1.0, far_weight=1.0, v_ego=20.0):
      return float(m.shaped_distance_cost(err, v_ego, close_band, band_weight, far_weight))
    for err in (-2.0, -0.1, 0.0, 0.3):
      assert cost(err) == pytest.approx(err)                                   # stock
      assert cost(err, close_band=0.3) == pytest.approx(err)                   # a band at weight 1 is stock too
    band = 0.3 * 20.0 / 30.0
    assert cost(-0.1, close_band=0.3, band_weight=0.25) == pytest.approx(-0.05)   # inside the band: sqrt(weight)
    assert cost(-band - 1e-9, 0.3, 0.25) == pytest.approx(cost(-band + 1e-9, 0.3, 0.25))   # continuous at its edge
    assert cost(-0.5, 0.3, 0.25) - cost(-0.4, 0.3, 0.25) == pytest.approx(-0.1)   # full slope past the band
    assert cost(0.4, far_weight=0.25) == pytest.approx(0.2)


def log_personality():
  from cereal import log
  return log.LongitudinalPersonality.aggressive
