import json
from types import SimpleNamespace

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
    assert s.glide and s.light_brake == -1.5 and s.min_ttc_s == 3.0 and s.coast_accel == 0.0
    assert s.band == "relative" and s.coast_margin_s == PersonalizedSettings().coast_margin_s
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
    assert json.load(open(p))["glide"] is True


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
    assert new.follow_s == (0.8, 1.1, 1.5)            # 0.8 floor, then at most +0.1 per point from the default table
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
