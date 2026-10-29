import numpy as np

from openpilot.tools.camry import flag_review as fr


def seg(t0=100.0, hard=True, lead_gap_m=20.0, v=20.0):
  """20 s of following at v m/s at 20 Hz; with hard, the planner brakes at -2.2 m/s2 5 s before the flag at t0 + 15."""
  ticks = []
  for i in range(int(20 / 0.05)):
    t = t0 + i * 0.05
    a_tgt = -2.2 if hard and 9.5 <= t - t0 <= 10.5 else 0.0
    ticks.append((t, v, a_tgt, a_tgt, a_tgt, True, False, False, True, lead_gap_m, -3.0, 1, 0, 0, 30.0, True))
  return dict(flags=[t0 + 15], buttons=[t0 + 14.98], ticks=ticks, fcw=[t0 + 10.0] if hard else [], alerts=[], clock=None, git=None)


def test_hard_brake_flag():
  s0 = seg()
  B, fcw, alerts = fr.window([s0], s0["flags"][0])
  s = fr.describe(B, s0["flags"][0], fcw, alerts)
  assert s["plan_a_min"] == -2.2 and s["plan_a_min_at"] == -5.5
  assert s["real_gap_at_flag"] == 1.0                       # 20 m at 20 m/s, bumper to bumper
  assert s["closing_max_mph"] == round(3.0 * fr.MPH, 1)
  assert "FCW at -5.0 s" in s["guess"] and "hard braking -2.2" in s["guess"] and "for lead" in s["guess"]


def test_calm_flag():
  s0 = seg(hard=False, lead_gap_m=30.0)
  B, fcw, alerts = fr.window([s0], s0["flags"][0])
  s = fr.describe(B, s0["flags"][0], fcw, alerts)
  assert s["fcw"] == [] and s["real_gap_min"] == 1.5
  assert s["guess"].startswith("nothing stands out")


def test_window_crosses_segments():
  a, b = seg(t0=100.0), seg(t0=120.0)
  B, _, _ = fr.window([a, b], 125.0)
  assert np.isclose(B[0, 0], 105.0) and np.isclose(B[-1, 0], 135.0)
