# Camry SE 2020 branch

Personal sunnypilot branch for one car:

- **Car:** 2020 Toyota Camry SE (4-cyl, TSS-P, limited-speed ACC)
- **Device:** comma 3X
- **Hardware:** OneClone radar filter (detected as smartDSU), with its
  firmware patched so it no longer rewrites ACC_TYPE 2 to 1
- **Base:** upstream sunnypilot `release-tizi`, v2026.002.002
  (`6a17f75`, built from source tag `v2026.002.002` / `c440933`,
  opendbc `380abecc`)

## Changes on top of v2026.002.002

1. **ACC type 2 cruise fix** (`opendbc_repo/opendbc/car/toyota/carstate.py`,
   `opendbc_repo/opendbc/sunnypilot/car/toyota/carstate_ext.py`)
   - With the SMART_DSU flag, sunnypilot always sends ACC_TYPE 1
     (full-speed range). The SE PCM rejects that: it flags a temporary
     cruise fault at every MAIN-on and ignores SET, so gas/brake never
     engage.
   - This branch sends ACC_TYPE 2 for `TOYOTA_CAMRY` with SMART_DSU and
     skips the LOW_SPEED_LOCKOUT == 2 fault for that case.
   - Effect: openpilot longitudinal works from the wheel buttons above
     ~25-28 mph. No stop-and-go (the PCM's low-speed lockout stays).
   - Without it on this car: "Cruise Fault: Restart the Car" (fails safe).

2. **Radar reader for the filter's 0x680 target**
   (`opendbc_repo/opendbc/car/toyota/radar_interface.py`)
   - With the radar filter, the car radar's in-path target shows up on
     bus 1 as 0x680 (10 Hz): distance and side position, no relative
     speed (reverse-engineered, unofficial).
   - Relative speed is estimated from distance with a Kalman filter on
     the lead's own speed, using ego speed from SPEED (0xB4) so ego
     braking is not read as lead braking. A new target is held back
     0.5 s until its speed has settled.
   - radard only uses the track when it matches the camera lead, so the
     camera still decides what the lead is; otherwise it falls back to
     the camera lead as before. Any reader error also falls back to the
     camera instead of crashing card.
   - Only active for `TOYOTA_CAMRY` with the SMART_DSU flag.

3. **Test job mode** (`system/hardware/test_job.py`, `system/hardware/hardwared.py`,
   `system/hardware/tici/hardware.py`, `system/hardware/fan_controller.py`, Settings > Fan)
   - Run a script through the wrapper to mark it as a test job:
     `python3 /data/openpilot/system/hardware/test_job.py python3 my_script.py`
     (over SSH: `ssh comma@<ip> python3 /data/openpilot/system/hardware/test_job.py python3 - < my_script.py`).
   - While it runs with the car off: all 8 cores online, performance
     governor, full clock speed, and the fan never slower than the
     Auto (testing) curve. It switches back as soon as the script exits
     (the marker holds the PID and process start time, so a crashed job
     or a stale marker after reboot is ignored).
   - One test job at a time: while one runs, the wrapper refuses a second
     one (exit code 75) without touching the marker, so the running job
     stays marked. The parked learner retries 10 min later.
   - Car on: no effect, stock onroad CPU config (1689 MHz cap).
   - Once the device is overheated/critical (offroad this includes the
     stock 75 C offroad danger temp after 5 min), clocks drop to the
     1689 MHz cap for the rest of the job and the fan goes to 100%.
   - In the car, off below 12.0 V low-passed battery voltage. The 11.8 V
     low-voltage shutdown, battery-estimate shutdown and max offroad
     time are unchanged.
   - Fan page shows whether a test job is running and which cores are online.

4. **Personalized follow gap: real gaps** (`selfdrive/controls/lib/camry_personalized.py`, `long_mpc.py`)
   - Every gap here is the REAL gap: distance to the car ahead divided by
     speed, bumper to bumper (the driver's choice, 2026-10-29: "I want to
     set the real gap"). The planner itself keeps t_follow × speed + 6 m,
     so `t_follow_for_gap` converts: t_follow = gap - 6 m / speed, never
     under `MIN_T_FOLLOW` 0.7 s (below about 34 mph a 1.1 s real gap
     would need less, so there the gap ends up a little longer, 1.15 s at
     30 mph). Before this change the table was t_follow, so the 6 m came
     on top: PR #8's table felt like 1.2 s in town and 1.6 s at 55 mph.
   - The Personalized profile (item 5) in the Aggressive slot defaults to
     1.1 s up to 40 mph, the driver's choice (2026-10-25): far enough that
     cars don't cut in. It blends to 1.2 s at 60 mph and up (2026-10-28: a
     longer highway gap held back and made them press the gas). In t_follow
     terms that is 0.72 s at 35 mph, 0.76 s at 40 and 0.98 s at 60, closer
     than any table benched before. The learner can move each of the three
     points (35, 40, 60 mph) on its own, comparing the driver's real gaps,
     never closer than 1.0 s real (`tools/camry/suggest_long.py`
     `LEARN_MIN_FOLLOW_S`, also the driver's choice).
   - Stock Aggressive is a flat 1.25 s t_follow (about 1.5 s real at
     60 mph). History, in t_follow: PR #7 used 1.0 s up to 40 mph blending
     to 1.45 s at 60+; PR #8 used 0.8 s up to 35 mph, 1.0 s at 40, 1.45 s
     at 60+. A flat 1.1 s t_follow (1.32 s real at 60 mph) crashed on the
     bench when a lead brakes at 3.5 m/s² from 60 mph to a stop (1.25 s and
     1.45 s stop in time).
   - Standard and Relaxed are unchanged. `tools/camry/drive_review.py`
     reports real gaps and targets. Checked with `tools/camry/follow_bench.py`
     (real planner vs. simulated leads; tunes in `REAL` are converted the
     way the car does).

5. **Personalized profile: glide, soft plan, learning** (`selfdrive/controls/lib/camry_personalized.py`,
   `selfdrive/controls/lib/longitudinal_planner.py`, `long_mpc.py`, Settings > Personalized,
   `tools/camry/learnd.py`, `tools/camry/suggest_long.py`, `tools/camry/driver_profile.py`)
   - A **Personalized** profile replaces Aggressive in the distance button
     (Personalized, Standard, Relaxed): on by default, so the speed-based
     gap table from item 4 is what the Aggressive position drives right
     after the update. Its toggle is the way back to stock Aggressive
     (flat 1.25 s). Standard and Relaxed keep their stock gaps. Settings >
     Toggles > Driving Personality names the first position "Personalized"
     while the profile is on (it follows the wheel's distance button as before).
     The popup after a press of the wheel's distance button says "Driving
     Personality: Personalized" there too (`selfdrive/selfdrived/events.py`,
     `camry_personalized.mode_name`); it said "Aggressive" before.
     On the driving screen the current mode's name sits under the MAX box:
     bright for 4 s after each press of the wheel's distance button and at
     the start of a drive, dim otherwise
     (`selfdrive/ui/sunnypilot/onroad/driving_mode.py`).
   - **Glide** (own toggle, on by default, in every distance mode): behind
     a lead in the same lane, small braking requests are raised to the
     car's coast decel while the gap is a little under the current mode's
     target (coast zone), braking is capped in a light zone below it, and
     the planner's output passes through unchanged below that, when
     closing fast (time to collision under the limit), when the lead brakes
     harder than -1.0 m/s², right after a cut-in, below 11 mph, or for
     anything that isn't the lead. It only ever asks for less braking;
     FCW and stopping are untouched.
   - **Glide over set speed** (Settings > Personalized, default 3 mph, 0 =
     off): over your set speed or the speed limit target with nothing else
     limiting the plan (no lead, curve, stop or end-to-end request), the
     planner's braking is lifted to the car's own coast on that grade
     (never a throttle request) while within the allowance. Past it,
     braking ramps in linearly over 5 mph and is capped at -1.0 m/s², so
     speed alone never brings hard braking. Curve slowdowns, leads, stops
     and FCW are as before; the Glide toggle switches it off with the rest.
     `tools/camry/drive_review.py` (car off, read-only) shows how much of
     a drive's braking was for speed alone and what an allowance covers.
   - **Ease into the gap** (approach taper, own toggle, on by default):
     closing on a slower lead, the acceleration request is tapered off
     once arriving at the lead's speed right at the target gap would need
     more than a quarter of the closing-in decel (default 0.5 m/s², the
     page's "Closing-In Decel"), down to no acceleration at all when it
     would need all of it: the car stops gaining speed early and eases
     into the gap instead of running up and braking. It only ever lowers
     a positive acceleration request, never below zero; braking requests,
     FCW and the glide safety gates are untouched. The closing speed it
     uses is filtered over 1 s (a noisy estimate on this car). Off below
     11 mph; switching it off applies mid-drive like Glide and Soft Plan.
   - **Soft plan** (on by default): while glide may act, the planner's
     weight on holding the exact distance is lowered (a runtime weight, no
     solver rebuild), so the plan itself glides. The too-close penalty is
     unchanged. The learner tunes the strength: one step firmer after
     brake takeovers inside the glide band, one step softer after an hour
     engaged without one (never below 0.2).
   - **Settings** live in `/data/camry_long.json` (not a Params key, as with
     the fan page), read once per drive; only switching Glide, Soft Plan or Ease
     Into The Gap off applies mid-drive. Every value is validated and clamped; the gap
     table can't go below 0.8 s or above 1.75 s, and the glide ranges stay
     inside what the bench has covered: coast decel -0.6 to -0.1 m/s², TTC
     gate 5-10 s, coast margin 0.05-0.5 s, light braking -1.5 to -0.5 m/s²,
     glide over set speed 0-10 mph, closing-in decel 0.3-1.0 m/s² (the page
     offers the same choices). Status for the page in
     `/dev/shm/camry_long_state.json`.
   - **Learning** ("Learn After Drives", on by default): `camry_learnd`
     (manager, offroad only) runs `driver_profile.py` + `suggest_long.py`
     through `test_job.py` at lowest priority once the car has been off
     10 min, the battery is above 12.2 V and the device below 60 °C; it is
     stopped if the car turns on or the device passes 65 °C; a run takes the
     20 newest unlearned drives, so a backlog is worked off in batches. It writes only
     `/data/camry_long_suggested.json` and `camry_long_learned.json`. The
     page shows each suggested change with its evidence (gas presses,
     brake takeovers, your manual gaps, your coasts), and interventions
     per engaged hour. **Apply** moves each value at most one step (0.1 s
     gap, 0.2 m/s²) and keeps the last 5 versions for **Revert**. Nothing
     changes without a tap unless **Apply Automatically** (off by default)
     is on; then the learner applies the same limited step itself after
     each parked run and the page says so. Before a suggestion is offered,
     `suggest_long.py` runs `follow_bench.py --gate` on the candidate
     settings against the current ones (the full scenario set, in the same
     test job; in Standard and Relaxed too when a glide or soft plan value
     changes). The rule is strict: more collision-warning frames in any
     scenario, a crash, braking below -2.5 m/s² or a gap under 0.6 s where
     the current settings had none, or a missed steady gap, in any mode,
     fails the candidate. If the whole set fails, the glide/soft plan
     changes and the gap-table change are benched separately and the
     group that passes is offered; the rest is shown as dropped with the
     bench's reason. Apply, manual or automatic, only accepts a
     suggestion the bench passed. The PCM brake-light check (0x3B7
     `BRAKE_LIGHTS_ACC`) is wired but inactive: `tools/camry/probe_3b7.py`
     found no bit of 0x3B7 that tracks braking on this car (2026-10-24).
   - Bench: `tools/camry/follow_bench.py` runs glide on/off through the
     real planner in all three distance modes (`--table FILE` benches a
     settings file before Apply), including overspeed scenarios (over the
     set speed on flat, downhill and uphill road, a set speed drop, and
     with a slower lead far ahead) and approach scenarios (closing on a
     slower lead from 80-150 m back at 30-60 mph: peak accel, jerk and the
     overshoot under the target gap). `tools/camry/overlay.py` lays a fetched
     branch over the installed tree under /tmp so it can be benched on the
     comma without touching the installed code.

## Updating

This branch is pinned to v2026.002.002. To move to a newer sunnypilot
release, rebase these commits onto the new `release-tizi` and re-test
before installing.
