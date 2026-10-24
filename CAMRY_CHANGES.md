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
   - Car on: no effect, stock onroad CPU config (1689 MHz cap).
   - Once the device is overheated/critical (offroad this includes the
     stock 75 C offroad danger temp after 5 min), clocks drop to the
     1689 MHz cap for the rest of the job and the fan goes to 100%.
   - In the car, off below 12.0 V low-passed battery voltage. The 11.8 V
     low-voltage shutdown, battery-estimate shutdown and max offroad
     time are unchanged.
   - Fan page shows whether a test job is running and which cores are online.

4. **Aggressive follows by speed** (`selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`)
   - Stock Aggressive is a flat 1.25 s gap. On this branch it is 0.8 s up
     to 35 mph, 1.0 s at 40 mph, blending to 1.45 s at 60 mph and above,
     fitted to the driver's own following (`tools/camry/driver_profile.py`).
   - 0.8 s in town is the driver's choice (was 1.0 s in PR #7). The bench
     shows it holds up in normal braking but has less margin than 1.0 s
     when a lead brakes very hard; see PR #8.
   - Standard and Relaxed are unchanged. Checked with
     `tools/camry/follow_bench.py` (real planner vs. simulated leads).

5. **Personalized profile: glide, soft plan, learning** (`selfdrive/controls/lib/camry_personalized.py`,
   `selfdrive/controls/lib/longitudinal_planner.py`, `long_mpc.py`, Settings > Personalized,
   `tools/camry/learnd.py`, `tools/camry/suggest_long.py`, `tools/camry/driver_profile.py`)
   - A **Personalized** profile replaces Aggressive in the distance button
     (Personalized, Standard, Relaxed): on by default, so the speed-based
     gap table from item 4 is what the Aggressive position drives right
     after the update. Its toggle is the way back to stock Aggressive
     (flat 1.25 s). Standard and Relaxed keep their stock gaps.
   - **Glide** (own toggle, on by default, in every distance mode): behind
     a lead in the same lane, small braking requests are raised to the
     car's coast decel while the gap is a little under the current mode's
     target (coast zone), braking is capped in a light zone below it, and
     the planner's output passes through unchanged below that, when
     closing fast (time to collision under the limit), when the lead brakes
     harder than -1.0 m/s², right after a cut-in, below 11 mph, or for
     anything that isn't the lead. It only ever asks for less braking;
     FCW and stopping are untouched.
   - **Soft plan** (off by default): while glide may act, the planner's
     weight on holding the exact distance is lowered (a runtime weight, no
     solver rebuild), so the plan itself glides. The too-close penalty is
     unchanged.
   - **Settings** live in `/data/camry_long.json` (not a Params key, as with
     the fan page), read once per drive; only switching Glide or Soft Plan
     off applies mid-drive. Every value is validated and clamped; the gap
     table can't go below 0.8 s or above 1.75 s. Status for the page in
     `/dev/shm/camry_long_state.json`.
   - **Learning** ("Learn After Drives", on by default): `camry_learnd`
     (manager, offroad only) runs `driver_profile.py` + `suggest_long.py`
     through `test_job.py` at lowest priority once the car has been off
     10 min, the battery is above 12.2 V and the device below 60 °C; it is
     stopped if the car turns on or the device passes 65 °C. It writes only
     `/data/camry_long_suggested.json` and `camry_long_learned.json`. The
     page shows each suggested change with its evidence (gas presses,
     brake takeovers, your manual gaps, your coasts), and interventions
     per engaged hour. **Apply** moves each value at most one step (0.1 s
     gap, 0.2 m/s²) and keeps the last 5 versions for **Revert**. Nothing
     changes without a tap. The PCM brake-light check (0x3B7
     `BRAKE_LIGHTS_ACC`) is wired but inactive: `tools/camry/probe_3b7.py`
     found no bit of 0x3B7 that tracks braking on this car (2026-10-24).
   - Bench: `tools/camry/follow_bench.py` runs glide on/off through the
     real planner in all three distance modes (`--table FILE` benches a
     settings file before Apply).

## Updating

This branch is pinned to v2026.002.002. To move to a newer sunnypilot
release, rebase these commits onto the new `release-tizi` and re-test
before installing.
