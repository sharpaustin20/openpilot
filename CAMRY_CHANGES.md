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

## Updating

This branch is pinned to v2026.002.002. To move to a newer sunnypilot
release, rebase these commits onto the new `release-tizi` and re-test
before installing.
