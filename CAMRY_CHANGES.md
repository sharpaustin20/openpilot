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

2. **Use All CPU Cores (Car Off)** (Settings > Fan; `system/hardware/offroad_cores.py`,
   `system/hardware/hardwared.py`, `system/hardware/tici/hardware.py`)
   - Stock power save takes cpu4-7 offline while the car is off and the
     screen is asleep. With this toggle on, all 8 cores stay online
     (ondemand governor, capped at 1689 MHz like onroad) so offline jobs
     such as log replays run faster.
   - Off by default. Stored in `/data/offroad_all_cores` ("1" = on).
   - Falls back to stock power save when the device is overheated, or in
     the car when the low-passed battery voltage is below 12.0 V.
   - The 11.8 V low-voltage shutdown, battery-estimate shutdown, max
     offroad time and thermal logic are unchanged. The battery estimate
     uses measured power draw, so extra draw is counted.
   - Onroad behavior is unchanged.

Not included yet: the 0x680 radar reader (still being tested).

## Updating

This branch is pinned to v2026.002.002. To move to a newer sunnypilot
release, rebase these commits onto the new `release-tizi` and re-test
before installing.
