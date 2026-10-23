#!/usr/bin/env python3
"""Read-only: why did the comma power off while parked?
Run on the comma with the car off:  python3 parked_shutdown_check.py
Prints settings, then for each recent "shutting device down" the last status
packets before it (car voltage, battery estimate, power draw). Writes nothing."""
import glob, json, os

P = "/data/params/d/"
for k in ("MaxTimeOffroad", "DisablePowerDown", "ForcePowerDown", "CarBatteryCapacity"):
  try:
    print(f"{k}: {open(P + k).read().strip()!r}")
  except OSError:
    print(f"{k}: <unset>")

files = sorted(glob.glob("/data/log/swaglog.*"), key=os.path.getmtime)
recent, hits = [], []
for f in files:
  try:
    lines = open(f, errors="ignore")
  except OSError:
    continue
  for line in lines:
    if "STATUS_PACKET" in line:
      try:
        j = json.loads(line)
        m = j["msg"] if isinstance(j.get("msg"), dict) else json.loads(j["msg"])
        ds, ps = m.get("deviceState", {}).get("deviceState", {}), m.get("peripheralState", {})
        recent.append(f'{j.get("created", 0):.0f} V={ps.get("voltage")}mV cap={ds.get("carBatteryCapacityUwh")}uWh '
                      f'used={ds.get("offroadPowerUsageUwh")}uWh draw={ds.get("powerDrawW")}W started={ds.get("started")}')
        recent = recent[-3:]
      except Exception:
        pass
    elif "shutting device down" in line:
      hits.append((os.path.basename(f), list(recent), line.strip()[:200]))

print(f"\nswaglog files: {len(files)}, shutdowns found: {len(hits)}")
for f, pk, l in hits[-3:]:
  print(f"\n== {f}\n  {l}")
  for p in pk:
    print("   ", p)
