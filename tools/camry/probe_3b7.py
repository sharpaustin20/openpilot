# Read-only probe: which bit of 0x3B7 (ESP_CONTROL) is the "ACC is braking" brake-light bit on this Camry, and on
# which bus. Scans rlogs, and for every bus that carries 0x3B7 counts, per bit, how often it is set while
# (a) the driver presses the brake, (b) sunnypilot long is active and asks for < -0.5 m/s2, (c) neither.
# The brake-light bit should be ~100% in (a) and (b) and ~0% in (c); a driver-only bit is high in (a) only.
#
# Mac:    PYTHONPATH=~/openpilot python3 probe_3b7.py --root ~/radar_work/logs [route ...]
# comma:  cd /data/openpilot && PYTHONPATH=/data/openpilot python3 system/hardware/test_job.py \
#           /usr/local/venv/bin/python3 /tmp/probe_3b7.py --root /data/media/0/realdata [route ...]
# Prints ~20 lines; writes nothing.
import argparse, os, sys
from collections import defaultdict
from multiprocessing import Pool

ADDR = 0x3B7


def seg_file(d):
  for name in ("rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
    p = os.path.join(d, name)
    if os.path.exists(p):
      return p
  return None


def scan(path):
  from openpilot.tools.lib.logreader import LogReader
  brake, long_active, req = False, False, 0.0
  # (bus) -> state -> [set-count per bit (64), total]
  stats = defaultdict(lambda: {s: [[0] * 64, 0] for s in ("driver_brake", "op_decel", "neither")})
  addrs_seen = defaultdict(int)
  try:
    for m in LogReader(path):
      w = m.which()
      if w == "carState":
        brake = m.carState.brakePressed
      elif w == "carControl":
        long_active = m.carControl.longActive
        req = m.carControl.actuators.accel
      elif w == "can":
        for c in m.can:
          if c.address != ADDR:
            continue
          addrs_seen[c.src] += 1
          if brake:
            st = "driver_brake"
          elif long_active and req < -0.5:
            st = "op_decel"
          else:
            st = "neither"
          bits, _ = stats[c.src][st]
          stats[c.src][st][1] += 1
          for i, byte in enumerate(c.dat[:8]):
            for b in range(8):
              if byte >> b & 1:
                bits[i * 8 + b] += 1
  except Exception as e:
    return path, None, None, f"{e}"
  return path, {k: dict(v) for k, v in stats.items()}, dict(addrs_seen), ""


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="*")
  ap.add_argument("--root", default="/data/media/0/realdata")
  ap.add_argument("--jobs", type=int, default=os.cpu_count())
  a = ap.parse_args()
  if os.path.exists("/data/params/d/IsOnroad") and open("/data/params/d/IsOnroad", "rb").read().strip() == b"1":
    sys.exit("car is onroad; run this with the car off")
  segs = sorted(os.path.join(a.root, d) for d in os.listdir(a.root)
                if "--" in d and (not a.routes or d.rsplit("--", 1)[0] in a.routes))
  files = [f for f in (seg_file(d) for d in segs) if f]
  print(f"{len(files)} segments, {a.jobs} workers", flush=True)
  tot = defaultdict(lambda: {s: [[0] * 64, 0] for s in ("driver_brake", "op_decel", "neither")})
  seen = defaultdict(int)
  errs = 0
  with Pool(a.jobs) as pool:
    for path, stats, addrs, err in pool.imap_unordered(scan, files):
      if err:
        errs += 1
        continue
      for bus, n in addrs.items():
        seen[bus] += n
      for bus, by_state in stats.items():
        for st, (bits, n) in by_state.items():
          tot[bus][st][1] += n
          for i in range(64):
            tot[bus][st][0][i] += bits[i]
  print(f"0x3B7 frames per bus: {dict(seen) or 'NONE'}; segment errors {errs}")
  for bus in sorted(tot):
    print(f"bus {bus}:")
    for st in ("driver_brake", "op_decel", "neither"):
      bits, n = tot[bus][st]
      print(f"  {st:13s} {n:7d} frames")
    # bits that differ between states: candidates for the brake-light bit
    rows = []
    for i in range(64):
      pct = {st: (100.0 * tot[bus][st][0][i] / tot[bus][st][1]) if tot[bus][st][1] else None for st in tot[bus]}
      vals = [v for v in pct.values() if v is not None]
      if vals and max(vals) - min(vals) > 20.0:
        rows.append((i, pct))
    for i, pct in rows:
      f = lambda v: "   - " if v is None else f"{v:5.1f}"
      print(f"  byte {i // 8} bit {i % 8} (dbc bit {i}): driver_brake {f(pct['driver_brake'])}%  op_decel {f(pct['op_decel'])}%  neither {f(pct['neither'])}%")
    if not rows:
      print("  no bit differs between states by more than 20 points")


if __name__ == "__main__":
  main()
