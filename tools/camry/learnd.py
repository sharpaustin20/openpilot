#!/usr/bin/env python3
"""
Camry learning daemon: runs the driving-profile learner while the car is parked, and nowhere else.

Started by manager only while offroad (process_config.py). Every 10 s it checks, in order:
  - "Learn After Drives" is on (camry_long.json)                otherwise idle
  - the car has been off for at least PARKED_WAIT                otherwise wait
  - no other test job is running (one comma job at a time)       otherwise wait
  - in the car: battery above MIN_VOLTAGE_MV                      otherwise wait
  - device below START_TEMP_C                                    otherwise wait
  - there are drives it hasn't learned from yet                   otherwise idle
Then it runs driver_profile.py + suggest_long.py through system/hardware/test_job.py (8 cores, testing fan curve)
at the lowest CPU priority, in its own process group. The job is killed at once if the car turns on, if the
device passes STOP_TEMP_C, or when manager stops this process. It writes only /data/camry_long_suggested.json
and /data/camry_long_learned.json; the live settings are never touched here.
"""
import os
import signal
import subprocess
import sys
import time

import cereal.messaging as messaging
from cereal import log
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import camry_personalized as cp
from openpilot.system.hardware.test_job import test_job_active

PARKED_WAIT = 10 * 60      # s after the car is off before learning may start
RETRY_WAIT = 10 * 60       # s after a stopped/failed run
MIN_VOLTAGE_MV = 12200     # car battery, when in the car
START_TEMP_C = 60.0
STOP_TEMP_C = 65.0         # above this the job is stopped so the device can still go onroad (75 C blocks it)
CHECK_PERIOD = 10.0
MAX_RUN_S = 30 * 60
REALDATA = "/data/media/0/realdata"


def routes_on_disk(root=REALDATA):
  routes = set()
  try:
    for d in os.listdir(root):
      if "--" in d:
        r, s = d.rsplit("--", 1)
        if s.isdigit() and any(os.path.exists(os.path.join(root, d, n)) for n in ("rlog.zst", "rlog.bz2", "rlog")):
          routes.add(r)
  except OSError:
    pass
  return routes


def learned_routes(path):
  hist = cp.read_json(path) or {}
  return set(hist.get("routes", {}))


class Learnd:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['deviceState', 'peripheralState', 'pandaStates'])
    self.off_since = time.monotonic()   # manager starts this process only once the car is off
    self.not_before = 0.0
    self.job = None
    self.job_started = 0.0
    self.history = os.path.join(os.path.dirname(cp.suggested_path()), "camry_long_learned.json")
    self.stop = False
    for sig in (signal.SIGINT, signal.SIGTERM):
      signal.signal(sig, self._on_signal)

  def _on_signal(self, *_):
    self.stop = True

  def ignition(self) -> bool:
    if self.params.get_bool("IsOnroad"):
      return True
    return bool(self.sm.recv_frame['deviceState'] and self.sm['deviceState'].started)

  def in_car(self) -> bool:
    ps = self.sm['pandaStates']
    return bool(self.sm.recv_frame['pandaStates'] and len(ps) and ps[0].harnessStatus != log.PandaState.HarnessStatus.notConnected)

  def voltage_mv(self) -> int | None:
    return int(self.sm['peripheralState'].voltage) if self.sm.recv_frame['peripheralState'] else None

  def temp_c(self) -> float | None:
    return float(self.sm['deviceState'].maxTempC) if self.sm.recv_frame['deviceState'] else None

  def kill_job(self, why: str):
    if self.job is None:
      return
    cloudlog.warning(f"camry learnd: stopping job: {why}")
    try:
      os.killpg(self.job.pid, signal.SIGTERM)
      try:
        self.job.wait(5)
      except subprocess.TimeoutExpired:
        os.killpg(self.job.pid, signal.SIGKILL)
        self.job.wait(5)
    except Exception:
      cloudlog.exception("camry learnd: kill")
    self.job = None
    self.not_before = time.monotonic() + RETRY_WAIT

  def start_job(self, routes):
    py = sys.executable
    wrapper = os.path.join(BASEDIR, "system", "hardware", "test_job.py")
    tools = os.path.join(BASEDIR, "tools", "camry")
    profile_out = "/tmp/camry_learn_profile.json"
    script = (f"{py} {tools}/driver_profile.py --root {REALDATA} --out {profile_out} {' '.join(sorted(routes))} && "
              f"{py} {tools}/suggest_long.py {profile_out} --history {self.history}")
    cmd = [py, wrapper, "/bin/sh", "-c", script]
    env = dict(os.environ, PYTHONPATH=BASEDIR)

    def low_priority():
      os.setsid()
      os.nice(19)

    cloudlog.info(f"camry learnd: learning from {len(routes)} new route(s)")
    self.job = subprocess.Popen(cmd, env=env, cwd=BASEDIR, preexec_fn=low_priority,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    self.job_started = time.monotonic()

  def step(self):
    self.sm.update(0)
    now = time.monotonic()
    if self.ignition():
      self.off_since = now
      self.kill_job("car on")
      return
    if self.job is not None:
      rc = self.job.poll()
      temp = self.temp_c()
      if rc is not None:
        self.job = None
        cloudlog.info(f"camry learnd: job finished rc={rc}")
        self.not_before = now + (RETRY_WAIT if rc != 0 else 0)
      elif temp is not None and temp > STOP_TEMP_C:
        self.kill_job(f"device {temp:.0f} C")
      elif now - self.job_started > MAX_RUN_S:
        self.kill_job("took too long")
      return
    if not cp.read_settings().learn or now < self.not_before or now - self.off_since < PARKED_WAIT:
      return
    if test_job_active():
      return   # another test job (e.g. over SSH) is running; one comma job at a time
    v = self.voltage_mv()
    if self.in_car() and (v is None or v < MIN_VOLTAGE_MV):
      return
    temp = self.temp_c()
    if temp is None or temp > START_TEMP_C:
      return
    new = routes_on_disk() - learned_routes(self.history)
    if not new:
      self.not_before = now + 5 * 60
      return
    self.start_job(new)

  def run(self):
    while not self.stop:
      try:
        self.step()
      except Exception:
        cloudlog.exception("camry learnd: step")
      end = time.monotonic() + CHECK_PERIOD
      while not self.stop and time.monotonic() < end:   # short sleeps so a stop from manager is seen at once
        time.sleep(0.5)
    self.kill_job("shutdown")


def main():
  Learnd().run()


if __name__ == "__main__":
  main()
