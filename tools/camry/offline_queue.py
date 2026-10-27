#!/usr/bin/env python3
"""
Offline job queue for the comma while it is parked and nobody can reach it: runs a fixed list of read-only jobs one
after the other and leaves the results in a folder to collect later. Nothing is applied, installed or changed: the
jobs read the drives on disk and write only their own result files (in OUT) and scratch files (in STAGE, under /tmp).

  cd /data/openpilot && PYTHONPATH=/tmp/combined_e1ad8e8/openpilot nohup /usr/local/venv/bin/python3 /tmp/q/offline_queue.py \
      --tree /tmp/combined_e1ad8e8/openpilot \
      --stage /tmp/q --out /data/media/0/camry_queue_2026-10-27 > /dev/null 2>&1 &

It waits until no other test job runs, then re-runs itself through the test job wrapper from --tree (so the whole
queue is ONE test job: all cores, testing fan curve, and the parked learner waits for it instead of colliding), and
works through the jobs:
  - car on: the running job is stopped at once; the queue waits until the car has been off PARKED_WAIT, then redoes it
  - car battery under MIN_VOLTAGE_MV (in the car): the running job is stopped and the queue ends
  - device over STOP_TEMP_C: the running job is stopped, the queue waits until it is under RESUME_TEMP_C, redoes it once
  - DEADLINE_S after start the queue ends wherever it is
Each job's output goes to OUT/NN_name.txt, and OUT/queue.log has one line per start/finish/stop.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

PARKED_WAIT = 10 * 60     # s the car must be off before a job (re)starts after a drive
MIN_VOLTAGE_MV = 12200
STOP_TEMP_C = 75.0
RESUME_TEMP_C = 60.0
DEADLINE_S = 8 * 3600
BUSY_EXIT = 75            # the wrapper's "another test job is running"
REALDATA = "/data/media/0/realdata"
ONROAD_PARAM = "/data/params/d/IsOnroad"
FIRST_NEW_ROUTE = 0x0c    # the first drive since 2026-10-25


def onroad() -> bool:
  try:
    with open(ONROAD_PARAM, "rb") as f:
      return f.read().strip() == b"1"
  except OSError:
    return False


def routes_since(first: int) -> list[str]:
  out = set()
  for d in os.listdir(REALDATA):
    if "--" in d:
      r = d.rsplit("--", 1)[0]
      try:
        if int(r.split("--")[0], 16) >= first:
          out.add(r)
      except ValueError:
        pass
  return sorted(out)


class Queue:
  def __init__(self, a):
    import cereal.messaging as messaging
    self.a = a
    self.sm = messaging.SubMaster(['deviceState', 'peripheralState', 'pandaStates'])
    self.start = time.monotonic()
    self.off_since = time.monotonic() - PARKED_WAIT   # the car is off (and has been) when the queue starts
    os.makedirs(a.out, exist_ok=True)
    os.makedirs(a.stage, exist_ok=True)

  def log(self, msg: str):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}"
    with open(os.path.join(self.a.out, "queue.log"), "a") as f:
      f.write(line + "\n")

  def state(self):
    """(car_on, low_battery, temp_c)"""
    from cereal import log
    self.sm.update(0)
    car_on = onroad() or bool(self.sm.recv_frame['deviceState'] and self.sm['deviceState'].started)
    ps = self.sm['pandaStates']
    in_car = bool(self.sm.recv_frame['pandaStates'] and len(ps) and ps[0].harnessStatus != log.PandaState.HarnessStatus.notConnected)
    v = int(self.sm['peripheralState'].voltage) if self.sm.recv_frame['peripheralState'] else None
    temp = float(self.sm['deviceState'].maxTempC) if self.sm.recv_frame['deviceState'] else None
    return car_on, in_car and v is not None and 0 < v < MIN_VOLTAGE_MV, temp

  def expired(self) -> bool:
    return time.monotonic() - self.start > DEADLINE_S

  def wait_ready(self) -> str | None:
    """Blocks until a job may start; returns why the queue must end instead, or None."""
    while True:
      if self.expired():
        return "deadline"
      car_on, low, temp = self.state()
      if car_on:
        self.off_since = time.monotonic()
      elif low:
        return "car battery low"
      elif time.monotonic() - self.off_since >= PARKED_WAIT and (temp is None or temp < RESUME_TEMP_C):
        return None
      time.sleep(5 if not car_on else 30)

  def run_job(self, n: int, name: str, cmd: list[str], env: dict) -> str:
    """'done', 'failed rc=N', 'car on', 'hot', 'car battery low' or 'deadline'."""
    path = os.path.join(self.a.out, f"{n:02d}_{name}.txt")
    self.log(f"start {name}: {' '.join(cmd)}")
    t0 = time.monotonic()
    with open(path, "w") as out:
      p = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, env=env, cwd=self.a.tree, start_new_session=True)
      why = None
      while p.poll() is None:
        car_on, low, temp = self.state()
        why = "car on" if car_on else "car battery low" if low else "hot" if temp is not None and temp > STOP_TEMP_C else \
              "deadline" if self.expired() else None
        if why:
          for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
              os.killpg(p.pid, sig)
              p.wait(5)
              break
            except (subprocess.TimeoutExpired, ProcessLookupError):
              continue
          break
        time.sleep(2)
    res = why or ("done" if p.returncode == 0 else f"failed rc={p.returncode}")
    if why == "car on":
      self.off_since = time.monotonic()
    self.log(f"{res} {name} after {int(time.monotonic() - t0)} s")
    return res

  def jobs(self):
    a, py = self.a, sys.executable
    tools = os.path.join(a.tree, "tools", "camry")
    new = routes_since(FIRST_NEW_ROUTE)
    base_env = {k: v for k, v in os.environ.items() if k != "OPENPILOT_PREFIX"}
    env = dict(base_env, PYTHONPATH=a.tree, PYTHONDONTWRITEBYTECODE="1")
    # settings files for the learner pass and the gate: the branch defaults (flat 1.1 s), and PR #9's fallback table
    sys.path.insert(0, a.tree)
    from openpilot.selfdrive.controls.lib import camry_personalized as cp
    from dataclasses import replace
    cur = cp.PersonalizedSettings()
    cur_path, fall_path = os.path.join(a.stage, "settings_defaults.json"), os.path.join(a.stage, "settings_fallback_125.json")
    cp.write_settings(cur, cur_path)
    cp.write_settings(replace(cur, follow_s=(1.1, 1.1, 1.25)), fall_path)
    hist = os.path.join(a.out, "learned_copy.json")
    try:
      with open(os.path.join(os.path.dirname(cp.suggested_path()), "camry_long_learned.json")) as f:
        learned = json.load(f)
    except Exception:
      learned = {}
    with open(hist, "w") as f:
      json.dump(learned, f)
    profile = os.path.join(a.out, "driver_profile_all.json")
    yield "drive_review", [py, os.path.join(a.stage, "drive_review.py"), *new, "--out", os.path.join(a.out, "drive_review.json")], env
    if os.path.exists(os.path.join(a.stage, "scc_check.py")):
      yield "scc_check", [py, os.path.join(a.stage, "scc_check.py"), *new, "--out", os.path.join(a.out, "scc_check.json")], env
    yield "driver_profile_all", [py, os.path.join(tools, "driver_profile.py"), "--out", profile], env
    yield "learner_dry_run", [py, os.path.join(tools, "suggest_long.py"), profile, "--settings", cur_path,
                              "--out", os.path.join(a.out, "suggested_dry_run.json"), "--history", hist], env
    yield "gate_fallback_1p25", [py, os.path.join(tools, "follow_bench.py"), "--gate", cur_path, fall_path, "--modes", "all"], env

  def run(self) -> int:
    self.log(f"queue running as test job pid {os.getpid()}, tree {self.a.tree}")
    for n, (name, cmd, env) in enumerate(self.jobs(), 1):
      for attempt in range(3):
        why = self.wait_ready()
        if why:
          self.log(f"queue ends: {why}")
          return 0
        res = self.run_job(n, name, cmd, env)
        if res in ("car battery low", "deadline"):
          self.log(f"queue ends: {res}")
          return 0
        if res not in ("car on", "hot") or (res == "hot" and attempt >= 1):
          break
    self.log("queue finished")
    return 0


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--tree", required=True, help="overlay root (OUT/openpilot from overlay.py); its test_job.py wraps the queue")
  ap.add_argument("--stage", required=True, help="scratch folder under /tmp with drive_review.py / scc_check.py")
  ap.add_argument("--out", required=True, help="result folder")
  ap.add_argument("--run", action="store_true", help=argparse.SUPPRESS)
  a = ap.parse_args()
  if not os.path.realpath(a.stage).startswith("/tmp/"):
    sys.exit("--stage must be under /tmp")
  if a.run:
    sys.exit(Queue(a).run())
  # stage 1: wait for any other test job (and the car being off), then become one test job through the wrapper
  sys.path.insert(0, os.path.join(a.tree, "system", "hardware"))
  from test_job import test_job_active
  wrapper = os.path.join(a.tree, "system", "hardware", "test_job.py")
  end = time.monotonic() + DEADLINE_S
  os.makedirs(a.out, exist_ok=True)
  while time.monotonic() < end:
    if onroad() or test_job_active():
      time.sleep(30)
      continue
    env = dict({k: v for k, v in os.environ.items() if k != "OPENPILOT_PREFIX"}, PYTHONPATH=a.tree, PYTHONDONTWRITEBYTECODE="1")
    rc = subprocess.call([sys.executable, wrapper, sys.executable, os.path.abspath(__file__), "--run",
                          "--tree", a.tree, "--stage", a.stage, "--out", a.out], env=env, cwd=a.tree)
    if rc != BUSY_EXIT:
      return
    time.sleep(60)


if __name__ == "__main__":
  main()
