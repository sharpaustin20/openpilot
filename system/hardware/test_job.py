#!/usr/bin/env python3
"""
Test job mode: while a test script runs with the car off, hardwared puts all 8 cores
online at full speed and uses the Auto (testing) fan curve. It switches back as soon
as the script exits.

Run a script through this wrapper to mark it as a test job:
  python3 /data/openpilot/system/hardware/test_job.py python3 my_script.py
  ssh comma@<ip> python3 /data/openpilot/system/hardware/test_job.py python3 - < my_script.py

The wrapper writes its PID and process start time to the marker file, waits briefly for
the cores to come online, then execs the command (same PID, so stdin/stdout pass through).
hardwared treats the job as running only while that exact process is alive, so a crashed
or killed job, or a stale marker after a reboot, is ignored.

One test job at a time: the wrapper holds a lock (marker path + ".lock") for the life of
the job and exits with BUSY_EXIT, without touching the marker, while another test job is
running. Without it, two jobs started in the same second (an SSH job and the parked
learner) both wrote the marker, and the survivor ran unmarked once the other ended.

Standalone (no openpilot imports) so it runs over SSH without PYTHONPATH.
"""
import os
import sys
import time

CPU_ONLINE_PATH = "/sys/devices/system/cpu/online"
WAIT_FOR_CORES_S = 3.0
BUSY_EXIT = 75   # EX_TEMPFAIL: another test job is running, try again when it has finished


def test_job_path() -> str:
  if os.path.isdir("/data"):
    return "/data/test_job"
  return os.path.join(os.path.expanduser("~"), ".comma", "test_job")


def _proc_start_time(pid: int) -> str | None:
  try:
    with open(f"/proc/{pid}/stat") as f:
      stat = f.read()
    # field 22 (starttime); split after the ")" that ends the command name
    return stat[stat.rindex(")") + 2:].split()[19]
  except Exception:
    return None


def test_job_active(path: str | None = None) -> bool:
  """True while the process recorded in the marker file is still running."""
  try:
    with open(path or test_job_path()) as f:
      pid_s, start = f.read(64).split()
    pid = int(pid_s)
  except Exception:
    return False
  return pid > 0 and _proc_start_time(pid) == start


def mark_test_job(path: str | None = None, pid: int | None = None) -> None:
  pid = pid or os.getpid()
  path = path or test_job_path()
  os.makedirs(os.path.dirname(path), exist_ok=True)
  tmp = f"{path}.tmp"
  with open(tmp, "w") as f:
    f.write(f"{pid} {_proc_start_time(pid)}")
  os.replace(tmp, path)


def _wait_for_cores(timeout: float = WAIT_FOR_CORES_S) -> None:
  # hardwared runs at 2 Hz; wait so the job's os.cpu_count()/worker pools see all 8 cores
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    try:
      with open(CPU_ONLINE_PATH) as f:
        if f.read().strip() == "0-7":
          return
    except Exception:
      return
    time.sleep(0.1)


def _lock(path: str) -> int | None:
  """Exclusive lock for the life of the job (the fd survives exec), or None while another job holds it."""
  import fcntl
  os.makedirs(os.path.dirname(path), exist_ok=True)
  fd = os.open(f"{path}.lock", os.O_RDWR | os.O_CREAT, 0o644)
  try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except OSError:
    os.close(fd)
    return None
  os.set_inheritable(fd, True)
  return fd


def _running_job(path: str) -> str:
  try:
    with open(path) as f:
      return f.read(64).split()[0]
  except Exception:
    return "?"


def main(argv: list[str], path: str | None = None) -> None:
  if not argv:
    print(__doc__, file=sys.stderr)
    sys.exit(2)
  path = path or test_job_path()
  # the lock makes the check and the marker one step; test_job_active() also catches a job started by an older
  # wrapper without the lock
  fd = _lock(path)
  if fd is None or test_job_active(path):
    msg = f"test_job: another test job is running (pid {_running_job(path)}); one comma job at a time, try again when it has finished"
    print(msg, file=sys.stderr)
    sys.exit(BUSY_EXIT)
  mark_test_job(path)
  _wait_for_cores()
  os.execvp(argv[0], argv)


if __name__ == "__main__":
  main(sys.argv[1:])
