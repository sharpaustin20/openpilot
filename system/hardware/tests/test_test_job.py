import os
import subprocess
import sys

import pytest

from openpilot.system.hardware import test_job as tj
from openpilot.system.hardware.test_job import mark_test_job, test_job_active

HW_DIR = os.path.dirname(os.path.abspath(tj.__file__))


class TestTestJob:
  def test_missing_is_inactive(self, tmp_path):
    assert not test_job_active(str(tmp_path / "nope"))

  def test_invalid_is_inactive(self, tmp_path):
    p = tmp_path / "test_job"
    for raw in ["", "abc", "123", "-1 5", "\x00\x01"]:
      p.write_text(raw)
      assert not test_job_active(str(p))

  def test_running_process_is_active(self, tmp_path):
    p = str(tmp_path / "sub" / "test_job")
    mark_test_job(p)
    assert test_job_active(p)

  def test_exited_process_is_inactive(self, tmp_path):
    p = str(tmp_path / "test_job")
    proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
      mark_test_job(p, proc.pid)
      assert test_job_active(p)
    finally:
      proc.kill()
      proc.wait()
    assert not test_job_active(p)

  def test_reused_pid_is_inactive(self, tmp_path):
    # same PID, different start time (e.g. a stale marker after a reboot)
    p = tmp_path / "test_job"
    p.write_text(f"{os.getpid()} 1")
    assert not test_job_active(str(p))


class TestOneJobAtATime:
  """A second wrapper started while a test job runs exits with BUSY_EXIT and leaves the marker alone."""

  @staticmethod
  def _job(path, secs=30):
    # a real wrapper run in a child: locks, marks, then execs "sleep" (the lock must survive the exec)
    code = f"import sys; sys.path.insert(0, {HW_DIR!r}); import test_job; test_job.main(['sleep', '{secs}'], path={path!r})"
    proc = subprocess.Popen([sys.executable, "-c", code])
    for _ in range(100):
      if test_job_active(path):
        break
      proc.poll()
      assert proc.returncode is None
      __import__("time").sleep(0.05)
    return proc

  def test_second_job_refused_marker_kept(self, tmp_path):
    p = str(tmp_path / "test_job")
    proc = self._job(p)
    try:
      with open(p) as f:
        before = f.read()
      assert before.split()[0] == str(proc.pid) and test_job_active(p)
      assert tj._lock(p) is None                      # held across the exec
      with pytest.raises(SystemExit) as e:
        tj.main(["true"], path=p)
      assert e.value.code == tj.BUSY_EXIT
      with open(p) as f:
        assert f.read() == before                     # the running job is still the marked one
    finally:
      proc.kill()
      proc.wait()
    fd = tj._lock(p)                                  # released when the job ends
    assert fd is not None
    os.close(fd)

  def test_live_marker_from_a_wrapper_without_the_lock(self, tmp_path):
    p = str(tmp_path / "test_job")
    proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
      mark_test_job(p, proc.pid)
      with pytest.raises(SystemExit) as e:
        tj.main(["true"], path=p)
      assert e.value.code == tj.BUSY_EXIT
    finally:
      proc.kill()
      proc.wait()

  def test_stale_marker_does_not_block(self, tmp_path):
    p = tmp_path / "test_job"
    p.write_text(f"{os.getpid()} 1")                  # stale: same PID, other start time
    fd = tj._lock(str(p))
    assert fd is not None and not test_job_active(str(p))
    os.close(fd)
