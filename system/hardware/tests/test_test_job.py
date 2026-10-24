import os
import subprocess
import sys

from openpilot.system.hardware.test_job import mark_test_job, test_job_active


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
