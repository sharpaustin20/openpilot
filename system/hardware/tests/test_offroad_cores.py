import pytest

from openpilot.system.hardware.offroad_cores import ALL_CORES_MIN_CAR_VOLTAGE_MV, keep_all_cores_offroad, \
                                                    read_offroad_all_cores, write_offroad_all_cores


class TestOffroadCores:
  def test_missing_is_off(self, tmp_path):
    assert not read_offroad_all_cores(str(tmp_path / "nope"))

  @pytest.mark.parametrize("raw", ["", "0", "2", "true", "\x00\x01"])
  def test_invalid_is_off(self, tmp_path, raw):
    p = tmp_path / "offroad_all_cores"
    p.write_text(raw)
    assert not read_offroad_all_cores(str(p))

  @pytest.mark.parametrize("enabled", [True, False])
  def test_roundtrip(self, tmp_path, enabled):
    p = str(tmp_path / "sub" / "offroad_all_cores")
    write_offroad_all_cores(enabled, p)
    assert read_offroad_all_cores(p) == enabled

  def test_keep_all_cores(self):
    ok = ALL_CORES_MIN_CAR_VOLTAGE_MV
    assert keep_all_cores_offroad(True, False, True, ok)
    assert not keep_all_cores_offroad(False, False, True, ok)
    # overheated falls back to stock power save
    assert not keep_all_cores_offroad(True, True, True, ok)
    assert not keep_all_cores_offroad(True, True, False, ok)
    # low car voltage falls back, but only in the car (USB power indoors reads ~5 V)
    assert not keep_all_cores_offroad(True, False, True, ok - 1)
    assert keep_all_cores_offroad(True, False, False, 5000)
