import pytest

from openpilot.system.hardware.fan_mode import FAN_MODE_AUTO, FAN_MODES, read_fan_mode, write_fan_mode


class TestFanMode:
  def test_missing_is_auto(self, tmp_path):
    assert read_fan_mode(str(tmp_path / "nope")) == FAN_MODE_AUTO

  @pytest.mark.parametrize("raw", ["", "abc", "50", "120", "-3", "-5", "20.5", "\x00\x01"])
  def test_invalid_is_auto(self, tmp_path, raw):
    p = tmp_path / "fan_mode"
    p.write_text(raw)
    assert read_fan_mode(str(p)) == FAN_MODE_AUTO

  @pytest.mark.parametrize("level", FAN_MODES)
  def test_roundtrip(self, tmp_path, level):
    p = str(tmp_path / "sub" / "fan_mode")
    write_fan_mode(level, p)
    assert read_fan_mode(p) == level

  def test_write_invalid_stores_auto(self, tmp_path):
    p = str(tmp_path / "fan_mode")
    write_fan_mode(55, p)
    assert read_fan_mode(p) == FAN_MODE_AUTO
