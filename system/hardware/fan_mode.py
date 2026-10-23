"""
Fan mode setting: Auto (stock fan logic), Auto (testing) (a more aggressive curve)
or a manual fan level.

Stored in a small file rather than a Params key: this branch is a prebuilt release,
so Params keys are compiled into params_pyx.so and unknown keys are deleted at boot.
Anything missing, unreadable or invalid reads back as Auto.
"""
import os

from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths

FAN_MODE_AUTO = -1
FAN_MODE_AUTO_TESTING = -2
MANUAL_FAN_LEVELS = (0, 20, 40, 60, 80, 100)
FAN_MODES = (FAN_MODE_AUTO, FAN_MODE_AUTO_TESTING, *MANUAL_FAN_LEVELS)


def fan_mode_path() -> str:
  if PC:
    return os.path.join(Paths.comma_home(), "fan_mode")
  return "/data/fan_mode"


def parse_fan_mode(raw) -> int:
  try:
    value = int(str(raw).strip())
  except (TypeError, ValueError):
    return FAN_MODE_AUTO
  return value if value in FAN_MODES else FAN_MODE_AUTO


def read_fan_mode(path: str | None = None) -> int:
  """Returns one of FAN_MODES: FAN_MODE_AUTO, FAN_MODE_AUTO_TESTING or a manual level."""
  try:
    with open(path or fan_mode_path()) as f:
      return parse_fan_mode(f.read(16))
  except Exception:
    return FAN_MODE_AUTO


def write_fan_mode(mode: int, path: str | None = None) -> None:
  mode = parse_fan_mode(mode)
  path = path or fan_mode_path()
  os.makedirs(os.path.dirname(path), exist_ok=True)
  tmp = f"{path}.tmp"
  with open(tmp, "w") as f:
    f.write(str(mode))
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)
