"""
Car-off CPU setting: keep all 8 cores online while offroad instead of taking the
big cluster (cpu4-7) offline in power save. Off by default.

Stored in a small file rather than a Params key, like fan_mode.py: this branch is a
prebuilt release, so Params keys are compiled into params_pyx.so and unknown keys
are deleted at boot. Anything missing, unreadable or invalid reads back as off.
"""
import os

from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths

# below this low-passed car voltage the big cluster goes back offline, to save the
# car battery. The stock 11.8 V low-voltage shutdown is unchanged.
ALL_CORES_MIN_CAR_VOLTAGE_MV = 12000


def offroad_all_cores_path() -> str:
  if PC:
    return os.path.join(Paths.comma_home(), "offroad_all_cores")
  return "/data/offroad_all_cores"


def read_offroad_all_cores(path: str | None = None) -> bool:
  try:
    with open(path or offroad_all_cores_path()) as f:
      return f.read(16).strip() == "1"
  except Exception:
    return False


def write_offroad_all_cores(enabled: bool, path: str | None = None) -> None:
  path = path or offroad_all_cores_path()
  os.makedirs(os.path.dirname(path), exist_ok=True)
  tmp = f"{path}.tmp"
  with open(tmp, "w") as f:
    f.write("1" if enabled else "0")
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)


def keep_all_cores_offroad(enabled: bool, overheated: bool, in_car: bool, car_voltage_mV: float) -> bool:
  """Whether power save should leave cpu4-7 online. Falls back to stock power save when the
  device is overheated or, in the car, when the car battery voltage is getting low."""
  if not enabled or overheated:
    return False
  return not (in_car and car_voltage_mV < ALL_CORES_MIN_CAR_VOLTAGE_MV)
