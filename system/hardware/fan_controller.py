#!/usr/bin/env python3
import numpy as np

from openpilot.common.pid import PIDController
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.fan_mode import FAN_MODE_AUTO, FAN_MODE_AUTO_TESTING

# raise fan setpoint on tici/tizi to reduce noise
# after raising LMH threshold in AGNOS 18.1 to prevent CPU throttling
OFFSET = 0 if HARDWARE.get_device_type() == "mici" else 5

# manual mode: at or above the stock setpoint, the stock controller output is a floor
MANUAL_HOT_TEMP = 75 + OFFSET
MANUAL_HOT_HYST = 5.

# Auto (testing): linear 0% at 40C to 100% at 90C, never below the stock output
TESTING_CURVE_TEMPS = [40.0, 90.0]
TESTING_CURVE_SPEEDS = [0, 100]


class FanController:
  def __init__(self, rate: int) -> None:
    self.last_ignition = False
    self.controller = PIDController(k_p=0, k_i=4e-3, rate=rate)
    self.manual_hot = False

  def update(self, cur_temp: float, ignition: bool, fan_mode: int = FAN_MODE_AUTO) -> int:
    # no offroad cap (stock caps offroad at 30%), so the device can cool during car-off testing
    self.controller.pos_limit = 100
    self.controller.neg_limit = 30 if ignition else 0

    if ignition != self.last_ignition:
      self.controller.reset()
    self.last_ignition = ignition

    # stock controller always runs, so it is wound up if another mode needs it
    stock = int(self.controller.update(
                  error=(cur_temp - (75 + OFFSET)),  # temperature setpoint in C
                  feedforward=np.interp(cur_temp, [60.0 + OFFSET, 100.0 + OFFSET], [0, 100])
               ))

    if fan_mode == FAN_MODE_AUTO_TESTING:
      self.manual_hot = False
      return max(stock, int(np.interp(cur_temp, TESTING_CURVE_TEMPS, TESTING_CURVE_SPEEDS)))

    if fan_mode < 0:
      self.manual_hot = False
      return stock

    if cur_temp >= MANUAL_HOT_TEMP:
      self.manual_hot = True
    elif cur_temp < MANUAL_HOT_TEMP - MANUAL_HOT_HYST:
      self.manual_hot = False

    level = int(np.clip(fan_mode, 0, 100))
    return max(level, stock) if self.manual_hot else level
