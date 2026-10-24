"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware.fan_controller import MANUAL_ONROAD_MIN
from openpilot.system.hardware.fan_mode import FAN_MODE_AUTO, FAN_MODE_AUTO_TESTING, FAN_MODES, read_fan_mode, write_fan_mode
from openpilot.system.hardware.offroad_cores import read_offroad_all_cores, write_offroad_all_cores
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import ListItemSP
from openpilot.system.ui.sunnypilot.widgets.option_control import OptionControlSP, LABEL_WIDTH
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import ItemAction, text_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
import pyray as rl

# option index -> fan mode: Auto, Auto (testing), then each manual level
FAN_MODE_OPTIONS = list(FAN_MODES)

THERMAL_STATUS_NAMES = {
  "ok": "OK",
  "overheated": "Overheated",
  "critical": "Critical",
}


class FanModeControl(OptionControlSP):
  """OptionControlSP backed by the fan mode file instead of a Params key."""

  def __init__(self):
    ItemAction.__init__(self, enabled=True)
    self.param_key = ""
    self.min_value = 0
    self.max_value = len(FAN_MODE_OPTIONS) - 1
    self.value_change_step = 1
    self._minus_enabled = True
    self._plus_enabled = True
    self.on_value_changed = None
    self.value_map = None
    self.label_width = LABEL_WIDTH
    self.use_float_scaling = False
    self.label_callback = self._label
    self.current_value = 0
    self._font = gui_app.font(FontWeight.MEDIUM)
    self.minus_btn_rect = rl.Rectangle(0, 0, 0, 0)
    self.plus_btn_rect = rl.Rectangle(0, 0, 0, 0)
    self.refresh()

  def refresh(self):
    self.current_value = FAN_MODE_OPTIONS.index(read_fan_mode())

  @staticmethod
  def _label(idx: int) -> str:
    mode = FAN_MODE_OPTIONS[idx]
    if mode == FAN_MODE_AUTO:
      return tr("Auto")
    if mode == FAN_MODE_AUTO_TESTING:
      return tr("Auto (testing)")
    return f"{mode} %"

  def set_value(self, value: int):
    if not (self.min_value <= value <= self.max_value) or value == self.current_value:
      return
    try:
      write_fan_mode(FAN_MODE_OPTIONS[value])
      self.current_value = value
    except Exception:
      cloudlog.exception("fan: failed to save fan mode")


def _fmt_temp(temp: float) -> str:
  return f"{temp:.1f} °C" if temp > 0 else "--"


class FanLayout(Widget):
  def __init__(self):
    super().__init__()
    self._scroller = Scroller(self._initialize_items(), line_separator=True, spacing=0)

  def _initialize_items(self):
    self._mode_control = FanModeControl()
    self._mode = ListItemSP(
      title=lambda: tr("Fan Mode"),
      description=lambda: tr("Auto runs the stock fan logic, without the stock 30% limit while the car is off. " +
                             "Auto (testing) is a more aggressive curve: 0% at 40°C rising to 100% at 90°C, " +
                             "never slower than Auto. A manual level is used onroad and offroad (minimum 30% while " +
                             "driving), but whenever Auto would run the fan faster, the fan follows Auto instead.<br>" +
                             "The fan goes to 100% if the device overheats."),
      action_item=self._mode_control,
      inline=True,
    )

    self._all_cores = toggle_item(
      lambda: tr("Use All CPU Cores (Car Off)"),
      description=lambda: tr("Keep all 8 CPU cores online while the car is off and the screen is asleep, so jobs like " +
                             "log replays run faster. Cores are capped at 1689 MHz, the same as while driving. Uses more " +
                             "power and heat while parked. It falls back to the normal 4 cores if the device overheats or " +
                             "the car battery drops below 12.0 V. The low-voltage and battery shutdowns are unchanged."),
      initial_state=read_offroad_all_cores(),
      callback=self._set_all_cores,
    )

    items = [
      self._mode,
      self._all_cores,
      text_item(lambda: tr("Commanded Fan Speed"), self._commanded_text),
      text_item(lambda: tr("Actual Fan Speed"), self._actual_text),
      text_item(lambda: tr("Thermal Status"), self._thermal_text),
      text_item(lambda: tr("Max Temp (drives fan)"), lambda: self._temp(lambda ds: ds.maxTempC)),
      text_item(lambda: tr("CPU"), lambda: self._temp(lambda ds: max(ds.cpuTempC, default=0.))),
      text_item(lambda: tr("GPU"), lambda: self._temp(lambda ds: max(ds.gpuTempC, default=0.))),
      text_item(lambda: tr("Memory"), lambda: self._temp(lambda ds: ds.memoryTempC)),
      text_item(lambda: tr("PMIC"), lambda: self._temp(lambda ds: max(ds.pmicTempC, default=0.))),
      text_item(lambda: tr("Modem"), lambda: self._temp(lambda ds: max(ds.modemTempC, default=0.))),
    ]
    return items

  def _set_all_cores(self, enabled: bool):
    try:
      write_offroad_all_cores(enabled)
    except Exception:
      cloudlog.exception("fan: failed to save all cores setting")
      self._all_cores.action_item.set_state(read_offroad_all_cores())

  @staticmethod
  def _device_state():
    sm = ui_state.sm
    return sm['deviceState'] if sm.recv_frame['deviceState'] > 0 else None

  def _temp(self, getter) -> str:
    ds = self._device_state()
    return _fmt_temp(getter(ds)) if ds is not None else "--"

  def _commanded_text(self) -> str:
    ds = self._device_state()
    if ds is None:
      return "--"
    commanded = ds.fanSpeedPercentDesired
    mode = FAN_MODE_OPTIONS[self._mode_control.current_value]
    if mode >= 0 and commanded > mode:
      if ui_state.ignition and commanded == MANUAL_ONROAD_MIN:
        return f"{commanded} % " + tr("(driving minimum)")
      return f"{commanded} % " + tr("(following Auto)")
    return f"{commanded} %"

  def _actual_text(self) -> str:
    sm = ui_state.sm
    if sm.recv_frame['peripheralState'] <= 0:
      return "--"
    return f"{sm['peripheralState'].fanSpeedRpm} RPM"

  def _thermal_text(self) -> str:
    ds = self._device_state()
    if ds is None:
      return "--"
    status = str(ds.thermalStatus)
    return tr(THERMAL_STATUS_NAMES.get(status, status))

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._mode_control.refresh()
    self._all_cores.action_item.set_state(read_offroad_all_cores())
    self._scroller.show_event()
