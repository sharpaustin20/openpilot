import pytest

from openpilot.system.hardware.fan_controller import FanController

ALL_CONTROLLERS = [FanController]

def patched_controller(mocker, controller_class):
  mocker.patch("os.system", new=mocker.Mock())
  return controller_class(2)

class TestFanController:
  def wind_up(self, controller, ignition=True):
    for _ in range(1000):
      controller.update(100, ignition)

  def wind_down(self, controller, ignition=False):
    for _ in range(1000):
      controller.update(10, ignition)

  @pytest.mark.parametrize("controller_class", ALL_CONTROLLERS)
  def test_hot_onroad(self, mocker, controller_class):
    controller = patched_controller(mocker, controller_class)
    self.wind_up(controller)
    assert controller.update(100, True) >= 70

  @pytest.mark.parametrize("controller_class", ALL_CONTROLLERS)
  def test_offroad_limits(self, mocker, controller_class):
    controller = patched_controller(mocker, controller_class)
    self.wind_up(controller)
    assert controller.update(100, False) <= 30

  @pytest.mark.parametrize("controller_class", ALL_CONTROLLERS)
  def test_no_fan_wear(self, mocker, controller_class):
    controller = patched_controller(mocker, controller_class)
    self.wind_down(controller)
    assert controller.update(10, False) == 0

  @pytest.mark.parametrize("controller_class", ALL_CONTROLLERS)
  def test_limited(self, mocker, controller_class):
    controller = patched_controller(mocker, controller_class)
    self.wind_up(controller, True)
    assert controller.update(100, True) == 100

  @pytest.mark.parametrize("controller_class", ALL_CONTROLLERS)
  def test_windup_speed(self, mocker, controller_class):
    controller = patched_controller(mocker, controller_class)
    self.wind_down(controller, True)
    for _ in range(10):
      controller.update(90, True)
    assert controller.update(90, True) >= 60


class TestManualFan:
  def test_auto_is_stock(self, mocker):
    stock = patched_controller(mocker, FanController)
    auto = patched_controller(mocker, FanController)
    temps = [40, 60, 80, 95, 100, 90, 70, 50]
    for ignition in (True, False):
      for t in temps:
        for _ in range(20):
          assert auto.update(t, ignition, None) == stock.update(t, ignition)

  @pytest.mark.parametrize("level", [0, 20, 40, 60, 80, 100])
  @pytest.mark.parametrize("ignition", [True, False])
  def test_manual_level_when_cool(self, mocker, level, ignition):
    controller = patched_controller(mocker, FanController)
    for _ in range(100):
      out = controller.update(50, ignition, level)
    assert out == level

  def test_manual_overrides_offroad_cap(self, mocker):
    controller = patched_controller(mocker, FanController)
    assert controller.update(50, False, 100) == 100

  @pytest.mark.parametrize("ignition", [True, False])
  def test_hot_never_below_stock(self, mocker, ignition):
    stock = patched_controller(mocker, FanController)
    manual = patched_controller(mocker, FanController)
    for _ in range(1000):
      s = stock.update(100, ignition)
      m = manual.update(100, ignition, 0)
      assert m >= s
    assert manual.update(100, ignition, 0) == (100 if ignition else 30)

  def test_hot_hysteresis(self, mocker):
    from openpilot.system.hardware.fan_controller import MANUAL_HOT_TEMP, MANUAL_HOT_HYST
    controller = patched_controller(mocker, FanController)
    for _ in range(200):
      controller.update(95, True, 20)
    assert controller.update(MANUAL_HOT_TEMP - 1, True, 20) > 20
    for _ in range(5):
      out = controller.update(MANUAL_HOT_TEMP - MANUAL_HOT_HYST - 1, True, 20)
    assert out == 20
