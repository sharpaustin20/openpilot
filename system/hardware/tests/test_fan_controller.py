import pytest

from openpilot.system.hardware.fan_controller import FanController, MANUAL_ONROAD_MIN
from openpilot.system.hardware.fan_mode import FAN_MODE_AUTO, FAN_MODE_AUTO_TESTING

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
  def test_offroad_no_cap(self, mocker, controller_class):
    controller = patched_controller(mocker, controller_class)
    self.wind_up(controller, False)
    assert controller.update(100, False) >= 90

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
          assert auto.update(t, ignition, FAN_MODE_AUTO) == stock.update(t, ignition)

  @pytest.mark.parametrize("level", [0, 20, 40, 60, 80, 100])
  @pytest.mark.parametrize("ignition", [True, False])
  def test_manual_level_when_cool(self, mocker, level, ignition):
    controller = patched_controller(mocker, FanController)
    for _ in range(100):
      out = controller.update(50, ignition, level)
    # manual levels have a 30% minimum while driving
    assert out == (max(level, MANUAL_ONROAD_MIN) if ignition else level)

  def test_manual_onroad_min(self, mocker):
    assert MANUAL_ONROAD_MIN == 30
    controller = patched_controller(mocker, FanController)
    assert controller.update(20, True, 0) == 30
    assert controller.update(20, False, 0) == 0

  def test_manual_full_offroad(self, mocker):
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
    assert manual.update(100, ignition, 0) >= 90

  @pytest.mark.parametrize("level", [0, 20, 40, 60, 80, 100])
  @pytest.mark.parametrize("ignition", [True, False])
  def test_follows_auto_when_faster(self, mocker, level, ignition):
    stock = patched_controller(mocker, FanController)
    manual = patched_controller(mocker, FanController)
    for t in [40, 60, 70, 80, 85, 90, 95, 85, 70, 50]:
      for _ in range(50):
        floor = max(level, MANUAL_ONROAD_MIN) if ignition else level
        assert manual.update(t, ignition, level) == max(floor, stock.update(t, ignition))


class TestTestingFan:
  @pytest.mark.parametrize("temp,expected", [(30, 0), (40, 0), (65, 50), (80, 80), (90, 100), (100, 100)])
  def test_curve_offroad(self, mocker, temp, expected):
    controller = patched_controller(mocker, FanController)
    assert controller.update(temp, False, FAN_MODE_AUTO_TESTING) == expected

  def test_onroad_floor(self, mocker):
    controller = patched_controller(mocker, FanController)
    assert controller.update(40, True, FAN_MODE_AUTO_TESTING) == 30

  @pytest.mark.parametrize("ignition", [True, False])
  def test_never_below_stock(self, mocker, ignition):
    stock = patched_controller(mocker, FanController)
    testing = patched_controller(mocker, FanController)
    for t in [30, 50, 70, 85, 95, 105, 110, 90, 60]:
      for _ in range(200):
        assert testing.update(t, ignition, FAN_MODE_AUTO_TESTING) >= stock.update(t, ignition)
