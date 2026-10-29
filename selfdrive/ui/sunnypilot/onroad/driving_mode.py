"""
Camry: the distance-button mode by name, under the MAX box. A press of the steering wheel distance button changes the
mode without showing it anywhere else on the screen, so the name ("Personalized" while the Personalized profile owns
the Aggressive slot, else "Aggressive", "Standard", "Relaxed") is shown bright for SHOW_S after each change and at the
start of a drive, and dim the rest of the time.
"""
import time

import pyray as rl

from openpilot.selfdrive.ui.layouts.settings.toggles import AggressiveSlotName
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

SHOW_S = 4.0
FONT_SIZE = 48
PAD_X, PAD_Y = 28, 12


class DrivingModeRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._slot0 = AggressiveSlotName()
    self._personality: int | None = None
    self._changed_at = 0.0

  def update(self):
    sm = ui_state.sm
    if not ui_state.started or sm.recv_frame["selfdriveState"] < ui_state.started_frame:
      self._personality = None
      return
    personality = sm["selfdriveState"].personality.raw
    if personality != self._personality:
      self._personality = personality
      self._changed_at = time.monotonic()

  def name(self) -> str:
    if self._personality == 0:
      return self._slot0()
    return tr("Relaxed") if self._personality == 2 else tr("Standard")

  def _render(self, rect: rl.Rectangle):
    if self._personality is None or not ui_state.has_longitudinal_control:
      return
    fresh = time.monotonic() - self._changed_at < SHOW_S
    text = self.name()
    sz = measure_text_cached(self._font, text, FONT_SIZE)
    box = rl.Rectangle(rect.x + 60, rect.y + 45 + UI_CONFIG.set_speed_height + 24, sz.x + 2 * PAD_X, sz.y + 2 * PAD_Y)
    rl.draw_rectangle_rounded(box, 0.4, 10, rl.Color(0, 0, 0, 200 if fresh else 120))
    if fresh:
      rl.draw_rectangle_rounded_lines_ex(box, 0.4, 10, 4, rl.Color(255, 255, 255, 230))
    rl.draw_text_ex(self._font, text, rl.Vector2(box.x + PAD_X, box.y + PAD_Y), FONT_SIZE, 0,
                    rl.Color(255, 255, 255, 255 if fresh else 170))
