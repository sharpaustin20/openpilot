"""
Camry: a flag button on the driving screen, bottom right. One tap marks "I didn't like what the car just did".

It sends the same bookmarkButton message as the sidebar's flag (feedbackd turns it into userBookmark, which is logged,
shows "Bookmark Saved" and keeps the segment from being deleted), but without opening the sidebar first. The button
turns green with the drive's flag count for FLASH_S after a tap. Nothing else runs while driving:
tools/camry/flag_review.py reads the flags back with the car off.
"""
import time

import pyray as rl

from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import get_bottom_dev_ui_offset
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

BTN_SIZE = 150
ICON_SIZE = 84
FLASH_S = 2.0
MIN_INTERVAL_S = 2.0   # a second tap this soon after the last is the same flag


class FlagButton(Widget):
  def __init__(self):
    super().__init__()
    self._icon = gui_app.texture("images/button_flag.png", ICON_SIZE, ICON_SIZE)
    self._font = gui_app.font(FontWeight.BOLD)
    self._on_flag = None
    self._last_flag = 0.0
    self._count = 0
    self._started_frame = -1

  def set_flag_callback(self, callback):
    self._on_flag = callback

  def button_rect(self, content_rect: rl.Rectangle) -> rl.Rectangle:
    x = content_rect.x + content_rect.width - UI_BORDER_SIZE - BTN_SIZE
    y = content_rect.y + content_rect.height - UI_BORDER_SIZE - BTN_SIZE - get_bottom_dev_ui_offset()
    return rl.Rectangle(x, y, BTN_SIZE, BTN_SIZE)

  def _update_state(self):
    if ui_state.started_frame != self._started_frame:   # new drive: the count starts over
      self._started_frame = ui_state.started_frame
      self._count = 0
      self._last_flag = 0.0

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    now = time.monotonic()
    if now - self._last_flag < MIN_INTERVAL_S:
      return
    self._last_flag = now
    self._count += 1
    if self._on_flag is not None:
      self._on_flag()

  def _render(self, rect: rl.Rectangle):
    cx, cy = rect.x + rect.width / 2, rect.y + rect.height / 2
    flashing = time.monotonic() - self._last_flag < FLASH_S
    if flashing:
      bg = rl.Color(0x16, 0x7F, 0x40, 230)
    else:
      bg = rl.Color(0, 0, 0, 200 if self.is_pressed else 140)
    rl.draw_circle(int(cx), int(cy), rect.width / 2, bg)
    rl.draw_circle_lines(int(cx), int(cy), rect.width / 2, rl.Color(255, 255, 255, 120))
    if flashing:
      text = str(self._count)
      sz = measure_text_cached(self._font, text, 80)
      rl.draw_text_ex(self._font, text, rl.Vector2(cx - sz.x / 2, cy - sz.y / 2), 80, 0, rl.WHITE)
    else:
      rl.draw_texture_v(self._icon, rl.Vector2(cx - ICON_SIZE / 2, cy - ICON_SIZE / 2), rl.Color(255, 255, 255, 230))
