"""
Camry: keeps a distance-button press from being undone by selfdrived's params thread.

The button handler changes the personality and writes LongitudinalPersonality with a non-blocking put, while
params_thread reads that param back every 0.1 s. When the disk is slow the read still returns the old value: on
2026-10-29 the writes took 0.16-2.7 s to land, so every press snapped back to the old mode within 0.1 s (and the
"Personalized" popup never stayed). After a press, reads of any other value are ignored until the new value reads back
or HOLD_S passes (so a write that never lands can't hold the mode forever).
"""
import threading
import time

HOLD_S = 20.0


class PersonalityHold:
  def __init__(self, clock=time.monotonic):
    # held by the caller around both calls and its own personality assignment
    self.lock = threading.Lock()
    self._clock = clock
    self._pending: tuple[int, float] | None = None

  def pressed(self, personality: int) -> None:
    """The button handler set this personality and queued its param write."""
    self._pending = (personality, self._clock())

  def accept(self, read: int) -> bool:
    """Whether params_thread should use the personality it just read from params."""
    if self._pending is None:
      return True
    value, since = self._pending
    if read == value or self._clock() - since > HOLD_S:
      self._pending = None
      return True
    return False
