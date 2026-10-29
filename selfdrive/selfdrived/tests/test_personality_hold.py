from openpilot.selfdrive.selfdrived.personality_hold import HOLD_S, PersonalityHold


class FakeClock:
  def __init__(self):
    self.t = 0.0

  def __call__(self):
    return self.t


class TestPersonalityHold:
  def setup_method(self):
    self.clock = FakeClock()
    self.hold = PersonalityHold(self.clock)

  def test_reads_pass_without_a_press(self):
    assert self.hold.accept(0)
    assert self.hold.accept(2)

  def test_stale_read_ignored_until_write_lands(self):
    self.hold.pressed(2)
    self.clock.t = 0.1
    assert not self.hold.accept(0)
    self.clock.t = 2.7
    assert not self.hold.accept(0)
    assert self.hold.accept(2)
    # once landed, a change from the settings page goes through again
    assert self.hold.accept(1)

  def test_write_that_never_lands_times_out(self):
    self.hold.pressed(2)
    self.clock.t = HOLD_S - 0.1
    assert not self.hold.accept(0)
    self.clock.t = HOLD_S + 0.1
    assert self.hold.accept(0)

  def test_queued_writes_of_earlier_presses_ignored(self):
    self.hold.pressed(2)
    self.clock.t = 0.5
    self.hold.pressed(1)
    self.clock.t = 1.0
    assert not self.hold.accept(2)  # the first press's write lands first
    assert self.hold.accept(1)

  def test_drive_2026_10_29_never_snaps_back(self):
    # route 00000027 seg 3: presses every 0.6-2.2 s from aggressive, each write landing 0.75 s after the one before it
    presses = [224.28, 224.43, 225.16, 227.36, 227.95, 228.64, 229.46, 230.17]
    personality, disk, pending_writes = 0, 0, []
    seen = []
    t = 224.0
    while t < 240.0:
      self.clock.t = t
      if presses and t >= presses[0]:
        presses.pop(0)
        with self.hold.lock:
          personality = (personality - 1) % 3
          self.hold.pressed(personality)
        pending_writes.append(personality)
      if pending_writes and round(t * 100) % 75 == 0:
        disk = pending_writes.pop(0)
      if round(t * 100) % 10 == 0:
        with self.hold.lock:
          if self.hold.accept(disk):
            personality = disk
      seen.append(personality)
      t = round(t + 0.01, 2)
    # 8 presses from aggressive (relaxed, standard, aggressive, ...) end on standard, and nothing moves it without a press
    assert personality == (0 - 8) % 3
    changes = sum(a != b for a, b in zip(seen, seen[1:], strict=False))
    assert changes == 8
