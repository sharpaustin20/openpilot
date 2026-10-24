#!/usr/bin/env python3
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import RadarData
from opendbc.car.toyota.values import DBC, TSS2_CAR
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

# 2018-20 Camry (TSS-P, no DSU) with a radar CAN filter (smartDSU firmware): the radar's primary in-path
# target appears on bus 1 as 0x680 (10 Hz). Reverse-engineered, unofficial:
#   DISTANCE: Intel start 8, 11 bits, 0.0625 m (from the front of the car)
#   LAT_POS:  Intel start 22, 9 bits signed, 0.04 m (same sign as openpilot yRel, left positive)
#   VALID:    bits 60 and 62 both set
# The radar sends no relative speed, so it is estimated from DISTANCE with a Kalman filter on the lead's absolute
# speed, run once per radar frame on the frame's own timestamp. Ego travel (integrated from SPEED 0xB4, bus 0)
# is a known input, so ego braking does not show up as lead braking. A new target is held back for WARMUP_FRAMES
# so radard never sees an unconverged speed (radard keeps using the camera lead until then).
CAMRY_RADAR_MSG = 0x680
CAMRY_RADAR_BUS = 1
EGO_SPEED_MSG = 0xB4   # SPEED: Motorola start 47, 16 bits, 0.01 km/h
EGO_SPEED_BUS = 0
EGO_TIMEOUT = 0.5      # s without SPEED before ego travel is considered unknown
KF_ACCEL_STD = 3.0     # m/s^2, process noise (lead acceleration)
KF_MEAS_STD = 0.15     # m, distance measurement noise
KF_INIT_V_VAR = 25.0   # (m/s)^2, initial relative speed variance
WARMUP_FRAMES = 5      # radar frames (0.5 s) before a new target is published
NEW_TARGET_JUMP_M = 4.0
NEW_TARGET_LAT_JUMP_M = 1.5


def _create_radar_can_parser(car_fingerprint):
  if car_fingerprint in TSS2_CAR:
    RADAR_A_MSGS = list(range(0x180, 0x190))
    RADAR_B_MSGS = list(range(0x190, 0x1a0))
  else:
    RADAR_A_MSGS = list(range(0x210, 0x220))
    RADAR_B_MSGS = list(range(0x220, 0x230))

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)
  messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n), strict=True))

  return CANParser(DBC[car_fingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.track_id = 0

    if CP.carFingerprint in TSS2_CAR:
      self.RADAR_A_MSGS = list(range(0x180, 0x190))
      self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
    else:
      self.RADAR_A_MSGS = list(range(0x210, 0x220))
      self.RADAR_B_MSGS = list(range(0x220, 0x230))

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)
    self.trigger_msg = self.RADAR_B_MSGS[-1]
    self.updated_messages = set()

    self.camry_filter_radar = CP.carFingerprint == "TOYOTA_CAMRY" and bool(CP_SP.flags & ToyotaFlagsSP.SMART_DSU)
    if self.camry_filter_radar:
      self.rcp = None
      self.kf_x = None   # [distance, lead absolute speed]
      self.kf_p = None   # covariance [[pdd, pdv], [pdv, pvv]]
      self.kf_t = 0.0
      self.kf_y = 0.0
      self.kf_n = 0
      self.last_nanos = 0
      self.ego_v = 0.0
      self.ego_s = 0.0   # integrated ego travel
      self.ego_t = None
      self.kf_ego_s = 0.0

  def _update_camry_filter_radar(self, can_packets):
    # frames arrive as (address, dat, src) tuples from card (can_capnp_to_list); index by position
    for nanos, frames in can_packets:
      self.last_nanos = nanos
      for c in frames:
        if c[0] == EGO_SPEED_MSG and c[2] == EGO_SPEED_BUS and len(c[1]) == 8:
          self._ego_speed_frame(nanos * 1e-9, c[1])
        elif c[0] == CAMRY_RADAR_MSG and c[2] == CAMRY_RADAR_BUS and len(c[1]) == 8:
          self._camry_radar_frame(nanos * 1e-9, bytes(c[1]))
    # publish at the standard 20 Hz cadence; distance is predicted to now between 10 Hz radar frames
    self.frame += 1
    if (self.frame % 5) != 0:
      return None
    if self.kf_x is not None and self.kf_n >= WARMUP_FRAMES and 0 in self.pts:
      pt = self.pts[0]
      v_rel = self.kf_x[1] - self.ego_v
      pt.dRel = float(self.kf_x[0] + v_rel * max(0.0, self.last_nanos * 1e-9 - self.kf_t))
      pt.yRel = float(self.kf_y)
      pt.vRel = float(v_rel)
    ret = RadarData()
    ret.points = list(self.pts.values()) if self.kf_n >= WARMUP_FRAMES else []
    return ret

  def _ego_speed_frame(self, t, dat):
    v = ((dat[5] << 8) | dat[6]) * 0.01 / 3.6
    if self.ego_t is not None and 0.0 < t - self.ego_t < EGO_TIMEOUT:
      self.ego_s += 0.5 * (v + self.ego_v) * (t - self.ego_t)
    self.ego_t, self.ego_v = t, v

  def _camry_radar_frame(self, t, dat):
    raw = int.from_bytes(dat, "little")
    valid = bool((raw >> 60) & 1) and bool((raw >> 62) & 1)
    dist = ((raw >> 8) & 0x7FF) * 0.0625
    lat = (raw >> 22) & 0x1FF
    lat = (lat - 512 if lat >= 256 else lat) * 0.04

    if not valid or dist <= 0.0:
      self.pts.clear()
      self.kf_x = None
      self.kf_n = 0
      return

    if self.ego_t is None or not (0.0 <= t - self.ego_t < EGO_TIMEOUT):
      # no ego speed: don't publish a speed we can't separate from ego motion
      self.pts.clear()
      self.kf_x = None
      self.kf_n = 0
      return

    new_target = self.kf_x is None
    if not new_target:
      dt = t - self.kf_t
      d_pred = self.kf_x[0] + self.kf_x[1] * dt - (self.ego_s - self.kf_ego_s)
      new_target = (not (0.0 < dt < 1.0) or abs(dist - d_pred) > max(NEW_TARGET_JUMP_M, 0.2 * self.kf_x[0])
                    or abs(lat - self.kf_y) > NEW_TARGET_LAT_JUMP_M)

    if new_target:
      self.kf_x = [dist, self.ego_v]
      self.kf_p = [KF_MEAS_STD ** 2, 0.0, KF_INIT_V_VAR]
      self.kf_n = 0
      self.pts.clear()
      pt = RadarData.RadarPoint()
      pt.trackId = self.track_id
      self.track_id += 1
      pt.aRel = float('nan')
      pt.yvRel = float('nan')
      pt.measured = True
      self.pts[0] = pt
    else:
      # predict (constant relative speed, white relative-acceleration noise)
      q = KF_ACCEL_STD ** 2
      pdd, pdv, pvv = self.kf_p
      pdd = pdd + 2 * dt * pdv + dt * dt * pvv + q * dt ** 4 / 4
      pdv = pdv + dt * pvv + q * dt ** 3 / 2
      pvv = pvv + q * dt ** 2
      # update with the measured distance
      s = pdd + KF_MEAS_STD ** 2
      k0, k1 = pdd / s, pdv / s
      res = dist - d_pred
      self.kf_x = [d_pred + k0 * res, self.kf_x[1] + k1 * res]
      self.kf_p = [(1 - k0) * pdd, (1 - k0) * pdv, pvv - k1 * pdv]
      self.kf_n += 1
    self.kf_t = t
    self.kf_ego_s = self.ego_s
    self.kf_y = lat

  def update(self, can_strings):
    if self.camry_filter_radar:
      try:
        return self._update_camry_filter_radar(can_strings)
      except Exception:
        # never take card down over the unofficial radar decode: drop the radar target (radard falls back to the camera lead)
        carlog.exception("camry 0x680 radar reader error")
        self.pts.clear()
        self.kf_x = None
        self.kf_n = 0
        return RadarData()
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0    # reset counter
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)

        score = self.rcp.vl[ii+16]['SCORE']
        # print ii, self.valid_cnt[ii], score, cpt['VALID'], cpt['LONG_DIST'], cpt['LAT_DIST']

        # radar point only valid if it's a valid measurement and score is above 50
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = RadarData.RadarPoint()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
          self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['REL_SPEED']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = bool(cpt['VALID'])
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
