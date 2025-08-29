#!/usr/bin/env python3
import math

from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import RadarData
from opendbc.car.mazda.values import DBC, MazdaFlags
from opendbc.car.interfaces import RadarInterfaceBase


def _create_radar_can_parser(car_fingerprint):
  if DBC[car_fingerprint]['radar'] is None:
    return None
  
  messages = [(f"RADAR_TRACK_{addr}", 10) for addr in range(361, 367)]
  return CANParser(DBC[car_fingerprint][Bus.radar], messages, 2)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.track_id = 0
    self.updated_messages = set()

    # Use Mazda's single range approach
    self.RADAR_TRACK_MSGS = list(range(361, 367))
    
    # Toyota-style validation tracking
    self.valid_cnt = {key: 0 for key in self.RADAR_TRACK_MSGS}

    # Check for radar availability using Mazda flags
    radar_available = not CP.radarUnavailable and (CP.flags & MazdaFlags.RADAR_INTERCEPTOR)
    self.rcp = None if not radar_available else _create_radar_can_parser(CP.carFingerprint)

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)
    
    # Mazda approach - process immediately without trigger message
    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for addr in self.RADAR_TRACK_MSGS:
      if addr in updated_messages:
        msg = self.rcp.vl[f"RADAR_TRACK_{addr}"]
        
        # Toyota-style validation logic adapted for Mazda message structure
        if msg['DIST_OBJ'] >= 4095 or msg['ANG_OBJ'] >= 2046 or msg['RELV_OBJ'] <= -16:
          self.valid_cnt[addr] = 0  # reset counter for invalid data
        
        # Check if this is valid radar data
        valid_measurement = (msg['DIST_OBJ'] != 4095) and (msg['ANG_OBJ'] != 2046) and (msg['RELV_OBJ'] != -16)
        
        if valid_measurement:
          self.valid_cnt[addr] += 1
        else:
          self.valid_cnt[addr] = max(self.valid_cnt[addr] - 1, 0)

        # Create or update radar point with Toyota-style validation
        if valid_measurement and self.valid_cnt[addr] > 0:
          if addr not in self.pts:
            self.pts[addr] = RadarData.RadarPoint()
            self.pts[addr].trackId = self.track_id
            self.track_id += 1
          
          # Convert Mazda radar data to standard format
          azimuth = math.radians(msg['ANG_OBJ'] / 64)
          self.pts[addr].dRel = msg['DIST_OBJ'] / 16  # from front of car
          self.pts[addr].yRel = -math.sin(azimuth) * msg['DIST_OBJ'] / 16  # in car frame's y axis, left is positive
          self.pts[addr].vRel = msg['RELV_OBJ'] / 16
          self.pts[addr].aRel = float('nan')
          self.pts[addr].yvRel = float('nan')
          self.pts[addr].measured = bool(self.valid_cnt[addr] > 2)  # Only consider measured after 3+ consistent detections
          # Higher value represent more confidence, slower response to objects
        else:
          if addr in self.pts:
            del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret