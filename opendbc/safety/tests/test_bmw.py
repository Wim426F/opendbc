#!/usr/bin/env python3
import numpy as np
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import make_msg

# 0x22E STEER_TORQUE: 8-bit signed, 0.125 Nm/bit. 12 Nm = 96 counts.
BMW_STEER_ADDR = 0x22E
BMW_STEER_STATUS_ADDR = 0x22F
BMW_GAS_ADDR = 0x200
BMW_PEDAL_STATUS_ADDR = 0x199
BMW_WHEEL_SPEEDS_ADDR = 0xCE
BMW_STALK_ADDR = 0x194
BMW_VCU_ABS_ADDR = 0xA8
BMW_TORQUE_CUT_ADDR = 0x201
BMW_SPEED_POISON_ADDR = 0x1A0
BMW_DSC_ADDR = 0x19E

BMW_STEER_MODE_OFF = 0
BMW_STEER_MODE_TORQUE = 1
BMW_STEER_MODE_ANGLE = 2
BMW_STEER_MODE_SOFTOFF = 3
BMW_GAS_ENABLE = 0xA5


def bmw_sum_fold_checksum(addr: int, dat: bytes | bytearray) -> int:
  checksum = addr
  for i, byte in enumerate(dat):
    checksum += 0 if i == 0 else byte
  checksum = (checksum & 0xFF) + (checksum >> 8)
  return checksum & 0xFF


def crc8_d5_forward(dat: bytes | bytearray) -> int:
  """Wim interceptor CRC: poly 0xD5, init 0xFF, forward over dat. Not toyota crc8_pedal."""
  crc = 0xFF
  for byte in dat:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ 0xD5) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc


class TestBmwSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[BMW_STEER_ADDR, 0], [BMW_GAS_ADDR, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (BMW_STEER_ADDR,)}
  FWD_BLACKLISTED_ADDRS = {2: [BMW_STEER_ADDR]}
  STANDSTILL_THRESHOLD = 0.0
  GAS_PRESSED_THRESHOLD = 20

  MAX_RATE_UP = 1
  MAX_RATE_DOWN = 2
  MAX_TORQUE_LOOKUP = [0], [96]
  MAX_RT_DELTA = 25
  MAX_TORQUE_ERROR = 16
  TORQUE_MEAS_TOLERANCE = 1

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.bmw, 0)
    self.safety.init_tests()
    self.cnt_22e = 0
    self.cnt_22f = 0
    self.cnt_200 = 0
    super().setUp()
    # 0x200 is rejected until 0x199 has been seen (interceptor heartbeat).
    self._rx(self._user_gas_msg(0))
    # Non-zero / TorqueControl 0x22E is rejected until 0x22F has been seen.
    self._rx(self._torque_meas_msg(0))

  def _pack_steer_cmd(self, torque: int, mode: int, counter: int, checksum: int | None = None):
    dat = bytearray(5)
    dat[1] = (counter & 0xF) | ((mode & 0x3) << 4)
    dat[4] = torque & 0xFF
    dat[0] = bmw_sum_fold_checksum(BMW_STEER_ADDR, dat) if checksum is None else checksum
    return make_msg(0, BMW_STEER_ADDR, dat=bytes(dat))

  def _torque_cmd_msg(self, torque, steer_req=1):
    mode = BMW_STEER_MODE_TORQUE if steer_req else BMW_STEER_MODE_OFF
    msg = self._pack_steer_cmd(int(torque), mode, self.cnt_22e)
    self.cnt_22e = (self.cnt_22e + 1) & 0xF
    return msg

  def _torque_meas_msg(self, torque):
    dat = bytearray(8)
    dat[1] = self.cnt_22f & 0xF
    dat[2] = int(torque) & 0xFF
    dat[0] = bmw_sum_fold_checksum(BMW_STEER_STATUS_ADDR, dat)
    self.cnt_22f = (self.cnt_22f + 1) & 0xF
    return make_msg(0, BMW_STEER_STATUS_ADDR, dat=bytes(dat))

  def _speed_msg(self, speed):
    raw = int(round((speed * 3.6) / 0.0625))
    raw = max(-32768, min(32767, raw))
    dat = bytearray(8)
    dat[4] = raw & 0xFF
    dat[5] = (raw >> 8) & 0xFF
    dat[6] = raw & 0xFF
    dat[7] = (raw >> 8) & 0xFF
    return make_msg(0, BMW_WHEEL_SPEEDS_ADDR, dat=bytes(dat))

  def _user_brake_msg(self, brake):
    dat = bytearray(8)
    dat[7] = 0x64 if brake else 0x04
    return make_msg(0, BMW_VCU_ABS_ADDR, dat=bytes(dat))

  def _user_gas_msg(self, gas, pedal_out=0):
    pedal_in = int(gas) & 0xFF
    dat = bytes([pedal_in, int(pedal_out) & 0xFF, 0])
    return make_msg(0, BMW_PEDAL_STATUS_ADDR, dat=dat)

  def _pcm_status_msg(self, enable):
    # Virtual cruise on 0x194: resume=engage, cancel_up_or_down=disengage.
    # Stalk is momentary on the car; panda latches until cancel. Tests hold the bit.
    dat = bytearray(4)
    dat[2] = 0x40 if enable else 0x10
    return make_msg(0, BMW_STALK_ADDR, dat=bytes(dat))

  def _acc_state_msg(self, enabled):
    return self._pcm_status_msg(enabled)

  def _gas_cmd(self, gas: int, enable: int = BMW_GAS_ENABLE, counter: int | None = None,
               crc: int | None = None):
    if counter is None:
      counter = self.cnt_200
      self.cnt_200 = (self.cnt_200 + 1) & 0xF
    dat = bytearray(4)
    dat[0] = int(gas) & 0xFF
    dat[1] = enable & 0xFF
    dat[2] = counter & 0xF
    dat[3] = crc8_d5_forward(dat[:3]) if crc is None else crc
    return make_msg(0, BMW_GAS_ADDR, dat=bytes(dat))

  def test_torque_absolute_limits(self):
    # STEER_TORQUE is 8-bit signed. The inherited test walks ±(max+1000) and wraps.
    for speed in self._torque_speed_range:
      self._reset_speed_measurement(speed)
      max_torque = self._get_max_torque(speed)
      for controls_allowed in [True, False]:
        for torque in range(-128, 128):
          self.safety.set_controls_allowed(controls_allowed)
          self.safety.set_rt_torque_last(torque)
          self.safety.set_torque_meas(torque, torque)
          self.safety.set_desired_torque_last(torque - self.MAX_RATE_UP)
          if controls_allowed:
            send = -max_torque <= torque <= max_torque
          else:
            send = torque == 0
          self.assertEqual(send, self._tx(self._torque_cmd_msg(torque)), (controls_allowed, torque))

  def test_exceed_torque_sensor(self):
    # Inherited test steps by 2; 8-bit rate-up is 1 count/frame.
    self.safety.set_controls_allowed(True)
    for sign in [-1, 1]:
      self._set_prev_torque(0)
      for t in np.arange(0, self.MAX_TORQUE_ERROR + 1, 1):
        t *= sign
        self.assertTrue(self._tx(self._torque_cmd_msg(t)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (self.MAX_TORQUE_ERROR + 1))))

  def test_steer_checksum_fail(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    dat = bytearray(5)
    dat[1] = (self.cnt_22e & 0xF) | (BMW_STEER_MODE_TORQUE << 4)
    dat[0] = (bmw_sum_fold_checksum(BMW_STEER_ADDR, dat) + 1) & 0xFF
    self.assertFalse(self._tx(make_msg(0, BMW_STEER_ADDR, dat=bytes(dat))))

  def test_steer_counter_stall(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    msg = self._torque_cmd_msg(0)
    self.assertTrue(self._tx(msg))
    self.assertFalse(self._tx(msg))

  def test_steer_angle_mode_rejected(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    msg = self._pack_steer_cmd(0, BMW_STEER_MODE_ANGLE, self.cnt_22e)
    self.cnt_22e = (self.cnt_22e + 1) & 0xF
    self.assertFalse(self._tx(msg))

  def test_steer_softoff_and_off_zero_torque(self):
    self.safety.set_controls_allowed(False)
    for mode in (BMW_STEER_MODE_OFF, BMW_STEER_MODE_SOFTOFF):
      msg = self._pack_steer_cmd(0, mode, self.cnt_22e)
      self.cnt_22e = (self.cnt_22e + 1) & 0xF
      self.assertTrue(self._tx(msg), f"mode {mode}")

  def test_steer_softoff_nonzero_torque_rejected(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    msg = self._pack_steer_cmd(8, BMW_STEER_MODE_SOFTOFF, self.cnt_22e)
    self.cnt_22e = (self.cnt_22e + 1) & 0xF
    self.assertFalse(self._tx(msg))

  def test_realtime_limit_up(self):
    # common.py calls init_tests() inside the loop, which clears bmw_seen_22f.
    self.safety.set_controls_allowed(True)

    for sign in [-1, 1]:
      self.safety.set_safety_hooks(CarParams.SafetyModel.bmw, 0)
      self.safety.init_tests()
      self.cnt_22e = 0
      self.cnt_22f = 0
      self.cnt_200 = 0
      self._rx(self._user_gas_msg(0))
      self._rx(self._torque_meas_msg(0))
      self._set_prev_torque(0)
      self.safety.set_controls_allowed(True)
      for t in np.arange(0, self.MAX_RT_DELTA + 1, 1):
        t *= sign
        self.safety.set_torque_meas(t, t)
        self.assertTrue(self._tx(self._torque_cmd_msg(t)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + 1))))

      self._set_prev_torque(0)
      for t in np.arange(0, self.MAX_RT_DELTA + 1, 1):
        t *= sign
        self.safety.set_torque_meas(t, t)
        self.assertTrue(self._tx(self._torque_cmd_msg(t)))

      self.safety.set_timer(common.RT_INTERVAL + 1)
      # Keep 0x22F heartbeat current; 50 ms lag is a real lat reject.
      self._rx(self._torque_meas_msg(sign * self.MAX_RT_DELTA))
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * self.MAX_RT_DELTA)))
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + 1))))

  def test_spam_torque_beyond_max(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(self.MAX_TORQUE)
    self.safety.set_torque_meas(self.MAX_TORQUE, self.MAX_TORQUE)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE + 1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(-(self.MAX_TORQUE + 1))))

  def test_gas_without_enable_rejected(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._gas_cmd(50, enable=0x00)))
    self.assertFalse(self._tx(self._gas_cmd(50, enable=0x01)))
    self.assertTrue(self._tx(self._gas_cmd(50, enable=BMW_GAS_ENABLE)))

  def test_gas_crc_fail_rejected(self):
    self.safety.set_controls_allowed(True)
    dat = bytearray(4)
    dat[0] = 40
    dat[1] = BMW_GAS_ENABLE
    dat[2] = self.cnt_200 & 0xF
    dat[3] = (crc8_d5_forward(dat[:3]) + 1) & 0xFF
    self.assertFalse(self._tx(make_msg(0, BMW_GAS_ADDR, dat=bytes(dat))))

  def test_gas_blocked_when_controls_not_allowed(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._gas_cmd(0, enable=BMW_GAS_ENABLE)))
    self.assertFalse(self._tx(self._gas_cmd(0, enable=0x00)))
    self.assertFalse(self._tx(self._gas_cmd(50, enable=BMW_GAS_ENABLE)))

  def test_gas_toyota_6byte_rejected(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(make_msg(0, BMW_GAS_ADDR, dat=b"\x00" * 6)))

  def test_gas_counter_stall(self):
    self.safety.set_controls_allowed(True)
    msg = self._gas_cmd(10)
    self.assertTrue(self._tx(msg))
    self.assertFalse(self._tx(msg))

  def test_toyota_interceptor_inactive_not_used(self):
    # toyota longitudinal_interceptor_checks allows 0x200 with byte0=byte1=0 when long not allowed.
    # Wim byte1 is enable 0xA5; panda must block ALL 0x200 without controls_allowed.
    self.safety.set_controls_allowed(False)
    dat = bytearray(4)
    dat[3] = crc8_d5_forward(dat[:3])
    self.assertFalse(self._tx(make_msg(0, BMW_GAS_ADDR, dat=bytes(dat))))

  def test_brake_0a8_disengages_long(self):
    self._rx(self._speed_msg(0))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(False))
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_longitudinal_allowed())

  def test_stalk_not_injectable(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(make_msg(0, BMW_STALK_ADDR, dat=b"\x00" * 4)))
    self.assertFalse(self._tx(make_msg(0, BMW_STALK_ADDR, dat=b"\x00" * 8)))

  def test_torque_cut_not_injectable(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(make_msg(0, BMW_TORQUE_CUT_ADDR, dat=b"\x00" * 3)))
    self.assertFalse(self._tx(make_msg(0, BMW_TORQUE_CUT_ADDR, dat=b"\x00" * 8)))

  def test_dsc_and_1a0_not_injectable(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(make_msg(0, BMW_DSC_ADDR, dat=b"\x00" * 8)))
    self.assertFalse(self._tx(make_msg(0, BMW_SPEED_POISON_ADDR, dat=b"\x00" * 8)))

  def test_gas_human_override(self):
    self._rx(self._user_gas_msg(0, pedal_out=0))
    self.assertFalse(self.safety.get_gas_pressed_prev())
    # PedalIn > 20 but tracking PedalOut: not human override
    self._rx(self._user_gas_msg(50, pedal_out=50))
    self.assertFalse(self.safety.get_gas_pressed_prev())
    # Human above interceptor output
    self._rx(self._user_gas_msg(50, pedal_out=10))
    self.assertTrue(self.safety.get_gas_pressed_prev())

  def test_mads_lat_without_long(self):
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
    self.assertFalse(self._tx(self._gas_cmd(20)))

  def test_gas_requires_pedal_status_rx(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.bmw, 0)
    self.safety.init_tests()
    self.cnt_200 = 0
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._gas_cmd(10)))
    self._rx(self._user_gas_msg(0))
    self.assertTrue(self._tx(self._gas_cmd(10)))

  def test_pedal_status_timeout_blocks_gas(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._user_gas_msg(0))
    self.assertTrue(self._tx(self._gas_cmd(10)))
    self.safety.set_timer(200001)
    self.assertFalse(self._tx(self._gas_cmd(10)))

  def test_steer_status_timeout_blocks_lat(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._torque_meas_msg(0))
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.safety.set_timer(50001)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
    # SoftOff with 0 torque still allowed so we can drop actuation
    msg = self._pack_steer_cmd(0, BMW_STEER_MODE_SOFTOFF, self.cnt_22e)
    self.cnt_22e = (self.cnt_22e + 1) & 0xF
    self.assertTrue(self._tx(msg))

  def test_virtual_cruise_idle_holds(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    idle = make_msg(0, BMW_STALK_ADDR, dat=b"\x00" * 4)
    self._rx(idle)
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_set_plus1_engages(self):
    dat = bytearray(4)
    dat[2] = 0x01  # plus1
    self._rx(make_msg(0, BMW_STALK_ADDR, dat=bytes(dat)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_steer_status_bad_checksum_rejected(self):
    dat = bytearray(8)
    dat[1] = 1
    dat[2] = 10
    dat[0] = (bmw_sum_fold_checksum(BMW_STEER_STATUS_ADDR, dat) + 1) & 0xFF
    self.assertFalse(self._rx(make_msg(0, BMW_STEER_STATUS_ADDR, dat=bytes(dat))))

  def test_enum_is_37(self):
    self.assertEqual(int(CarParams.SafetyModel.bmw), 37)
    self.assertEqual(self.safety.get_current_safety_mode(), 37)


if __name__ == "__main__":
  unittest.main()
