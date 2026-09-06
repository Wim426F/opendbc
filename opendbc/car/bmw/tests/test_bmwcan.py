from opendbc.car.bmw import bmwcan
from opendbc.car.bmw.values import CarControllerParams


def test_wim_pedal_crc_worked_example():
  assert bmwcan.crc8_wim_pedal(bytes([51, 0xA5, 1])) == 0xD5
  addr, dat, bus = bmwcan.create_pedal_command(51, 1)
  assert addr == 0x200
  assert bus == 0
  assert dat == bytes([0x33, 0xA5, 0x01, 0xD5])


def test_accel_to_gas_coast_and_clip():
  # 0 m/s² → 8 % → 20/255
  assert CarControllerParams.accel_to_gas_raw(0.0) == 20
  # 0 % pedal for regen at or below measured foot-off
  assert CarControllerParams.accel_to_gas_raw(-0.4) == 0
  assert CarControllerParams.accel_to_gas_raw(-1.5) == 0
  # +1.5 m/s² → 30 %
  assert CarControllerParams.accel_to_gas_raw(1.5) == 76
  assert CarControllerParams.accel_to_gas_raw(3.0) == 76  # clip
