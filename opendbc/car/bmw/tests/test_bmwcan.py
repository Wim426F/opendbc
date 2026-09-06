from opendbc.car.bmw import bmwcan
from opendbc.car.bmw.values import CarControllerParams


def test_wim_pedal_crc_worked_example():
  assert bmwcan.crc8_wim_pedal(bytes([51, 0xA5, 1])) == 0xD5
  addr, dat, bus = bmwcan.create_pedal_command(51, 1)
  assert addr == 0x200
  assert bus == 0
  assert dat == bytes([0x33, 0xA5, 0x01, 0xD5])


def test_accel_to_gas_monotonic_full_authority():
  # Weak FF only: 0 m/s² → coast guess 10 %; I-term trims in LongControl.
  assert CarControllerParams.accel_to_gas_raw(0.0) == 26  # 10% * 255
  assert CarControllerParams.accel_to_gas_raw(-1.5) == 0   # max regen request
  assert CarControllerParams.accel_to_gas_raw(1.5) == CarControllerParams.PEDAL_MAX_RAW
  assert CarControllerParams.accel_to_gas_raw(3.0) == CarControllerParams.PEDAL_MAX_RAW
  assert CarControllerParams.PEDAL_MAX_RAW == 102  # 40 %
  assert CarControllerParams.accel_to_pedal_pct(1.5) == CarControllerParams.PEDAL_MAX_PCT
  # Strictly increasing through the clip range
  prev = -1
  for a in (-1.5, -0.75, 0.0, 0.75, 1.5):
    raw = CarControllerParams.accel_to_gas_raw(a)
    assert raw >= prev
    prev = raw


def test_slew_pedal_does_not_step_to_home():
  # 25 %/s down, DT_CTRL=0.01 s → 0.25 % per control step
  nxt = CarControllerParams.slew_pedal_pct(0.0, 40.0)
  assert nxt == 39.75
  nxt = CarControllerParams.slew_pedal_pct(100.0, 40.0)
  assert nxt == 40.5
