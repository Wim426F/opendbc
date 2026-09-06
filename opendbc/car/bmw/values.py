from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

import numpy as np

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts, SupportType
from opendbc.car.structs import CarState

GearShifter = CarState.GearShifter

# v1 long enable floor. 30 km/h; drop below 25 km/h is carcontroller hysteresis.
MIN_ENABLE_SPEED = 8.3  # m/s  (30 km/h)
LONG_DROP_SPEED = 25. * CV.KPH_TO_MS
MIN_STEER_SPEED = 10. * CV.KPH_TO_MS


class CanBus:
  """Single panda CAN0 = SavvyCAN bus 0 = gateway CAN2 / VCU vehicle / BMW PT after GW.

  Do not splice Tesla/PCS (SavvyCAN bus 1): ID collisions 0x1D6/0x2B2/0x398.
  Pedal 0x199/0x200 and servo 0x22E/0x22F are moved onto this bus (human).
  Three DBC parsers share this one hardware bus (e9x + powertrain + ocelot).
  """
  pt = 0


class SteeringModes(IntEnum):
  Off = 0
  TorqueControl = 1
  AngleControl = 2
  SoftOff = 3


class E90Flags(IntFlag):
  STEPPER_SERVO_CAN = 1 << 0
  NORMAL_CRUISE_CONTROL = 1 << 1  # VO540 / $540 stalk speed +/- . Never TX 0x194.
  PEDAL_INTERCEPTOR = 1 << 2  # long later; leave OFF so carcontroller must not TX 0x200


class CarControllerParams:
  # Nm at the column. DBC ±16 Nm; start half of the E8x 12 Nm scale.
  STEER_MAX = 6.0
  STEER_DELTA_UP = 0.10    # Nm per 10 ms → 10 Nm/s
  STEER_DELTA_DOWN = 0.20  # faster unwind than windup
  STEER_STEP = 1           # 100 Hz, servo wants 10 ms
  STEER_DRIVER_ALLOWANCE = 1.0
  STEER_DRIVER_MULTIPLIER = 1
  STEER_DRIVER_FACTOR = 1
  SOFT_OFF_FRAMES = 100    # 1 s SoftOff ramp, then Off

  # Pedal human-override (panda + carstate). Percent 0–100.
  GAS_PRESSED_THRESHOLD = 20.0
  GAS_OVERRIDE_MARGIN = 5.0

  BRAKE_PRESSED_MAGIC = 0x64  # VCU 0x0A8 byte7
  BRAKE_RELEASED_MAGIC = 0x04

  # Planner clip. Neighborhood log never needed more than +1.5 to follow 30 % pedal.
  ACCEL_MAX = 1.5
  ACCEL_MIN = -1.5

  # Feedforward: actuators.accel (m/s²) → BMW pedal % (interceptor GasReq).
  # From smalldrivearoundtheblock.csv, D, brake released, PedalIn vs dv/dt on 0xCE.
  # 0 % is Tesla unpressed (regen). This drive's foot-off regen was only ~−0.4 m/s²
  # (15–55 km/h, not highway pack-limited). Highway may be stronger; 0 % still
  # means "max regen available". Do not extrapolate above 35 % (not in the log).
  # Zero accel ≈ 8 % pedal. +1.5 m/s² ≈ 30 % pedal.
  ACCEL_TO_PEDAL_BP = [-1.50, -0.40, 0.00, 0.21, 0.43, 0.82, 1.24, 1.50]
  PEDAL_PCT_V =       [  0.0,   0.0,  8.0, 12.5, 17.5, 22.5, 27.5, 30.0]
  GAS_COAST_PCT = 8.0
  LONG_TX_STEP = 2  # 50 Hz from 100 Hz loop

  def __init__(self, CP):
    pass

  @classmethod
  def accel_to_gas_raw(cls, accel: float) -> int:
    """Clip accel and map to interceptor byte0 (0–255)."""
    a = float(np.clip(accel, cls.ACCEL_MIN, cls.ACCEL_MAX))
    pct = float(np.interp(a, cls.ACCEL_TO_PEDAL_BP, cls.PEDAL_PCT_V))
    return int(round(max(0.0, min(100.0, pct)) * 255.0 / 100.0))


@dataclass
class BMWCarDocs(CarDocs):
  package: str = "StepperServoCAN + custom can2 harness"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  support_type: SupportType = SupportType.CUSTOM
  support_link: str = "#community"


@dataclass
class BMWPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'bmw_e9x_e8x',
    Bus.alt: 'bmw_can2_powertrain',
    Bus.body: 'ocelot_controls',
  })


class CAR(Platforms):
  BMW_E90_EV = BMWPlatformConfig(
    [BMWCarDocs("BMW 320i E90 EV 2006")],
    CarSpecs(
      mass=1570.,
      wheelbase=2.76,
      steerRatio=16.0,  # start 16. Log yaw-from-axle vs SAS ~10 (n=7); re-measure with servo
      centerToFrontRatio=0.44,  # RWD
      minSteerSpeed=MIN_STEER_SPEED,
      minEnableSpeed=MIN_ENABLE_SPEED,
    ),
    flags=E90Flags.STEPPER_SERVO_CAN | E90Flags.NORMAL_CRUISE_CONTROL,
  )


# 0x3FD LeverCommand_From_VCU Direction
GEAR_MAP = {
  32: GearShifter.park,
  64: GearShifter.reverse,
  96: GearShifter.neutral,
  128: GearShifter.drive,
  129: GearShifter.sport,
}


DBC = CAR.create_dbc_map()
