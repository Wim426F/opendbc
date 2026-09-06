from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

import numpy as np

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, DT_CTRL, rate_limit
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

  # Planner / LongControl clip (m/s²). Not a plant model.
  # 0 % interceptor = Tesla unpressed = whatever regen the DU will give *right now*
  # (SOC, temp, speed, your regen setting). OP cannot regen harder than that.
  ACCEL_MAX = 1.5
  ACCEL_MIN = -1.5

  # Hard ceiling on interceptor command. I-term must not wind to 100 % because
  # AccY has not reached aTarget (power/SOC/speed). Neighborhood log: ~30 % ≈
  # +1.5 m/s². Panda must use the same raw cap (PEDAL_MAX_RAW).
  PEDAL_MAX_PCT = 40.0
  PEDAL_MAX_RAW = int(round(40.0 * 255.0 / 100.0))  # 102; keep in sync with bmw.h

  # Monotonic FF only. +ACCEL_MAX maps to PEDAL_MAX_PCT, not 100 %.
  ACCEL_TO_PEDAL_BP = [-1.50, 0.00, 1.50]
  PEDAL_PCT_V =       [  0.0, 10.0, PEDAL_MAX_PCT]
  GAS_COAST_PCT = 10.0
  LONG_TX_STEP = 2  # 50 Hz from 100 Hz loop (DT_CTRL = 10 ms)

  # Actuator rate limits (% Tesla pedal / s). Single limiter; not a jerk model.
  # Down = toward home = more regen. 25 %/s => 100 %→0 % in 4 s while tracking.
  # Disable does not use these: 0x200 stops immediately.
  PEDAL_MAX_DOWN_PCT_S = 25.0
  PEDAL_MAX_UP_PCT_S = 50.0

  def __init__(self, CP):
    pass

  @classmethod
  def accel_to_pedal_pct(cls, accel: float) -> float:
    a = float(np.clip(accel, cls.ACCEL_MIN, cls.ACCEL_MAX))
    pct = float(np.interp(a, cls.ACCEL_TO_PEDAL_BP, cls.PEDAL_PCT_V))
    return float(np.clip(pct, 0.0, cls.PEDAL_MAX_PCT))

  @classmethod
  def accel_to_gas_raw(cls, accel: float) -> int:
    return int(round(cls.accel_to_pedal_pct(accel) * 255.0 / 100.0))

  @classmethod
  def slew_pedal_pct(cls, target: float, last: float) -> float:
    down = -cls.PEDAL_MAX_DOWN_PCT_S * DT_CTRL
    up = cls.PEDAL_MAX_UP_PCT_S * DT_CTRL
    return rate_limit(target, last, down, up)


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
