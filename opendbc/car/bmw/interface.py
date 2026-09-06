from opendbc.car import get_safety_config, structs
from opendbc.car.bmw.carcontroller import CarController
from opendbc.car.bmw.carstate import CarState
from opendbc.car.bmw.radar_interface import RadarInterface
from opendbc.car.bmw.values import CarControllerParams, E90Flags
from opendbc.car.interfaces import CarInterfaceBase

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "bmw"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.bmw)]
    ret.dashcamOnly = False
    ret.radarUnavailable = True
    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.transmissionType = TransmissionType.direct

    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 0.8
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.pcmCruise = False  # stock DCC is dead; OP + MADS own enable
    ret.autoResumeSng = False
    ret.alphaLongitudinalAvailable = False
    # alpha_long must not enable the interceptor. Set E90Flags.PEDAL_INTERCEPTOR after
    # 0x199 is on panda CAN0 and lat is proven. Until then, no 0x200 TX.
    ret.openpilotLongitudinalControl = bool(ret.flags & E90Flags.PEDAL_INTERCEPTOR)
    ret.stopAccel = 0.0  # no friction brake; do not request a stop

    # LongControl: kp is 0 in stock. I-term on (aTarget - aEgo) is what tracks
    # when regen/SOC/power limits move the plant. Honda interceptor does the same.
    ret.longitudinalTuning.kiBP = [0., 5., 35.]
    ret.longitudinalTuning.kiV = [0.8, 0.5, 0.35]

    return ret

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX
