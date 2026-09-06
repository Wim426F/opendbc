import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, rate_limit
from opendbc.car.bmw import bmwcan
from opendbc.car.bmw.values import CarControllerParams, LONG_DROP_SPEED, MIN_ENABLE_SPEED, SteeringModes
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.apply_steer_last = 0.0
    self.soft_off_counter = CarControllerParams.SOFT_OFF_FRAMES
    self.long_speed_ok = False
    self.gas_counter = 0
    self.packer = CANPacker(dbc_names[Bus.body]) if Bus.body in dbc_names else None

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Lateral: 0x22E. Long: 0x200 only if PEDAL_INTERCEPTOR (off until interceptor is on this bus).
    # Never TX 0x194 / 0x201.
    apply_steer = 0.0
    mode = SteeringModes.Off

    if self.packer is not None and self.frame % CarControllerParams.STEER_STEP == 0:
      if CC.latActive:
        new_steer = float(np.clip(actuators.torque, -1.0, 1.0)) * CarControllerParams.STEER_MAX
        apply_steer = rate_limit(new_steer, self.apply_steer_last,
                                 -CarControllerParams.STEER_DELTA_DOWN, CarControllerParams.STEER_DELTA_UP)
        mode = SteeringModes.TorqueControl
        self.soft_off_counter = 0
      elif CS.out.steerFaultPermanent:
        apply_steer = 0.0
        mode = SteeringModes.Off
        self.soft_off_counter = CarControllerParams.SOFT_OFF_FRAMES
      elif self.soft_off_counter < CarControllerParams.SOFT_OFF_FRAMES:
        apply_steer = 0.0
        mode = SteeringModes.SoftOff
        self.soft_off_counter += 1
      else:
        apply_steer = 0.0
        mode = SteeringModes.Off

      apply_steer = float(np.clip(apply_steer, -CarControllerParams.STEER_MAX, CarControllerParams.STEER_MAX))
      can_sends.append(bmwcan.create_steer_command(self.packer, mode, apply_steer, self.frame))
      self.apply_steer_last = apply_steer

    # Interceptor: 50 Hz while longActive above 30 km/h; stop sending below 25 km/h or when inactive.
    if CS.out.vEgo >= MIN_ENABLE_SPEED:
      self.long_speed_ok = True
    elif CS.out.vEgo < LONG_DROP_SPEED:
      self.long_speed_ok = False

    apply_accel = 0.0
    if self.CP.openpilotLongitudinalControl and CC.longActive and self.long_speed_ok:
      apply_accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      if self.frame % CarControllerParams.LONG_TX_STEP == 0:
        gas_raw = CarControllerParams.accel_to_gas_raw(apply_accel)
        can_sends.append(bmwcan.create_pedal_command(gas_raw, self.gas_counter))
        self.gas_counter = (self.gas_counter + 1) & 0xF

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_steer_last
    new_actuators.accel = apply_accel

    self.frame += 1
    return new_actuators, can_sends
