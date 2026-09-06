import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, rate_limit
from opendbc.car.bmw import bmwcan
from opendbc.car.bmw.values import CarControllerParams, SteeringModes
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.apply_steer_last = 0.0
    self.soft_off_counter = CarControllerParams.SOFT_OFF_FRAMES
    self.packer = CANPacker(dbc_names[Bus.body]) if Bus.body in dbc_names else None

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # v1: 0x22E only. Do not send 0x200 / 0x194 / 0x201 even if long is later flagged on.
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

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_steer_last

    self.frame += 1
    return new_actuators, can_sends
