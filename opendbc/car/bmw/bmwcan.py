from opendbc.can import CANPacker
from opendbc.car.bmw.values import CanBus, SteeringModes


def calc_checksum_8bit(work_data: bytes | bytearray, msg_id: int) -> int:
  """StepperServoCAN Msg_calc_checksum_8bit / E8x bmwcan.

  checksum = addr; add every payload byte (CHECKSUM byte must already be 0);
  fold (s & 0xFF) + (s >> 8); keep low byte.
  """
  checksum = msg_id
  for byte in work_data:
    checksum += byte
  checksum = (checksum & 0xFF) + (checksum >> 8)
  checksum &= 0xFF
  return checksum


def create_steer_command(packer: CANPacker, mode: SteeringModes | int, torque_nm: float, frame: int,
                         steer_angle: float = 0.0, bus: int = CanBus.pt):
  """0x22E STEERING_COMMAND, 5 bytes, 10 ms. panda bus 0.

  Two-pass pack: CHECKSUM=0, sum-fold, pack again. Do not use packer auto-checksum.
  """
  values = {
    "COUNTER": frame % 16,
    "STEER_MODE": int(mode),
    "STEER_ANGLE": steer_angle,
    "STEER_TORQUE": torque_nm,
    "CHECKSUM": 0,
  }
  addr, dat, _bus = packer.make_can_msg("STEERING_COMMAND", bus, values)
  values["CHECKSUM"] = calc_checksum_8bit(dat, addr)
  return packer.make_can_msg("STEERING_COMMAND", bus, values)
