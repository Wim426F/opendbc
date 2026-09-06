from opendbc.can import CANPacker
from opendbc.car.bmw.values import CanBus, SteeringModes

GAS_ENABLE_MAGIC = 0xA5
CRUISE_GAS_ADDR = 0x200


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


def crc8_wim_pedal(data: bytes, poly: int = 0xD5) -> int:
  """CRC8 poly 0xD5, init 0xFF, FORWARD over data (bytes 0..2 of 0x200).

  Not toyota crc8_pedal (that walks backwards).
  """
  crc = 0xFF
  for b in data:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


def create_pedal_command(gas_raw: int, idx: int, bus: int = CanBus.pt):
  """0x200 CRUISE_Gas, 4 bytes. Call only when longActive and controls_allowed.

  Stop sending when inactive (100 ms timeout → passthrough). Do not pack enable=0.
  """
  dat = bytearray(4)
  dat[0] = int(max(0, min(255, gas_raw)))
  dat[1] = GAS_ENABLE_MAGIC
  dat[2] = int(idx) & 0xF
  dat[3] = crc8_wim_pedal(bytes(dat[:3]))
  return CRUISE_GAS_ADDR, bytes(dat), bus
