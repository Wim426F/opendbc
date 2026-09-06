import math

from opendbc.can import CANParser
from opendbc.can.dbc import DBC
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.bmw.values import DBC as BMW_DBC, GEAR_MAP, CanBus, CarControllerParams
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

STANDSTILL_THRESHOLD = 0.3  # m/s
ABS_ADDR = 0xA8


class RawFrameCache:
  """Last payload for IDs we may need raw (0x0A8) if a DBC signal is missing."""

  def __init__(self, addrs: tuple[int, ...], bus: int = CanBus.pt):
    self.dbc_name = ""
    self.bus = bus
    self.addrs = addrs
    self.last: dict[int, bytes] = {}
    self.can_valid = True

  @property
  def bus_timeout(self) -> bool:
    return False

  def update(self, strings, sendcan: bool = False):
    if strings and not isinstance(strings[0], list | tuple):
      strings = [strings]
    for entry in strings:
      frames = entry[1]
      for address, dat, src in frames:
        if src == self.bus and address in self.addrs:
          self.last[address] = bytes(dat)
    return set()


def _try_can_parser(dbc_name: str | None, messages: list[tuple[str | int, int]], bus: int) -> CANParser | None:
  if not dbc_name:
    return None
  try:
    dbc = DBC(dbc_name)
  except FileNotFoundError:
    return None
  filtered = []
  for name, freq in messages:
    if isinstance(name, int):
      if name in dbc.addr_to_msg:
        filtered.append((name, freq))
    elif name in dbc.name_to_msg:
      filtered.append((name, freq))
  return CANParser(dbc_name, filtered, bus)


def _has_msg(parser, name: str) -> bool:
  return parser is not None and name in parser.dbc.name_to_msg


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.plus1 = 0
    self.minus1 = 0
    self.plus5 = 0
    self.minus5 = 0
    self.resume = 0
    self.cancel = 0
    self.cancel_lever_up = 0

  def _brake_pressed(self, can_parsers) -> bool:
    cp_alt = can_parsers.get(Bus.alt)
    if _has_msg(cp_alt, "SendAbsDscMessages"):
      return int(cp_alt.vl["SendAbsDscMessages"]["BrakeMagic"]) == CarControllerParams.BRAKE_PRESSED_MAGIC
    dat = can_parsers.get(Bus.loopback).last.get(ABS_ADDR) if Bus.loopback in can_parsers else None
    if dat is not None and len(dat) >= 8:
      return dat[7] == CarControllerParams.BRAKE_PRESSED_MAGIC
    return False

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers.get(Bus.pt)
    cp_alt = can_parsers.get(Bus.alt)
    cp_body = can_parsers.get(Bus.body)
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # --- speed: driven axle RL/RR. Never 0x1A0 (gateway clamps ≥ 10 km/h). ---
    if _has_msg(cp, "WheelSpeeds"):
      fl = cp.vl["WheelSpeeds"]["Wheel_FL"]
      fr = cp.vl["WheelSpeeds"]["Wheel_FR"]
      rl = cp.vl["WheelSpeeds"]["Wheel_RL"]
      rr = cp.vl["WheelSpeeds"]["Wheel_RR"]
      ret.wheelSpeeds.fl = fl * CV.KPH_TO_MS
      ret.wheelSpeeds.fr = fr * CV.KPH_TO_MS
      ret.wheelSpeeds.rl = rl * CV.KPH_TO_MS
      ret.wheelSpeeds.rr = rr * CV.KPH_TO_MS
      self.parse_wheel_speeds(ret, rl, rr, rl, rr)
    ret.standstill = ret.vEgoRaw < STANDSTILL_THRESHOLD

    if _has_msg(cp, "SteeringWheelAngle"):
      ret.steeringAngleDeg = cp.vl["SteeringWheelAngle"]["SteeringPosition"]
      ret.steeringRateDeg = cp.vl["SteeringWheelAngle"]["SteeringSpeed"]

    # 0x22F motor torque is actuator, not a torsion bar. Do not treat as driver override.
    if _has_msg(cp_body, "STEERING_STATUS"):
      ret.steeringTorqueEps = cp_body.vl["STEERING_STATUS"]["STEERING_TORQUE"]
      status = int(cp_body.vl["STEERING_STATUS"]["CONTROL_STATUS"])
      ret.steerFaultTemporary = (status & 0x4) != 0

    if _has_msg(cp, "TurnSignals"):
      blinker_on = cp.vl["TurnSignals"]["TurnSignalActive"] != 0 and cp.vl["TurnSignals"]["TurnSignalIdle"] == 0
      ret.leftBlinker = blinker_on and cp.vl["TurnSignals"]["LeftTurn"] != 0
      ret.rightBlinker = blinker_on and cp.vl["TurnSignals"]["RightTurn"] != 0

    # --- gear 0x3FD ---
    if _has_msg(cp_alt, "LeverCommand_From_VCU"):
      direction = int(cp_alt.vl["LeverCommand_From_VCU"]["Direction"])
      ret.gearShifter = GEAR_MAP.get(direction, GearShifter.unknown)
    else:
      ret.gearShifter = GearShifter.unknown

    # --- pedal 0x199. Do not use e9x AccPedal. ---
    pedal_in = 0.0
    pedal_out = 0.0
    if _has_msg(cp_alt, "PEDAL_Status"):
      pedal_in = float(cp_alt.vl["PEDAL_Status"]["PedalInPct"])
      pedal_out = float(cp_alt.vl["PEDAL_Status"]["PedalOutPct"])
    ret.deprecated.gas = pedal_in / 100.0
    ret.gasPressed = (pedal_in > CarControllerParams.GAS_PRESSED_THRESHOLD and
                      pedal_in > pedal_out + CarControllerParams.GAS_OVERRIDE_MARGIN)

    # --- brake: VCU 0x0A8 byte7. Do not use EngineAndBrake / 0x19E. ---
    ret.brakePressed = self._brake_pressed(can_parsers)

    if _has_msg(cp_alt, "EPB_Status"):
      ret.parkingBrake = int(cp_alt.vl["EPB_Status"]["ParkingBrake"]) == 2
    else:
      dat_epb = can_parsers.get(Bus.loopback).last.get(0x3FE) if Bus.loopback in can_parsers else None
      if dat_epb is not None and len(dat_epb) >= 1:
        ret.parkingBrake = dat_epb[0] == 2

    # --- cruise stalk 0x194 RX only. Never TX. Do not parse e9x CruiseControlStatus 0x200. ---
    prev_plus1, prev_minus1 = self.plus1, self.minus1
    prev_plus5, prev_minus5 = self.plus5, self.minus5
    prev_resume, prev_cancel = self.resume, self.cancel
    prev_cancel_up = self.cancel_lever_up
    if _has_msg(cp, "CruiseControl"):
      stalk = cp.vl["CruiseControl"]
      self.plus1 = int(stalk["plus1mph_request"])
      self.minus1 = int(stalk["minus1mph_request"])
      self.plus5 = int(stalk["plus5mph_request"])
      self.minus5 = int(stalk["minus5mph_request"])
      self.resume = int(stalk["Resume_request"])
      self.cancel = int(stalk["Cancel_request_up_or_down_stalk"])
      self.cancel_lever_up = int(stalk["Cancel_request_up_stalk"])

    ret.buttonEvents = [
      *create_button_events(self.plus1, prev_plus1, {1: ButtonType.accelCruise}),
      *create_button_events(self.minus1, prev_minus1, {1: ButtonType.decelCruise}),
      *create_button_events(self.plus5, prev_plus5, {1: ButtonType.accelCruise}),
      *create_button_events(self.minus5, prev_minus5, {1: ButtonType.decelCruise}),
      *create_button_events(self.resume, prev_resume, {1: ButtonType.resumeCruise}),
      *create_button_events(self.cancel, prev_cancel, {1: ButtonType.cancel}),
      *create_button_events(self.cancel_lever_up, prev_cancel_up, {1: ButtonType.lkas}),
    ]

    ignition = True
    if _has_msg(cp, "TerminalStatus"):
      ignition = not bool(cp.vl["TerminalStatus"]["IgnitionOff"])
    ret.cruiseState.available = ignition and ret.gearShifter == GearShifter.drive
    ret.cruiseState.enabled = False  # stock DCC is dead; OP state machine owns enable
    ret.cruiseState.nonAdaptive = True
    ret.cruiseState.standstill = ret.standstill

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    dbc = BMW_DBC[CP.carFingerprint]
    parsers = {}

    pt = _try_can_parser(dbc.get(Bus.pt), [
      ("WheelSpeeds", 50),
      ("SteeringWheelAngle", 100),
      ("CruiseControl", 5),
      ("TerminalStatus", math.nan),  # CAS 0x130 may be missing on can2
      ("TurnSignals", math.nan),
      ("SteeringButtons", math.nan),
    ], CanBus.pt)
    if pt is not None:
      parsers[Bus.pt] = pt

    alt = _try_can_parser(dbc.get(Bus.alt), [
      ("PEDAL_Status", 10),
      ("LeverCommand_From_VCU", 10),
      ("EPB_Status", 10),
      ("SendAbsDscMessages", 100),
      ("ID108DIR_torque_From_Motor", math.nan),
    ], CanBus.pt)
    if alt is not None:
      parsers[Bus.alt] = alt

    body = _try_can_parser(dbc.get(Bus.body), [
      ("STEERING_STATUS", math.nan),  # required by panda when lat is on; optional for listen
    ], CanBus.pt)
    if body is not None:
      parsers[Bus.body] = body

    parsers[Bus.loopback] = RawFrameCache((ABS_ADDR, 0x3FE), CanBus.pt)
    return parsers
