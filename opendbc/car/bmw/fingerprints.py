"""AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.bmw.values import CAR

# Static can2 fingerprint. No ECU UDS. Lengths from DBC / firmware.
FINGERPRINTS = {
  CAR.BMW_E90_EV: [{
    0xA8: 8,    # SendAbsDscMessages (VCU brake)
    0xC4: 7,    # SteeringWheelAngle
    0xCE: 8,    # WheelSpeeds
    0x107: 8,   # ID108DIR_torque_From_Motor
    0x194: 4,   # CruiseControl stalk
    0x199: 3,   # PEDAL_Status
    0x22F: 8,   # STEERING_STATUS
    0x3FD: 5,   # LeverCommand_From_VCU
    0x3FE: 7,   # EPB_Status
  }],
}

FW_VERSIONS = {}
