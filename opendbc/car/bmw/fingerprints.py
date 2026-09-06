"""AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.bmw.values import CAR

# Static fingerprint of panda CAN0 = SavvyCAN bus 0 (BMW PT after gateway / VCU vehicle).
# Built from smalldrivearoundtheblock.csv (2026-09). Extra 11-bit IDs on the bus
# eliminate the candidate, so this is the full observed set plus interceptor/servo
# frames that will join the same bus (not Tesla/PCS log bus 1).
FINGERPRINTS = {
  CAR.BMW_E90_EV: [{
    0xA8: 8,     # VCU SendAbsDscMessages
    0xA9: 8,
    0xAA: 8,
    0xB6: 5,
    0xC4: 7,     # SAS
    0xC8: 6,
    0xCE: 8,     # WheelSpeeds
    0x130: 5,    # TerminalStatus (5 B on this car, DBC says 8)
    0x135: 2,
    0x194: 4,    # Cruise stalk
    0x197: 4,
    0x19E: 8,    # stock DSC (ignore for brake)
    0x1A0: 8,    # speed POISON ≥10 km/h
    0x1A6: 8,
    0x1B4: 8,
    0x1B5: 7,
    0x1D0: 8,
    0x1D6: 2,    # BMW SZL buttons (NOT Tesla 8 B 0x1D6)
    0x1F6: 2,
    0x1FB: 2,
    0x202: 2,
    0x21A: 3,
    0x226: 5,
    0x242: 5,
    0x285: 8,
    0x2A6: 2,
    0x2B2: 8,    # i3LIM 8 B on this bus (PCS 5 B stays on Tesla/PCS)
    0x2C0: 3,
    0x2F6: 2,
    0x2F8: 8,
    0x2FA: 5,
    0x2FC: 7,
    0x310: 7,
    0x311: 2,
    0x31D: 2,
    0x330: 8,
    0x34F: 2,
    0x374: 5,
    0x398: 8,
    0x3B0: 2,
    0x3BE: 2,
    0x3FD: 5,    # F30 Direction
    0x3FE: 7,    # EPB
    0x480: 8,
    0x4A9: 8,
    0x4B0: 8,
    0x55E: 8,
    0x5A9: 8,
    0x5E2: 8,
    0x629: 8,
    0x62D: 8,
    0x630: 8,
    0x6BD: 8,
    0x6F0: 8,
    # Joining this bus (human): pedal interceptor + servo. Not on the 2026-09 log.
    0x199: 3,    # PEDAL_Status
    0x201: 3,    # VCU torque-cut (if bridged; ignore as actuator)
    0x22F: 8,    # STEERING_STATUS
  }],
}

FW_VERSIONS = {}
