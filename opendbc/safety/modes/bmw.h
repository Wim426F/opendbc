#pragma once

#include "opendbc/safety/declarations.h"

// BMW E90 EV (Wim426F): StepperServoCAN lateral + Wim pedal interceptor long.
// Panda CAN0 = can2_powertrain. Do not TX 0x194, 0x201, DSC, or 0x1A0.

#define BMW_STEERING_COMMAND     0x22EU
#define BMW_STEERING_STATUS      0x22FU
#define BMW_CRUISE_GAS           0x200U
#define BMW_PEDAL_STATUS         0x199U
#define BMW_WHEEL_SPEEDS         0xCEU
#define BMW_STEERING_WHEEL_ANGLE 0xC4U
#define BMW_CRUISE_STALK         0x194U
#define BMW_VCU_ABS_DSC          0xA8U

#define BMW_STEER_MODE_OFF       0U
#define BMW_STEER_MODE_TORQUE    1U
#define BMW_STEER_MODE_ANGLE     2U
#define BMW_STEER_MODE_SOFTOFF   3U

#define BMW_GAS_ENABLE_MAGIC     0xA5U
#define BMW_BRAKE_PRESSED        0x64U

#define BMW_PEDAL_RX_TIMEOUT_US      200000U
#define BMW_STEER_STATUS_TIMEOUT_US  50000U

static uint8_t bmw_crc_lut[256];
static uint8_t bmw_tx_counter_22e;
static uint8_t bmw_tx_counter_200;
static bool bmw_tx_counter_22e_seen;
static bool bmw_tx_counter_200_seen;
static uint32_t bmw_ts_199;
static uint32_t bmw_ts_22f;
static bool bmw_seen_199;
static bool bmw_seen_22f;
static bool bmw_virtual_cruise;

// BMW 8-bit sum-fold: init=addr, add bytes with checksum byte 0, (s&0xFF)+(s>>8).
static uint8_t bmw_sum_fold_checksum(const CANPacket_t *msg) {
  uint16_t checksum = (uint16_t)msg->addr;
  const int len = GET_LEN(msg);
  for (int i = 0; i < len; i++) {
    const uint8_t b = (i == 0) ? 0U : msg->data[i];
    checksum += (uint16_t)b;
  }
  checksum = (uint16_t)((checksum & 0xFFU) + (checksum >> 8U));
  return (uint8_t)(checksum & 0xFFU);
}

// Wim 0x200 CRC8: poly 0xD5, init 0xFF, FORWARD over data[0..2]. Not toyota crc8_pedal.
static uint8_t bmw_compute_gas_crc(const CANPacket_t *msg) {
  uint8_t crc = 0xFFU;
  for (int i = 0; i < 3; i++) {
    crc ^= msg->data[i];
    crc = bmw_crc_lut[crc];
  }
  return crc;
}

static uint32_t bmw_get_checksum(const CANPacket_t *msg) {
  return msg->data[0];
}

static uint32_t bmw_compute_checksum(const CANPacket_t *msg) {
  return bmw_sum_fold_checksum(msg);
}

static uint8_t bmw_get_counter(const CANPacket_t *msg) {
  return msg->data[1] & 0x0FU;
}

static void bmw_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    if (msg->addr == BMW_WHEEL_SPEEDS) {
      int rl = ((int)msg->data[5] << 8) | (int)msg->data[4];
      int rr = ((int)msg->data[7] << 8) | (int)msg->data[6];
      rl = to_signed(rl, 16);
      rr = to_signed(rr, 16);
      const int avg = (rl + rr) / 2;
      const int avg_abs = SAFETY_ABS(avg);
      vehicle_moving = avg_abs > 0;
      UPDATE_VEHICLE_SPEED(((float)avg_abs) * 0.0625f * KPH_TO_MS);
    }

    if (msg->addr == BMW_STEERING_STATUS) {
      const int torque_meas_new = to_signed(msg->data[2], 8);
      update_sample(&torque_meas, torque_meas_new);
      torque_meas.min--;
      torque_meas.max++;
      bmw_ts_22f = microsecond_timer_get();
      bmw_seen_22f = true;
    }

    if (msg->addr == BMW_PEDAL_STATUS) {
      const int pedal_in = (int)msg->data[0];
      const int pedal_out = (int)msg->data[1];
      gas_pressed = (pedal_in > 20) && (pedal_in > (pedal_out + 5));
      bmw_ts_199 = microsecond_timer_get();
      bmw_seen_199 = true;
    }

    if (msg->addr == BMW_VCU_ABS_DSC) {
      brake_pressed = (msg->data[7] == BMW_BRAKE_PRESSED);
    }

    // 0x194 is UI buttons only. Never TX. Latch virtual cruise for panda
    // controls_allowed: plus1/resume engage, cancel disengages, idle holds.
    if (msg->addr == BMW_CRUISE_STALK) {
      const bool plus1 = (msg->data[2] & 0x01U) != 0U;
      const bool resume = (msg->data[2] & 0x40U) != 0U;
      const bool cancel = (msg->data[2] & 0x90U) != 0U;
      if (cancel) {
        bmw_virtual_cruise = false;
      } else if (plus1 || resume) {
        bmw_virtual_cruise = true;
      } else {
      }
      pcm_cruise_check(bmw_virtual_cruise);
      acc_main_on = bmw_virtual_cruise;
    }
  }
}

static bool bmw_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits BMW_STEERING_LIMITS = {
    .max_torque = 96,        // 12 Nm at 0.125 Nm/bit
    .max_rate_up = 1,        // 0.125 Nm / 10 ms = 12.5 Nm/s (8-bit floor on 10 Nm/s)
    .max_rate_down = 2,
    .max_rt_delta = 25,      // 12.5 Nm/s over 250 ms
    .max_torque_error = 16,  // 2 Nm meas vs cmd
    .type = TorqueMotorLimited,
  };

  bool tx = true;

  if (msg->addr == BMW_STEERING_COMMAND) {
    bool violation = bmw_sum_fold_checksum(msg) != msg->data[0];

    const uint8_t counter = msg->data[1] & 0x0FU;
    if (!violation) {
      if (bmw_tx_counter_22e_seen && (counter != ((bmw_tx_counter_22e + 1U) & 0x0FU))) {
        violation = true;
      } else {
        bmw_tx_counter_22e = counter;
        bmw_tx_counter_22e_seen = true;
      }
    }

    const uint8_t steer_mode = (msg->data[1] >> 4) & 0x03U;
    if (steer_mode == BMW_STEER_MODE_ANGLE) {
      violation = true;
    }

    const int desired_torque = to_signed(msg->data[4], 8);
    const int steer_req = (steer_mode == BMW_STEER_MODE_TORQUE) ? 1 : 0;

    if ((steer_req != 0) || (desired_torque != 0)) {
      if (bmw_seen_22f) {
        const uint32_t elapsed = safety_get_ts_elapsed(microsecond_timer_get(), bmw_ts_22f);
        if (elapsed > BMW_STEER_STATUS_TIMEOUT_US) {
          violation = true;
        }
      }
    }

    if (!violation) {
      violation |= steer_torque_cmd_checks(desired_torque, steer_req, BMW_STEERING_LIMITS);
    }

    if (violation) {
      tx = false;
    }
  }

  // Do not call longitudinal_interceptor_checks(): Wim byte1 is enable 0xA5, not gas.
  if (msg->addr == BMW_CRUISE_GAS) {
    bool violation = false;

    if (!get_longitudinal_allowed()) {
      violation = true;
    }

    if (bmw_seen_199) {
      const uint32_t elapsed = safety_get_ts_elapsed(microsecond_timer_get(), bmw_ts_199);
      if (elapsed > BMW_PEDAL_RX_TIMEOUT_US) {
        violation = true;
      }
    }

    if (msg->data[1] != BMW_GAS_ENABLE_MAGIC) {
      violation = true;
    }

    if (bmw_compute_gas_crc(msg) != msg->data[3]) {
      violation = true;
    } else {
      const uint8_t counter = msg->data[2] & 0x0FU;
      if (bmw_tx_counter_200_seen && (counter != ((bmw_tx_counter_200 + 1U) & 0x0FU))) {
        violation = true;
      } else {
        bmw_tx_counter_200 = counter;
        bmw_tx_counter_200_seen = true;
      }
    }

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static safety_config bmw_init(uint16_t param) {
  static const CanMsg BMW_TX_MSGS[] = {
    {BMW_STEERING_COMMAND, 0, 5, .check_relay = true},   // STEERING_COMMAND actuator
    {BMW_CRUISE_GAS, 0, 4, .check_relay = false},         // CRUISE_Gas interceptor
  };

  static RxCheck bmw_rx_checks[] = {
    {.msg = {{BMW_PEDAL_STATUS, 0, 3, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_WHEEL_SPEEDS, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_STEERING_WHEEL_ANGLE, 0, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_STEERING_STATUS, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_CRUISE_STALK, 0, 4, 5U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_VCU_ABS_DSC, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  SAFETY_UNUSED(param);

  bmw_tx_counter_22e = 0U;
  bmw_tx_counter_200 = 0U;
  bmw_tx_counter_22e_seen = false;
  bmw_tx_counter_200_seen = false;
  bmw_ts_199 = 0U;
  bmw_ts_22f = 0U;
  bmw_seen_199 = false;
  bmw_seen_22f = false;
  bmw_virtual_cruise = false;

  gen_crc_lookup_table_8(0xD5U, bmw_crc_lut);

  return BUILD_SAFETY_CFG(bmw_rx_checks, BMW_TX_MSGS);
}

const safety_hooks bmw_hooks = {
  .init = bmw_init,
  .rx = bmw_rx_hook,
  .tx = bmw_tx_hook,
  .get_checksum = bmw_get_checksum,
  .compute_checksum = bmw_compute_checksum,
  .get_counter = bmw_get_counter,
};
