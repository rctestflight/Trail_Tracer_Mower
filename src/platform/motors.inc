// The motor controllers (ODrive) on the CAN bus, and the servo pulse outputs.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

// Native ESP32-S2 TWAI controller connected to a 3.3V SN65HVD230 transceiver.
#define TWAI_TX_PIN GPIO_NUM_36
#define TWAI_RX_PIN GPIO_NUM_38
#define CAN_BUS_BITRATE 1000000  // Must match every connected ODrive CAN baud rate.

// ODrive CAN protocol command IDs (see ODrive CAN protocol docs)
enum OdriveCanCmd : uint8_t {
  ODRIVE_CMD_HEARTBEAT              = 0x001,
  ODRIVE_CMD_ESTOP                  = 0x002,
  ODRIVE_CMD_GET_MOTOR_ERROR        = 0x003,
  ODRIVE_CMD_GET_ERROR              = 0x003, // Micro firmware: active errors + disarm reason.
  ODRIVE_CMD_GET_ENCODER_ERROR      = 0x004,
  ODRIVE_CMD_SET_AXIS_STATE         = 0x007,
  ODRIVE_CMD_GET_ENCODER_ESTIMATES  = 0x009,
  ODRIVE_CMD_SET_CONTROLLER_MODE    = 0x00B,
  ODRIVE_CMD_SET_INPUT_POS          = 0x00C,
  ODRIVE_CMD_SET_INPUT_VEL          = 0x00D,
  ODRIVE_CMD_SET_INPUT_TORQUE       = 0x00E,
  ODRIVE_CMD_SET_LIMITS             = 0x00F,
  ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT = 0x017,
  ODRIVE_CMD_CLEAR_ERRORS           = 0x018,
};

bool can_driver_installed = false;
bool can_init_ok = false;  // Peripheral running, not proof of an ODrive connection.

uint32_t odrive_axis_errors[ODRIVE_NODE_COUNT] = {};
bool odrive_heartbeat_received[ODRIVE_NODE_COUNT] = {};
uint32_t odrive_heartbeat_millis[ODRIVE_NODE_COUNT] = {};
uint8_t odrive_axis_states[ODRIVE_NODE_COUNT] = {};
uint8_t odrive_status_byte5[ODRIVE_NODE_COUNT] = {};
uint8_t odrive_status_byte6[ODRIVE_NODE_COUNT] = {};
uint32_t odrive_seen_errors[ODRIVE_NODE_COUNT] = {}; // Retain faults even after they clear.
uint32_t odrive_active_errors[ODRIVE_NODE_COUNT] = {};
uint32_t odrive_disarm_reasons[ODRIVE_NODE_COUNT] = {};
bool odrive_error_received[ODRIVE_NODE_COUNT] = {};
uint32_t odrive_error_millis[ODRIVE_NODE_COUNT] = {};
float odrive_node0_vbus = 0.0f;
float odrive_node0_ibus = 0.0f;
bool odrive_node0_ibus_received = false;
bool odrive_node0_vbus_received = false;
uint32_t odrive_node0_vbus_millis = 0;

uint32_t odrive_encoder_poll_millis = 0;  // last time encoder estimates were requested
// Wheel speeds for the telemetry log: Get_Encoder_Estimates (pos, vel) from nodes 0 and 1, one
// per side. ODrive firmware 0.6 sends them by itself every 10 ms by default; a node that has
// been silent for ODRIVE_VEL_STALE_MS is asked by RTR, at most one request each 10 ms.
float odrive_vel_rps[ODRIVE_NODE_COUNT] = {};
float odrive_pos_rev[ODRIVE_NODE_COUNT] = {};  // encoder position, rev (Get_Encoder_Estimates; log layout 3)
uint32_t odrive_vel_millis[ODRIVE_NODE_COUNT] = {};
bool odrive_vel_received[ODRIVE_NODE_COUNT] = {};
#define ODRIVE_ENCODER_POLL_MS 500  // encoder estimate poll interval

// Install and start native TWAI in normal mode; no external CAN controller.
bool odriveCan_init() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  // UART/RMT/GPIO consume level-1 interrupts on the single-core ESP32-S2.
  // Use a dedicated level-2 interrupt; IRAM requires a different SDK build.
  general.intr_flags = ESP_INTR_FLAG_LEVEL2;
  general.tx_queue_len = 16;
  general.rx_queue_len = 128;  // Buffer bursts while the application handles STM32 data.
  general.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_RX_QUEUE_FULL;
#if CAN_BUS_BITRATE == 1000000
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
#elif CAN_BUS_BITRATE == 250000
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_250KBITS();
#else
#error "Select a TWAI timing configuration matching CAN_BUS_BITRATE"
#endif
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  /* usbOut.println("Boot: TWAI driver install (level-2 interrupt)"); */;
  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  /* usbOut.printf("Boot: TWAI install returned %s\n", esp_err_to_name(err)); */;
  if (err != ESP_OK) {
    /* usbOut.printf("TWAI install FAILED: %s\n", esp_err_to_name(err)); */;
    return false;
  }
  can_driver_installed = true;
  /* usbOut.println("Boot: TWAI driver start"); */;
  err = twai_start();
  if (err != ESP_OK) {
    /* usbOut.printf("TWAI start FAILED: %s\n", esp_err_to_name(err)); */;
    twai_driver_uninstall();
    can_driver_installed = false;
    return false;
  }
  /* usbOut.printf("TWAI init OK: TX=%d RX=%d bitrate=%u normal mode\n",
                (int)TWAI_TX_PIN, (int)TWAI_RX_PIN, (unsigned)CAN_BUS_BITRATE); */;
  return true;
}

// Success means queued for transmission, not acknowledged by the ODrive.
// Single-shot prevents unacknowledged frames from retrying indefinitely.
bool sendODriveCAN(uint8_t nodeID, uint8_t commandID,
                   const uint8_t *data, uint8_t length, bool remote = false) {
  if (nodeID > 0x3F || commandID > 0x1F || length > 8 ||
      (!remote && length && !data) || !can_init_ok) return false;
  twai_status_info_t status;
#ifdef STEP4
  // STEP4: one bus state read per batch of sends in the same millisecond (the frame's 6 or 12 sends), not one per send
  static uint32_t state_ms = 0;
  static bool state_ok = false;
  if (!state_ok || millis() != state_ms) {
    state_ms = millis();
    state_ok = twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_RUNNING;
  }
  if (!state_ok) return false;
#else
  if (twai_get_status_info(&status) != ESP_OK ||
      status.state != TWAI_STATE_RUNNING) return false;
#endif
  twai_message_t frame = {};
  frame.identifier = (static_cast<uint32_t>(nodeID) << 5) | commandID;
  frame.data_length_code = length;
  frame.rtr = remote;
  frame.ss = 1;
  if (!remote && length) memcpy(frame.data, data, length);
  // Zero timeout: a full queue fails immediately, preserving loop responsiveness.
  esp_err_t err = twai_transmit(&frame, 0);
  if (err != ESP_OK) {
    static uint32_t last_report = 0;
    if (millis() - last_report >= 1000) {
      last_report = millis();
      twai_get_status_info(&status);
      /* usbOut.printf("TWAI TX queue failed: id=0x%03lX %s pending=%lu RXpin=%d bus_errors=%lu arb_lost=%lu\n",
                    (unsigned long)frame.identifier, esp_err_to_name(err),
                    (unsigned long)status.msgs_to_tx, gpio_get_level(TWAI_RX_PIN),
                    (unsigned long)status.bus_error_count,
                    (unsigned long)status.arb_lost_count); */;
    }
  }
  return err == ESP_OK;
}

// Exposes arbitration ID, DLC and payload in the native TWAI message.
bool receiveODriveCAN(twai_message_t &frame) {
  return can_init_ok && twai_receive(&frame, 0) == ESP_OK;
}

// Commands an ODrive axis into the given AXIS_STATE (see ODrive Enums.AxisState)
// Match nano_temp.ino: the axis-state payload is one uint32_t (four bytes).
void odrive_setAxisState(uint8_t node_id, uint32_t axis_state) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_SET_AXIS_STATE;
  frame.data_length_code = 4;
  memset(frame.data, 0, sizeof(frame.data));
  memcpy(frame.data, &axis_state, sizeof(axis_state));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Sends a velocity setpoint (turns/s) with optional torque feed-forward (Nm) to an ODrive axis
void odrive_setInputVel(uint8_t node_id, float vel, float torque_ff) {  // torque_ff: 0 if left out (rx_platform.h)
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_SET_INPUT_VEL;
  frame.data_length_code = 8;
  memcpy(&frame.data[0], &vel, sizeof(vel));
  memcpy(&frame.data[4], &torque_ff, sizeof(torque_ff));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Sets an ODrive axis velocity limit [rev/s] and current limit [A]
void odrive_setLimits(uint8_t node_id, float vel_limit, float current_limit) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_SET_LIMITS;
  frame.data_length_code = 8;
  memcpy(&frame.data[0], &vel_limit, sizeof(vel_limit));
  memcpy(&frame.data[4], &current_limit, sizeof(current_limit));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Sets an ODrive axis control mode and input mode (see ODrive ControlMode/InputMode enums)
void odrive_setControllerMode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_SET_CONTROLLER_MODE;
  frame.data_length_code = 8;
  memcpy(&frame.data[0], &control_mode, sizeof(control_mode));
  memcpy(&frame.data[4], &input_mode, sizeof(input_mode));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Clears all errors on an ODrive axis (and re-arms the brake resistor etc.)
// Match nano_temp.ino's zero-length Clear_Errors command.
void odrive_clearErrors(uint8_t node_id) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_CLEAR_ERRORS;
  frame.data_length_code = 0;
  memset(frame.data, 0, sizeof(frame.data));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Recovery runs asynchronously: initiate once, poll state, restart after recovery.
void odrive_recoverIfNeeded() {
  if (!can_driver_installed) return;
  uint32_t alerts = 0;
  twai_read_alerts(&alerts, 0);
  static uint32_t last_report = 0;
  if (alerts && millis() - last_report >= 1000) {
    last_report = millis();
    /* usbOut.printf("TWAI alerts: 0x%08lX\n", (unsigned long)alerts); */;
  }
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;
  can_init_ok = status.state == TWAI_STATE_RUNNING;
  if (status.state == TWAI_STATE_BUS_OFF) {
    esp_err_t err = twai_initiate_recovery();  // Clears pending TX; never waits for recovery.
    if (err == ESP_OK) /* usbOut.println("TWAI bus-off: recovery started"); */;
  } else if (status.state == TWAI_STATE_STOPPED) {
    esp_err_t err = twai_start();  // Recovery completes in the stopped state.
    can_init_ok = err == ESP_OK;
    if (can_init_ok) /* usbOut.println("TWAI recovered: normal mode restarted"); */;
  }
}

// Requests encoder estimates from one ODrive node (cycles through nodes each call).
// The ODrive responds with a Get_Encoder_Estimates message.
void odrive_requestEncoderEstimates() {
  if (!can_init_ok) return;
  // Wheel speeds of nodes 0 and 1 for the telemetry log: ask only a node whose own messages stopped.
  static uint32_t vel_poll_ms = 0;
  static uint8_t vel_node = 0;
  if (millis() - vel_poll_ms >= 10) {
    vel_poll_ms = millis();
    vel_node ^= 1;
    if (millis() - odrive_vel_millis[vel_node] > ODRIVE_VEL_STALE_MS)
      sendODriveCAN(vel_node, ODRIVE_CMD_GET_ENCODER_ESTIMATES, nullptr, 8, true);
  }
  if (millis() - odrive_encoder_poll_millis < ODRIVE_ENCODER_POLL_MS) return;

  static uint8_t poll_node = 0;
  twai_message_t frame = {};
  frame.identifier = (poll_node << 5) | ODRIVE_CMD_GET_ENCODER_ESTIMATES;
  frame.data_length_code = 8;  // RTR requests the eight-byte response; no data is transmitted.
  memset(frame.data, 0, sizeof(frame.data));
  bool queued = sendODriveCAN(poll_node, ODRIVE_CMD_GET_ENCODER_ESTIMATES, nullptr,
                              frame.data_length_code, true);
  // Request node 0 bus voltage alongside the periodic encoder poll.
  sendODriveCAN(0, ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT, nullptr, 8, true);
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    sendODriveCAN(node, ODRIVE_CMD_GET_ERROR, nullptr, 8, true);
  }
  if (poll_node == 5) {
    /* usbOut.printf("ODrive node 5 encoder request: id=0x0A9 RTR=1 DLC=8 %s\n",
                  queued ? "queued (delivery not confirmed)" : "queue failed"); */;
  }

  poll_node++;
  if (poll_node >= ODRIVE_NODE_COUNT) poll_node = 0;
  odrive_encoder_poll_millis = millis();
}

// Drains CAN RX and reports new faults and closed-loop-to-idle transitions.
void odrive_printErrorNames(uint32_t errors);
void odrive_pollAndPrintCan() {
  if (!can_init_ok) return;
  twai_message_t frame = {};
  const uint32_t started_us = micros();
  // Drain bursts with bounded work; inspect every received frame for events.
  for (uint8_t received = 0; received < 64 &&
       micros() - started_us < 2000 && receiveODriveCAN(frame); received++) {
    if (frame.extd || frame.rtr || frame.data_length_code > 8) continue;
    uint8_t node_id = frame.identifier >> 5;
    uint8_t cmd_id = frame.identifier & 0x1F;
    if (node_id >= ODRIVE_NODE_COUNT) continue;
    if (cmd_id == ODRIVE_CMD_HEARTBEAT && frame.data_length_code >= 7) {
      const uint32_t previous_errors = odrive_axis_errors[node_id];
      const bool went_idle = odrive_heartbeat_received[node_id] &&
        odrive_axis_states[node_id] == ODRIVE_AXIS_STATE_CLOSED_LOOP &&
        frame.data[4] == ODRIVE_AXIS_STATE_IDLE;
      memcpy(&odrive_axis_errors[node_id], frame.data, sizeof(uint32_t));
      const uint32_t new_errors = odrive_axis_errors[node_id] & ~previous_errors;
      if (new_errors || went_idle) {
        usbOut.printf("ODrive N%u: %s error=0x%08lX procedure=%u status6=%u", node_id,
          went_idle ? "CLOSED_LOOP -> IDLE" : "axis fault",
          (unsigned long)odrive_axis_errors[node_id], frame.data[5], frame.data[6]);
        odrive_printErrorNames(new_errors);
        usbOut.println();
      }
      odrive_seen_errors[node_id] |= odrive_axis_errors[node_id];
      odrive_axis_states[node_id] = frame.data[4];
      // Preserve raw bytes: newer firmware uses procedure result/trajectory done;
      // older firmware can use motor/encoder/controller flags here.
      odrive_status_byte5[node_id] = frame.data[5];
      odrive_status_byte6[node_id] = frame.data[6];
      odrive_heartbeat_received[node_id] = true;
      odrive_heartbeat_millis[node_id] = millis();
    }
    if (cmd_id == ODRIVE_CMD_GET_ERROR && frame.data_length_code == 8) {
      const uint32_t previous_active = odrive_active_errors[node_id];
      const uint32_t previous_disarm = odrive_disarm_reasons[node_id];
      memcpy(&odrive_active_errors[node_id], frame.data, sizeof(uint32_t));
      memcpy(&odrive_disarm_reasons[node_id], &frame.data[4], sizeof(uint32_t));
      odrive_seen_errors[node_id] |= odrive_active_errors[node_id] | odrive_disarm_reasons[node_id];
      const uint32_t new_active = odrive_active_errors[node_id] & ~previous_active;
      const uint32_t new_disarm = odrive_disarm_reasons[node_id] & ~previous_disarm;
      if (new_active || new_disarm) {
        usbOut.printf("ODrive N%u: active=0x%08lX disarm=0x%08lX", node_id,
          (unsigned long)odrive_active_errors[node_id],
          (unsigned long)odrive_disarm_reasons[node_id]);
        odrive_printErrorNames(new_active | new_disarm);
        usbOut.println();
      }
      odrive_error_received[node_id] = true;
      odrive_error_millis[node_id] = millis();
    }
    if (cmd_id == ODRIVE_CMD_GET_ENCODER_ESTIMATES && frame.data_length_code == 8) {
      memcpy(&odrive_vel_rps[node_id], &frame.data[4], sizeof(float));  // pos [rev] at 0, vel [rev/s] at 4
      memcpy(&odrive_pos_rev[node_id], &frame.data[0], sizeof(float));
      odrive_vel_millis[node_id] = millis();
      odrive_vel_received[node_id] = true;
    }
    // Keep bus telemetry cached without printing periodic updates.
    if (node_id == 0 && cmd_id == ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT &&
        frame.data_length_code >= 4) {
      memcpy(&odrive_node0_vbus, frame.data, sizeof(odrive_node0_vbus));
      odrive_node0_vbus_received = true;
      odrive_node0_vbus_millis = millis();
      if (frame.data_length_code == 8) {
        memcpy(&odrive_node0_ibus, &frame.data[4], sizeof(odrive_node0_ibus));
        odrive_node0_ibus_received = true;
      } else {
        odrive_node0_ibus_received = false;
      }
    }
  }
}

void odrive_printErrorNames(uint32_t errors) {
  // Common Micro errors; all bits are also printed in hex for other faults.
  static const struct { uint32_t bit; const char* name; } names[] = {
    {0x20, "DRV_FAULT"}, {0x100, "DC_BUS_OVER_VOLTAGE"},
    {0x200, "DC_BUS_UNDER_VOLTAGE"}, {0x400, "DC_BUS_OVER_CURRENT"},
    {0x800, "DC_BUS_OVER_REGEN_CURRENT"}, {0x1000, "CURRENT_LIMIT_VIOLATION"},
    {0x2000, "MOTOR_OVER_TEMP"}, {0x4000, "INVERTER_OVER_TEMP"},
    {0x8000, "VELOCITY_LIMIT_VIOLATION"}, {0x1000000, "WATCHDOG_TIMER_EXPIRED"},
    {0x2000000, "ESTOP_REQUESTED"}, {0x4000000, "SPINOUT_DETECTED"}
  };
  for (const auto& entry : names) {
    if (errors & entry.bit) usbOut.printf(" %s", entry.name);
  }
}

// write a PWM output channel's on-time in microseconds
uint32_t pwm_test_us = 0;  // 0.5.104: bench "pwm <us>": OUT1 held at this width while not driving (0 = off)
void writePwmMicros(uint8_t ch, uint32_t micros){
  if (pwm_test_us && ch == 1 && drive_mode_status != 2) micros = pwm_test_us;
  uint16_t duty_cycle = (int32_t)((float)micros * 1e-6 * (float)(PWM_OUT_FREQUENCY * ((1 << PWM_OUT_RESOLUTION_BITS) - 1)));
  ledcWrite(ch, duty_cycle);
}

