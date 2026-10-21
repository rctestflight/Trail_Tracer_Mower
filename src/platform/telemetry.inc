// The telemetry log: 100 Hz samples, status record, events, the dual tone items, the bin magnitudes.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

uint32_t telem_loop_max_us = 0;  // longest packet handling since the last status record
#ifndef DT_SIGN_PLUS_IS_LEFT
#define DT_SIGN_PLUS_IS_LEFT 1  // 0.6.13: over the wire, item 9 = +1 means Side A lies to the vehicle's left (0: to its right)
#endif
int32_t dt_decoded = -1;     // 0.6.12: the last CID 50 index 0 read (bit mask of decoded Tx channels); -1 none
uint32_t dt_decoded_ms = 0;  // when it came
// Telemetry: the coil magnitudes of all 8 bins of both sensors (body 0x2E, lib/rover_telemetry), for the
// per-bin plots of the telemetry app. The log's anti-alias filter (decimator.h, two moving averages) at the
// rate telemetry_bins_hz (0 = off), on |v| and on v of each coil: the record holds the filtered |v| with the
// sign of the filtered v (as the STMCOIL8 line). One FAST record per output. The filter is on the heap.
extern bool safe_mode;
rec::ChannelDecimator* telem_bins_decim = nullptr;
void telemetryBins(){
  const int hz = g_deviceConfig.telemetry_bins_hz;
  if (safe_mode || !g_deviceConfig.telemetry_enabled || hz <= 0) return;
  if (!telem_bins_decim) {
    telem_bins_decim = new rec::ChannelDecimator(2 * WFRX_BINS * 2 * 2);
    telem_bins_decim->setRate(hz);
  }
  if (telem_bins_decim->rate() != hz) telem_bins_decim->setRate(hz);
  float in[2 * WFRX_BINS * 2 * 2] = {};  // [sensor][bin][coil][|v|, v]
  bool seen[2] = {false, false};
  for (unsigned k = 0; k < WFRX_RESULTS; k++) {
    const unsigned sn = (stmDec.lanes >> k) & 1u;  // the sensor this lane reads; both lanes on one sensor: lane 1
    if (seen[sn]) continue;
    seen[sn] = true;
    for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
      const int32_t v[2] = {wfrx_int24_get(stmDec.coil[k][bn].v1), wfrx_int24_get(stmDec.coil[k][bn].v2)};
      for (unsigned c = 0; c < 2; c++) {
        float* p = in + ((sn * WFRX_BINS + bn) * 2 + c) * 2;
        p[0] = (float)(v[c] < 0 ? -v[c] : v[c]);
        p[1] = (float)v[c];
      }
    }
  }
  float out[2 * WFRX_BINS * 2 * 2];
  uint32_t out_ms;
  if (!telem_bins_decim->push(in, millis(), out, &out_ms)) return;
  int32_t v[WFRX_BINS][4];
  for (unsigned bn = 0; bn < WFRX_BINS; bn++)
    for (unsigned sn = 0; sn < 2; sn++)
      for (unsigned c = 0; c < 2; c++) {
        const float* p = out + ((sn * WFRX_BINS + bn) * 2 + c) * 2;
        const int32_t m = (int32_t)lroundf(p[0]);
        v[bn][sn * 2 + c] = !seen[sn] || stm_bin_hz[bn] <= 0 ? rec::BINS_NONE : p[1] < 0 ? -m : m;
      }
  uint8_t body[rec::BINS_BYTES];
  const int f = telem_bins_decim->factor();
  const int n = rec::packBins(body, (uint8_t)(f < 255 ? f : 255), (uint16_t)(1000 / telem_bins_decim->rate()), stm_bin_hz, v);
  telemlog_add(TELEMLOG_FAST, out_ms, body, (size_t)n);
}
// 0.6.11: the dual tone polarity items of the Rx STM32 extension slot (2.1.134+) into the telemetry table: id 9 the sign
// (+1, -1, 0 none yet), id 10 the spread of the sign angle (deg x 10), per sensor (index bit 4) and Tx channel (bits 0-3,
// 0-4). An item older than 1 s, or never seen, gives no value.
void dtTelemetry() {
  static const tlm_id SIGN[2][5] = {
    {TLM_dt_sign_s1_ch1, TLM_dt_sign_s1_ch2, TLM_dt_sign_s1_ch3, TLM_dt_sign_s1_ch4, TLM_dt_sign_s1_ch5},
    {TLM_dt_sign_s2_ch1, TLM_dt_sign_s2_ch2, TLM_dt_sign_s2_ch3, TLM_dt_sign_s2_ch4, TLM_dt_sign_s2_ch5}};
  static const tlm_id SPREAD[2][5] = {
    {TLM_dt_spread_s1_ch1, TLM_dt_spread_s1_ch2, TLM_dt_spread_s1_ch3, TLM_dt_spread_s1_ch4, TLM_dt_spread_s1_ch5},
    {TLM_dt_spread_s2_ch1, TLM_dt_spread_s2_ch2, TLM_dt_spread_s2_ch3, TLM_dt_spread_s2_ch4, TLM_dt_spread_s2_ch5}};
  static const tlm_id POL[2][5] = {
    {TLM_dt_polarity_s1_ch1, TLM_dt_polarity_s1_ch2, TLM_dt_polarity_s1_ch3, TLM_dt_polarity_s1_ch4, TLM_dt_polarity_s1_ch5},
    {TLM_dt_polarity_s2_ch1, TLM_dt_polarity_s2_ch2, TLM_dt_polarity_s2_ch3, TLM_dt_polarity_s2_ch4, TLM_dt_polarity_s2_ch5}};
  const uint16_t now10 = (uint16_t)(millis() / 10u);
  for (unsigned s = 0; s < 2; s++)
    for (unsigned c = 0; c < 5; c++) {
      const unsigned sl = s * 8u + c;
      const uint16_t ts = stmDec.ext_t10[8][sl], tp = stmDec.ext_t10[9][sl];
      const bool sign_ok = ts && (uint16_t)(now10 - ts) <= 100u, spread_ok = tp && (uint16_t)(now10 - tp) <= 100u;
      tlm_set(SIGN[s][c], sign_ok ? (float)stmDec.ext_val[8][sl] : NAN);
      tlm_set(SPREAD[s][c], spread_ok ? stmDec.ext_val[9][sl] * 0.1f : NAN);
      // 0.6.21: polarity, ext id 12 (Rx STM32 2.2.0+): no value when no id 12 item came in the last 1 s.
      const uint16_t tp12 = stmDec.pol_t10;
      tlm_set(POL[s][c], (tp12 && (uint16_t)(now10 - tp12) <= 100u) ? (float)stmDec.pol[s][c] : NAN);
    }
  tlm_set(TLM_wire_inside, (float)world_inside);
  tlm_set(TLM_wire_heading, (float)world_heading);
  tlm_set(TLM_dt_decoded, (dt_decoded >= 0 && millis() - dt_decoded_ms < 5000) ? (float)(dt_decoded & 0xFF) : NAN);

  // 0.6.13: the direction round the perimeter in follow mode (drive mode 2), seen from above: +1 clockwise, -1
  // counterclockwise, 0 not known; no value outside follow mode. Inputs: the item 9 sign of the followed Tx channel
  // (rx_channel) at the sensor nearest the wire, and the user setting "Side A of Tx channel N" (1 inside, 2 outside).
  // Over the wire the sign flips when the vehicle turns round (from the sensor geometry), so it says on which
  // side of the vehicle Side A lies. Inside to the vehicle's left = counterclockwise. A sign of 0 holds the last value.
  static int8_t follow_dir_held = 0;
  if (drive_mode_status != 2) {
    follow_dir_held = 0;
    tlm_set(TLM_follow_dir, NAN);
  } else {
    const int c = (int)rx_channel - 1;
    const uint8_t sides[5] = {g_deviceConfig.dt_side_a_ch1, g_deviceConfig.dt_side_a_ch2, g_deviceConfig.dt_side_a_ch3,
                              g_deviceConfig.dt_side_a_ch4, g_deviceConfig.dt_side_a_ch5};
    const uint8_t side_a = (c >= 0 && c < 5) ? sides[c] : 0;
    const float r1 = stmPacket.items.s1_radius, r2 = stmPacket.items.s2_radius;
    const unsigned s = (r2 > 0.0f && (r1 <= 0.0f || r2 < r1)) ? 1u : 0u;  // the sensor nearest the wire
    int sign = 0;
    if (c >= 0 && c < 5) {
      const unsigned sl = s * 8u + (unsigned)c;
      const uint16_t ts = stmDec.ext_t10[8][sl];
      if (ts && (uint16_t)(now10 - ts) <= 100u) sign = stmDec.ext_val[8][sl];
    }
    const uint8_t dirs[5] = {g_deviceConfig.tx_dir_ch1, g_deviceConfig.tx_dir_ch2, g_deviceConfig.tx_dir_ch3,
                             g_deviceConfig.tx_dir_ch4, g_deviceConfig.tx_dir_ch5};
    if (c >= 0 && c < 5 && dirs[c] != 0) {
      follow_dir_held = world_heading;  // 0.6.21: from the polarity and the current direction setting (worldFacts)
    } else if (side_a != 1 && side_a != 2) {
      follow_dir_held = 0;
    } else if (sign != 0) {
      const bool a_left = (sign > 0) == (DT_SIGN_PLUS_IS_LEFT != 0);
      const bool inside_left = a_left == (side_a == 1);
      follow_dir_held = inside_left ? -1 : 1;
    }
    tlm_set(TLM_follow_dir, (float)follow_dir_held);
  }
}

// 0.6.12: the Tx channels the Rx STM32 decodes now (CID 50 read, index 0): each 1 s while it answers, else each 10 s
// (an app before 2.1.134 refuses the read, so it costs one refused command each 10 s).
void dtPollService() {
  static uint32_t last = 0;
  if (stm_link_ver != 2) return;
  const uint32_t now = millis();
  const bool answers = dt_decoded_ms && now - dt_decoded_ms < 5000;
  if (now - last < (answers ? 1000u : 10000u)) return;
  last = now;
  stmLink.push(50, 0, 0);
}

// Telemetry log ---------------------------------------------------------------------------------
// docs/espnow_protocol.md, "Telemetry log". lib/telemlog numbers and stores the records and does
// the radio work; lib/rover_telemetry packs them. Each STM32 packet gives one 100 Hz sample (7 a
// record, stream 0); each second gives one status record (stream 1), plus event records on
// changes. Nothing here waits: the log runs whether a collector listens or not.
#define FLASHTEST_SECTORS 16  // the last 16 sectors of tlog: flashtest only, never the telemetry ring
uint32_t telem_boot_id = 0;
rec::Block telem_block;
rec::Decimator* telem_decim = nullptr;  // from the heap in telemetryBegin()


// An erase stops the CPU for about 45 ms (the STM32 packets of that time are lost). Allowed only
// when nothing drives: docked, the wheels known to stand still, or no ODrive on the bus (bench).
bool telemetryEraseAllowed();
bool bench_moving = false;  // bench USB "moving on": the vehicle counts as moving (RAM only, off at each start)
bool vehicleStationary(){ return !bench_moving && telemetryEraseAllowed(); }
bool telemetryEraseAllowed(){
  if (charge_status > 0) return true;
  const uint32_t now = millis();
  bool any_odrive = false;
  for (int n = 0; n < ODRIVE_NODE_COUNT; n++)
    if (odrive_heartbeat_received[n] && now - odrive_heartbeat_millis[n] < 1000) any_odrive = true;
  if (!any_odrive) return true;
  const bool fresh = odrive_vel_received[0] && odrive_vel_received[1]
      && now - odrive_vel_millis[0] < ODRIVE_VEL_STALE_MS && now - odrive_vel_millis[1] < ODRIVE_VEL_STALE_MS;
  return fresh && fabsf(odrive_vel_rps[0]) < 0.02f && fabsf(odrive_vel_rps[1]) < 0.02f
      && fabsf(odrive_drive_command) < 0.01f && fabsf(odrive_steer_command) < 0.01f;
}
uint32_t telem_packets = 0;           // packets since the last status record
uint32_t telem_status_ms = 0;
int telem_last_mode = -1, telem_last_ap = -2, telem_last_charge = 99;
bool telem_last_stuck = false, telem_last_halted = false;
uint32_t telem_last_axis_errors[ODRIVE_NODE_COUNT] = {};
// USB "log dump": records go out paced by the USB buffer, so that no line is cut.
int telem_dump_stream = -1;
uint32_t telem_dump_boot = 0, telem_dump_seq = 0, telem_dump_left = 0;

uint8_t telemetryApByte(){
  if (!rover_active) return rec::AP_OFF;
  return (uint8_t)(((uint8_t)roverAp.state << 4) | ((uint8_t)roverAp.sub & 0x0F));
}

bool telemetrySend(const uint8_t mac[6], const uint8_t* buf, size_t len){
  if (sim_loss_tx && esp_random() % 100 < sim_loss_tx) { sim_dropped_tx++; return true; }  // bench loss simulation
  espnowEnsurePeer(mac);  // a send needs its destination (broadcast, or the requester) in the peer list
  return esp_now_send(mac, buf, len) == ESP_OK;
}

void telemetryEvent(uint16_t code, int32_t a, int32_t b, const char* text){
  if (safe_mode) return;  // the log is not started in safe mode
  uint8_t body[rec::EVENT_MAX];
  const int n = rec::packEvent(body, code, a, b, text);
  telemlog_add(TELEMLOG_SLOW, millis(), body, (size_t)n);
}

void telemetryBegin(){
  // boot_id: random, not a counter in flash (0.5.82: no repeating flash writes). It tells boots apart; the
  // previous boot after a warm reset is the RAM ring's previous segment (INFO prev_boot_id).
  do telem_boot_id = esp_random(); while (telem_boot_id == 0 || telem_boot_id == TELEMLOG_NO_SEQ);
  const esp_reset_reason_t rr = esp_reset_reason();
  const bool warm = rr != ESP_RST_POWERON && rr != ESP_RST_BROWNOUT && rr != ESP_RST_UNKNOWN;
  telemlog_init(telem_boot_id, warm);
  telem_decim = new rec::Decimator();  // about 400 B; heap, because static RAM is full
  telem_decim->setRate(g_deviceConfig.telemetry_rate_hz);
  tlm::init(telemlog_add);  // table values: "no value" until set; record buffers on the heap for each rate in use
  // Settings hash: CRC-32 over the "key=value" lines, as the "list" command prints them.
  uint32_t crc = 0;
  String kv;
  for (int i = 0; msc_fat_key_at(&g_deviceConfig, i, &kv); i++) crc = rec::crc32((const uint8_t*)kv.c_str(), kv.length(), crc);
  char text[64];
  snprintf(text, sizeof(text), "fw=%s cfg=%08lx", ROVER_FW_VERSION, (unsigned long)crc);
  telemlog_info_t li;
  telemlog_info(&li);
  telemetryEvent(rec::EV_BOOT, (int32_t)rr, (int32_t)li.prev_boot_id, text);  // b: previous boot_id (warm reset), else 0
}

void telemetrySample(){
  if (safe_mode) return;
  const uint32_t sb = stmPacket.items.status_bits;
  rec::Sample s;
  s.ms = millis();
  s.x1_mm = stmPacket.items.s1_x;
  s.y1_mm = stmPacket.items.s1_y;
  s.r1_mm = stmPacket.items.s1_radius;
  s.flags = (WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_LOST) ? rec::F_LOST : 0)
          | (WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_OVERLOAD) ? rec::F_OVERLOAD : 0)
          | (WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_DISABLED) ? rec::F_DISABLED : 0)
          | (WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_WEAK) ? rec::F_WEAK : 0)
          | (charge_status > 0 ? rec::F_CHARGING : 0);
  const float btx[3] = {(float)BTxThreshold + 1.0f, BTx1Signal, BTx2Signal};
  if (rover_entry_btx && btx[rover_entry_btx] < BTxThreshold) s.flags |= rec::F_ENTRY_BTX;
  if (rover_exit_btx && btx[rover_exit_btx] < BTxThreshold) s.flags |= rec::F_EXIT_BTX;
  const uint32_t now = millis();
  if (odrive_vel_received[0] && odrive_vel_received[1] && now - odrive_vel_millis[0] < ODRIVE_VEL_STALE_MS
      && now - odrive_vel_millis[1] < ODRIVE_VEL_STALE_MS) s.flags |= rec::F_WHEELS_FRESH;
  s.ap = telemetryApByte();
  s.drive = odrive_drive_command;
  s.steer = odrive_steer_command;
  s.vel0_rps = odrive_vel_rps[0];
  s.vel1_rps = odrive_vel_rps[1];
#ifdef BENCH_FAKE_IMU
  // Bench fill: static values, different on each axis so that a swapped field shows. The status
  // record flags them (S_IMU_FAKE), so they never pass as measurements.
  s.imu = s.gyro = true;
  s.ax_g = 0.125f; s.ay_g = -0.250f; s.az_g = 1.000f;
  s.gx_dps = 1.50f; s.gy_dps = -2.50f; s.gyro_dps = 3.50f;
#else
  s.gyro = s.imu = mpu_ok;  // with an MPU6050: all six axes (0x22 block); without: 0x20, gyro z "none"
  if (mpu_ok) {
    s.ax_g = mpu.getAccX(); s.ay_g = mpu.getAccY(); s.az_g = mpu.getAccZ();
    s.gx_dps = mpu.getGyroX(); s.gy_dps = mpu.getGyroY(); s.gyro_dps = mpu.getGyroZ();
  }
#endif
  s.btx1_mm = stmPacket.items.s1_btx_radius[0];
  s.btx2_mm = stmPacket.items.s1_btx_radius[1];
  if (stm_link_ver == 2) {
    // Layout 2 (0.5.45+): the coil fields carry the coil values, sensor 1 coil 1, sensor 1 coil 2, sensor 2 coil 1,
    // sensor 2 coil 2, at the wire bin of each sensor (bin 0 if the sensor has none), value = v x 2^-24. From 0.5.72
    // signed (the Rx STM32 sign, the field side of the coil axis); up to 0.5.71 |v|. Below 100 Hz the log filter
    // averages the signed value. rover_log.py gives them as "coil_mag" (firmware 0.5.45+ and status bit 24).
    for (int sn = 0; sn < 2; sn++) {
      const int lane = ((stmDec.lanes >> 1) & 1) == sn ? 1 : 0;  // lane 2 if it reads this sensor, else lane 1
      const int bn = stmDec.wireBin(sn) < 0 ? 0 : stmDec.wireBin(sn);
      const int32_t v1 = wfrx_int24_get(stmDec.coil[lane][bn].v1), v2 = wfrx_int24_get(stmDec.coil[lane][bn].v2);
      s.coil[2 * sn] = (float)v1 * (1.0f / 16777216.0f);
      s.coil[2 * sn + 1] = (float)v2 * (1.0f / 16777216.0f);
    }
  } else {
    for (int i = 0; i < 4; i++) s.coil[i] = stmPacket.items.coil_data[0][i];
  }
  s.xf_valid = rover_active && roverAp.rd.valid;
  s.xf_m = roverAp.rd.Xf;
  // Layout 3: sensor 2 beacons and position, the wheel positions with their ages, the gyro heading
  s.btx1_s2_mm = stmPacket.items.s2_btx_radius[0];
  s.btx2_s2_mm = stmPacket.items.s2_btx_radius[1];
  const bool s2_on = enable_sensor_2 && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S2_SIGNAL_LOST)
      && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S2_SIGNAL_DISABLED);
  s.x2_mm = s2_on ? stmPacket.items.s2_x : NAN;
  s.y2_mm = s2_on ? stmPacket.items.s2_y : NAN;
  for (int k = 0; k < 2; k++) {
    s.enc_pos[k] = (int32_t)lroundf(odrive_pos_rev[k] * 10000.0f);
    const uint32_t age = now - odrive_vel_millis[k];
    s.enc_age_ms[k] = (!odrive_vel_received[k] || age > 254) ? 255 : (uint8_t)age;
  }
  s.heading = (uint16_t)lround(fmod(fmod(gyro_heading_deg, 360.0) + 360.0, 360.0) * (65536.0 / 360.0));
  telem_packets++;
  // Log rate: the anti-alias filter and decimation (lib/rover_telemetry/decimator.h). Its state is
  // on the heap (static RAM is full). A rate change clears the filter.
  if (!telem_decim) return;
  if (telem_decim->rate() != g_deviceConfig.telemetry_rate_hz) telem_decim->setRate(g_deviceConfig.telemetry_rate_hz);
  rec::Sample o;
  if (!telem_decim->push(s, &o)) return;
  const uint8_t rate = (uint8_t)telem_decim->rate();
  if (!telem_block.fits(o, rate)) {  // 65 s since the block began, or another block type or rate: store it as it is
    telemlog_add(TELEMLOG_FAST, telem_block.ms(), telem_block.body(), (size_t)telem_block.size());
    telem_block.clear();
  }
  if (telem_block.add(o, rate)) {
    telemlog_add(TELEMLOG_FAST, telem_block.ms(), telem_block.body(), (size_t)telem_block.size());
    telem_block.clear();
  }
}

void telemetryStatus(){
  telemlog_counters_t c;
  telemlog_counters(&c);
  rec::Status st;
  st.drive_mode = (uint8_t)drive_mode_status;
  st.charge_step = charge_status;
  st.vehicle_status = (uint8_t)vehicleStatus;
  st.vbatt = stmPacket.items.vbatt;
  st.vcharge = stmPacket.items.vcharge;
  st.icharge = stmPacket.items.icharge;
  st.v5v = stmPacket.items.v5v;
  st.temperature = stmPacket.items.temperature;
  st.ap_state = rover_active ? (uint8_t)roverAp.state : rec::AP_OFF;
  st.ap_sub = (uint8_t)roverAp.sub;
#ifdef BENCH_FAKE_IMU
  const uint8_t imu_flags = rec::S_GYRO | rec::S_IMU_FAKE;
#else
  const uint8_t imu_flags = mpu_ok ? rec::S_GYRO : 0;
#endif
  st.monitor_age_s = monitorAgeS();
  st.pc_unix_ms = pc_time_known ? (uint64_t)((int64_t)millis() + pc_offset_ms) : 0;
  {
    unsigned a = 0, b = 0, c = 0;
    sscanf(ROVER_FW_VERSION, "%u.%u.%u", &a, &b, &c);
    st.fw_ver[0] = (uint8_t)a; st.fw_ver[1] = (uint8_t)b; st.fw_ver[2] = (uint8_t)c;
  st.cpu_busy = cpu_busy_pct; st.cpu_loop = cpu_loop_pct; st.cpu_drive = cpu_drive_pct;  // 0.5.97
  st.dsp_period_us = (stm_link_ver == 2 && isfinite(g_dsp_eff[DSP_EFF_PERIOD]) && g_dsp_eff[DSP_EFF_PERIOD] > 0) ? (uint32_t)g_dsp_eff[DSP_EFF_PERIOD] : 0;  // 0.5.104
  }
  st.flags = (rover_need_charge ? rec::S_GO_HOME : 0) | imu_flags | (can_init_ok ? rec::S_CAN : 0)
           | (vehicleStationary() ? 0 : rec::S_MOVING)
           | (espnow_started ? rec::S_ESPNOW : 0) | (rover_enabled ? rec::S_ROVER_ENABLED : 0)
           | (roverAp.isLocked() ? rec::S_LOCKED : 0);
  st.status_bits = stmPacket.items.status_bits;
  st.charge_code = stmPacket.items.charge_code;
  st.crc_errors = crc_error_count;
  st.overruns = uart_overrun_count;
  st.packets_last_s = telem_packets;
  st.loop_max_us = telem_loop_max_us;
  st.bounces = roverAp.bounces;
  st.turnarounds = roverAp.turnarounds;
  st.charges = roverAp.charges;
  st.odrive_vbus = odrive_node0_vbus;
  st.odrive_ibus = odrive_node0_ibus;
  const uint32_t now = millis();
  for (int n = 0; n < ODRIVE_NODE_COUNT; n++) {
    st.axis_state[n] = odrive_axis_states[n];
    st.axis_errors_or |= odrive_axis_errors[n];
    if (odrive_axis_errors[n]) st.error_nodes |= (uint8_t)(1 << n);
    if (odrive_heartbeat_received[n] && now - odrive_heartbeat_millis[n] < 1000) st.heartbeat_nodes |= (uint8_t)(1 << n);
  }
  st.free_heap = ESP.getFreeHeap();
  st.send_fail = c.send_fail;
  st.backfill_last_s = c.backfill_sent_last_s;
  st.xf_valid = rover_active && roverAp.rd.valid;
  st.xf_m = roverAp.rd.Xf;
  st.dist_m = roverAp.rd.df;
  st.angle_deg = fabsf(roverAp.dec.psiDeg);  // the fitted heading at the last decision (bounce, join, probe), size
#ifdef TELEMETRY_NEWVALUE
  tlm_set(TLM_test_heap_kb, (float)st.free_heap / 1024.0f);  // example line, 1 Hz
#endif
  uint8_t body[rec::STATUS_BYTES];
  rec::packStatus(body, st);
  telemlog_add(TELEMLOG_SLOW, now, body, sizeof(body));
  telem_packets = 0;
  telem_loop_max_us = 0;
}

// The telemetry page (src/web_telemetry.cpp), from loop() on request: the newest status record (the SLOW ring of the
// log), the newest sample (the open sample record) and the table values. Nothing in the frame path.
void webTelemetryCopy(tlmpage::Snapshot* s){
  s->now_ms = millis();
  tlmpage::take_status_log(s);
  tlmpage::take_sample(s, telem_block.body(), telem_block.ms());
  tlmpage::take_values(s);
}

void telemetryEvents(){
  if (drive_mode_status != telem_last_mode) {
    if (telem_last_mode >= 0) telemetryEvent(rec::EV_MODE, telem_last_mode, drive_mode_status, nullptr);
    telem_last_mode = drive_mode_status;
  }
  const int ap = rover_active ? (int)roverAp.state : -1;
  if (ap != telem_last_ap) {
    if (telem_last_ap != -2) {
      char text[40];
      snprintf(text, sizeof(text), "%s->%s", telem_last_ap < 0 ? "off" : rover::stateName((rover::State)telem_last_ap),
               ap < 0 ? "off" : rover::stateName((rover::State)ap));
      telemetryEvent(rec::EV_AP_STATE, telem_last_ap, ap, text);
    }
    telem_last_ap = ap;
  }
  if (charge_status != telem_last_charge) {
    if (telem_last_charge != 99) telemetryEvent(rec::EV_CHARGE_STEP, telem_last_charge, charge_status, nullptr);
    telem_last_charge = charge_status;
  }
  const bool stuck = stuckDetector.GetStatus();
  if (stuck && !telem_last_stuck) telemetryEvent(rec::EV_STUCK, 0, 0, nullptr);
  telem_last_stuck = stuck;
  const bool halted = rover_active && roverAp.state == rover::FAULT;
  if (halted && !telem_last_halted) {
    telemetryEvent(rec::EV_HALTED, roverAp.fault, 0, roverAp.fault == rover::FAULT_STUCK ? "fault: stuck"
                   : roverAp.fault == rover::FAULT_NO_POLARITY ? "fault: no polarity at join" : "fault: wire lost too long");
  }
  telem_last_halted = halted;
  for (int n = 0; n < ODRIVE_NODE_COUNT; n++) {
    const uint32_t fresh = odrive_axis_errors[n] & ~telem_last_axis_errors[n];
    if (fresh) telemetryEvent(rec::EV_ODRIVE_ERROR, n, (int32_t)odrive_axis_errors[n], nullptr);
    telem_last_axis_errors[n] = odrive_axis_errors[n];
  }
}

void telemetryUsbCommand(const char* line){
  if (safe_mode) { usbOut.println("LOG off (safe mode)"); return; }
  unsigned long st = 0, from = 0, count = 0;
  telemlog_info_t in;
  telemlog_info(&in);
  if (!strcmp(line, "log info")) {
    usbOut.printf("LOG info boot=%lu prev=%lu fast=%ld..%ld slow=%ld..%ld fast_prev=%ld..%ld slow_prev=%ld..%ld\n",
      (unsigned long)in.boot_id, (unsigned long)in.prev_boot_id,
      (long)in.r[0][0].oldest, (long)in.r[0][0].newest, (long)in.r[1][0].oldest, (long)in.r[1][0].newest,
      (long)in.r[0][1].oldest, (long)in.r[0][1].newest, (long)in.r[1][1].oldest, (long)in.r[1][1].newest);
    telemlog_counters_t c;
    telemlog_counters(&c);
    usbOut.printf("LOG counters live_sent=%lu live_skipped=%lu backfill_sent=%lu send_fail=%lu req=%lu req_dropped=%lu add_refused=%lu prev_kept=%u\n",
      (unsigned long)c.live_sent, (unsigned long)c.live_skipped, (unsigned long)c.backfill_sent, (unsigned long)c.send_fail,
      (unsigned long)c.req_received, (unsigned long)c.req_dropped, (unsigned long)c.add_refused, (unsigned)c.prev_kept);
    usbOut.printf("LOG sweep frames=%lu in_records=%lu dropped=%lu\n", (unsigned long)sweep_updates,
                  (unsigned long)sweep_sent, (unsigned long)sweep_dropped);  // 0.6.16: the sweep record counters
    return;
  }
  // "log dump <stream> <from_seq> <count>" (this boot) or "log dumpprev <stream> <from> <count>"
  const bool prev = !strncmp(line, "log dumpprev ", 13);
  if (sscanf(line + (prev ? 13 : 9), "%lu %lu %lu", &st, &from, &count) == 3 && st <= 1
      && (prev || !strncmp(line, "log dump ", 9))) {
    telem_dump_stream = (int)st;
    telem_dump_boot = prev ? in.prev_boot_id : in.boot_id;
    const telemlog_range_t& r = in.r[st][prev ? 1 : 0];
    telem_dump_seq = (r.oldest != TELEMLOG_NO_SEQ && from < r.oldest) ? r.oldest : from;
    telem_dump_left = (r.oldest == TELEMLOG_NO_SEQ || telem_dump_seq > r.newest) ? 0
                    : (uint32_t)min((unsigned long)(r.newest - telem_dump_seq + 1), count);
    usbOut.printf("LOG dump boot=%lu stream=%lu from=%lu count=%lu\n", (unsigned long)telem_dump_boot, st,
      (unsigned long)telem_dump_seq, (unsigned long)telem_dump_left);
    return;
  }
  usbOut.println("LOG usage: log info | log dump <stream> <from> <count> | log dumpprev <stream> <from> <count>");
}

void telemetryDumpStep(){
  if (telem_dump_stream < 0) return;
  // One line is at most 7 + 2 x 242 + 1 bytes; queue it only when the USB queue has room for it.
  while (telem_dump_left > 0 && usbOut.room() >= 500) {
    uint8_t frame[TELEMLOG_LOG_HDR + TELEMLOG_FAST_BODY_MAX];
    uint32_t ms = 0;
    const size_t n = telemlog_get(telem_dump_stream, telem_dump_boot, telem_dump_seq, frame + TELEMLOG_LOG_HDR,
                                  TELEMLOG_FAST_BODY_MAX, &ms);
    if (n) {
      frame[0] = TELEMLOG_T_LOG;
      frame[1] = (uint8_t)telem_dump_stream;
      memcpy(frame + 2, &telem_dump_boot, 4);
      memcpy(frame + 6, &telem_dump_seq, 4);
      memcpy(frame + 10, &ms, 4);
      char hex[2 * sizeof(frame) + 1];
      for (size_t i = 0; i < TELEMLOG_LOG_HDR + n; i++) snprintf(hex + 2 * i, 3, "%02x", frame[i]);
      usbOut.printf("LOG %s\n", hex);
    }
    telem_dump_seq++;
    telem_dump_left--;
  }
  if (telem_dump_left == 0) {
    usbOut.println("LOG end");
    telem_dump_stream = -1;
  }
}

void telemetryService(){
  telemetryEvents();
  tlm::service(millis(), rec::rateNearest(g_deviceConfig.telemetry_rate_hz));  // table value records (lib/tlm)
  if (millis() - telem_status_ms >= 1000) {
    telem_status_ms = millis();
    telemetryStatus();
  }
  // The vehicle name (body 0x31, 0.5.69): each 10 s, and at once after a change
  static uint32_t name_ms = 0;
  static char name_sent[17] = "";
  if (millis() - name_ms >= 10000 || strcmp(name_sent, g_deviceConfig.vehicle_name)) {
    name_ms = millis();
    strcpy(name_sent, g_deviceConfig.vehicle_name);
    uint8_t body[3 + rec::VEHICLE_NAME_MAX];
    const int n = rec::packName(body, vehicleID, g_deviceConfig.vehicle_name);
    telemlog_add(TELEMLOG_SLOW, name_ms, body, (size_t)n);
  }
  telemetryDumpStep();
}


// loop(), after the control step: one log sample of this packet and the outputs it gave.
void telemetryFrame(){
  mainLoopCounter();  // increment data loop counter
  mark(SEC_SAMPLE);
  telemetrySample();  // one 100 Hz sample of this packet and the outputs it gave
  dtTelemetry();  // 0.6.11: dual tone polarity items 9 and 10 into the telemetry table
}
