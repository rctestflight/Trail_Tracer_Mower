// The gyro and accelerometer (MPU6050): start, calibration, one sample per loop() pass.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

#ifdef IMU_I2C_NB
#include "i2c_nb.h"
#ifdef IMU_SAVE
#include "wf_fastmath.h"  // the wire solve fast atan2 (roll)
#endif
#ifdef DIAG_IMU_PROF
#include "hal/cpu_hal.h"
#endif
ImuNb mpu;
#else
MPU6050 mpu(Wire);
#endif
bool mpu_ok = false;  // false on a board with no gyro; the gyro is logged only (the autopilot does not use it)
int16_t imu_roll16 = 0;  // the accelerometer roll, 65 536 = one turn (Rx UART frame angle); 0 with no IMU
double gyro_heading_deg = 0.0;  // log layout 3: the integral of gyro z over every MPU sample, deg; zero at boot
void gyroHeadingUpdate(uint32_t now_us) {  // now_us: the sample time (micros())
  static uint32_t last_us = 0;
  if (last_us && mpu_ok) gyro_heading_deg += mpu.getGyroZ() * (double)(now_us - last_us) * 1e-6;
  if (mpu_ok) imu_roll16 = (int16_t)lroundf(atan2f(mpu.getAccY(), mpu.getAccZ()) * (32768.0f / (float)M_PI));  // Rx UART frame angle
  last_us = now_us;
}
bool imu_halt = false;  // retired at the merge of f10cd31: the main-branch gyro lock keeps the motors off with no MPU; always false
#define SDA_PIN 39
#define SCL_PIN 40

bool gyro_ready = false;
void mpuReport(bool ok, unsigned status);  // main-branch MPU messages through the USB text queue and the log
uint32_t gyro_sample_ms = 0;
static uint8_t imu_reg_gyro = 0, imu_reg_acc = 0, imu_reg_id = 0;  // GYRO_CONFIG, ACCEL_CONFIG, WHO_AM_I after start
static uint16_t imu_gyro_range = 500;  // deg/s full scale that the gyro scale uses (0.6.31: from the read-back)

#ifndef IMU_I2C_NB
void imuTake(uint8_t) {}  // the Wire path reads in serviceMpu()
#ifdef DIAG_IMU_STAT
// Test build (2026-10-29, IMU read cost): each read timed and checked; bench "imustat" prints and resets the counts.
// A failed read (NACK, short read) leaves -1 in every word: the values then equal -1 / scale - offset on all three gyro
// axes. A bus held low gives 0 in every word: -offset on all three. WHO_AM_I is read again once a second (not timed).
static uint32_t is_n = 0, is_us_sum = 0, is_us_max = 0, is_ff = 0, is_zero = 0, is_id_n = 0, is_id_bad = 0, is_t0 = 0, is_last_id_ms = 0;
static uint8_t is_id_last = 0;
static bool isEq(float v, float raw, float scale, float off) { return v == raw / scale - off; }
void imuStatRead() {
  const uint32_t t0 = (uint32_t)esp_timer_get_time();
  mpu.update();
  const uint32_t us = (uint32_t)esp_timer_get_time() - t0;
  if (!is_n) is_t0 = millis();
  is_n++; is_us_sum += us; if (us > is_us_max) is_us_max = us;
  const float gs = imu_gyro_range == 250 ? 131.0f : imu_gyro_range == 1000 ? 32.8f : imu_gyro_range == 2000 ? 16.4f : 65.5f;  // the library's scales
  const float gx = mpu.getGyroX(), gy = mpu.getGyroY(), gz = mpu.getGyroZ();
  if (isEq(gx, -1, gs, mpu.getGyroXoffset()) && isEq(gy, -1, gs, mpu.getGyroYoffset()) && isEq(gz, -1, gs, mpu.getGyroZoffset())) is_ff++;
  if (isEq(gx, 0, gs, mpu.getGyroXoffset()) && isEq(gy, 0, gs, mpu.getGyroYoffset()) && isEq(gz, 0, gs, mpu.getGyroZoffset())) is_zero++;
  if (millis() - is_last_id_ms >= 1000) {
    is_last_id_ms = millis();
    is_id_last = mpu.readData(0x75); is_id_n++;
    if (is_id_last != imu_reg_id) is_id_bad++;
  }
}
void imuStatReport() {
  const uint32_t s_ms = millis() - is_t0;
  usbOut.printf("IMUSTAT clock_hz=%lu reads=%lu s=%.1f per_s=%.1f us_mean=%lu us_max=%lu fail_ff=%lu fail_zero=%lu id_checks=%lu id_bad=%lu id_last=0x%02x id_start=0x%02x\n",
    (unsigned long)Wire.getClock(), (unsigned long)is_n, s_ms / 1000.0, is_n ? is_n * 1000.0 / s_ms : 0.0, (unsigned long)(is_n ? is_us_sum / is_n : 0),
    (unsigned long)is_us_max, (unsigned long)is_ff, (unsigned long)is_zero, (unsigned long)is_id_n, (unsigned long)is_id_bad, is_id_last, imu_reg_id);
  is_n = is_us_sum = is_us_max = is_ff = is_zero = is_id_n = is_id_bad = 0;
}
#endif
// Take one calibration sample per loop, leaving STM32 packet handling active.
// Keep the sensor stationary until the calibration-complete message appears.
void serviceMpu() {
  enum class State { Init, Settling, Sampling, Ready, Failed };
  static State state = State::Init;
  static uint32_t started_ms = 0;
  static uint32_t last_sample_ms = 0;
  static uint16_t samples = 0;
  static float sums[6] = {};
  const uint32_t now = millis();

  if (state == State::Init) {
    Wire.begin(SDA_PIN, SCL_PIN);
#ifdef IMU_I2C_HZ
    Wire.setClock(IMU_I2C_HZ);  // test build (2026-10-29): the I2C clock; the core default is 100 kHz
#endif
    byte status = mpu.begin();
    if (status != 0) {
      mpuReport(false, status);  // USB text and event 7 (defined after the USB queue and the log)
      state = State::Failed;
      return;
    }
    // 0.6.30: the ranges again after the wake (begin() writes them while the chip sleeps; a chip that drops those
    // writes stays at +-250 deg/s, read as +-500: 2 x the turn), and the registers read back for the log.
    mpu.setGyroConfig(1);
    mpu.setAccConfig(0);
    imu_reg_gyro = mpu.readData(0x1B);
    imu_reg_acc = mpu.readData(0x1C);
    imu_reg_id = mpu.readData(0x75);
    // 0.6.31: the library scale follows the range the chip reports, so a range write that the chip dropped no longer
    // scales the turn. A value with bits outside the range field (a failed read) keeps the written range.
    if ((imu_reg_gyro & ~0x18) == 0 && imu_reg_gyro != 0x08) mpu.setGyroConfig(imu_reg_gyro >> 3);
    if ((imu_reg_acc & ~0x18) == 0 && imu_reg_acc != 0x00) mpu.setAccConfig(imu_reg_acc >> 3);
    imu_gyro_range = ((imu_reg_gyro & ~0x18) == 0) ? (250 << (imu_reg_gyro >> 3)) : 500;
    mpu.setGyroOffsets(0, 0, 0);
    mpu.setAccOffsets(0, 0, 0);
    started_ms = millis();
    state = State::Settling;
    return;
  }
  if (state == State::Failed) return;
  if (state == State::Settling) {
    if (now - started_ms < 500) return;
    state = State::Sampling;
  }
  if (state == State::Sampling) {
    if (now - last_sample_ms < 1) return;
    last_sample_ms = now;
    mpu.fetchData();
    sums[0] += mpu.getAccX();
    sums[1] += mpu.getAccY();
    sums[2] += mpu.getAccZ() - 1.0f;
    sums[3] += mpu.getGyroX();
    sums[4] += mpu.getGyroY();
    sums[5] += mpu.getGyroZ();
    if (++samples == CALIB_OFFSET_NB_MES) {
      mpu.setAccOffsets(sums[0] / samples, sums[1] / samples, sums[2] / samples);
      mpu.setGyroOffsets(sums[3] / samples, sums[4] / samples, sums[5] / samples);
      state = State::Ready;
      // Wait for the first offset-corrected update before enabling motors.
      mpuReport(true, 0);
    }
    return;
  }
  if (now - last_sample_ms >= 1) {
    last_sample_ms = now;
#ifdef DIAG_IMU_STAT
    imuStatRead();
#else
    mpu.update();
#endif
    gyro_sample_ms = millis();
    rtprof::gyroSample();  // real-time path timing (lib/rt_prof)
    gyroHeadingUpdate(micros());   // log layout 3: heading = integral of gyro z
    gyro_ready = isfinite(mpu.getGyroZ());
    mpu_ok = gyro_ready;  // the log reads the gyro while it is valid
  }
}
#else  // IMU_I2C_NB
// Test build (2026-10-30): the IMU through lib/i2c_nb; the loop never waits on the bus. Same values and read rule as
// the Wire path above: one 14-byte read when 1 ms has passed since the last read start, the same start writes and
// read-backs, 500 samples for the offsets. The loop starts a read and returns; the read ends in the interrupt; the
// loop takes the newest result before each use (imuTake). A failed read keeps the last good sample (the Wire path
// gave -1 in each word).
enum : uint8_t { T_SAMPLE = 0, T_W, T_PWR, T_W2, T_RGYRO, T_RACC, T_RID, T_ID };
#ifndef IMU_I2C_HZ
#define IMU_I2C_HZ 400000
#endif
static uint32_t nb_seen[i2cnb::N_TAGS];
static uint8_t nbReg(uint8_t tag) {  // a one-byte register read-back: 0xff after a failed read (as Wire read() -1)
  i2cnb::Result r;
  if (!i2cnb::take(tag, &nb_seen[tag], &r) || r.status != i2cnb::ST_OK) return 0xff;
  return r.data[0];
}
#ifdef DIAG_IMU_STAT
static uint32_t is_n = 0, is_ff = 0, is_zero = 0, is_id_n = 0, is_id_bad = 0, is_t0 = 0, is_last_id_ms = 0;
static uint32_t is_hold_max = 0; static uint64_t is_hold_sum = 0;
static uint8_t is_id_last = 0;
static bool isEq(float v, float raw, float scale, float off) { return v == raw / scale - off; }
static void isHold(uint32_t t0) {  // loop time in the IMU code: the take and the read start
  const uint32_t us = (uint32_t)esp_timer_get_time() - t0;
  is_hold_sum += us; if (us > is_hold_max) is_hold_max = us;
}
#endif
static bool nbConvert(i2cnb::Result* r) {  // the newest finished sample read into mpu; false: none new, or the read failed
  if (!i2cnb::take(T_SAMPLE, &nb_seen[T_SAMPLE], r) || r->status != i2cnb::ST_OK) return false;
  mpu.convert(r->data);
  return true;
}
#ifdef DIAG_IMU_PROF
// Test build (2026-10-30): where the loop time of a sample take goes. Each take is timed in CPU cycles per step. Every
// second take runs with interrupts masked (no preemption inside), and then runs the same arithmetic again at once on a
// copy (warm cache). Bench "imuprof" prints the means and resets.
struct ImuProfAcc { uint64_t sum[6]; uint32_t max_total, n; };
static ImuProfAcc ip_open, ip_masked;  // steps: take, convert, heading and roll, checks, other, total
static uint64_t ip_warm_sum = 0; static uint32_t ip_warm_n = 0;
static uint32_t ip_site[4];  // takes by caller: autopilot step, motor output step, IMU service, other
uint8_t ip_caller = 3;
#endif
#ifdef IMU_SAVE
// IMU savings with the same values (test build IMU_SAVE): the heading integral in whole numbers (sums of raw gyro z x
// microseconds), converted when the log takes a sample; the roll from the newest sample when a frame asks for it.
static int64_t hd_raw_us = 0, hd_us = 0;  // sum of raw z x dt_us; sum of dt_us (from the first integrated sample)
static uint32_t roll_seq = 0, sample_seq = 0;
#ifdef DIAG_IMU_SAVECHK
static double hd_ref = 0, hd_diff_max = 0; static uint32_t hd_checks = 0;  // the double integral, as before
static uint32_t roll_checks = 0, roll_diff_n = 0, roll_diff_max = 0;        // fast roll against atan2f, in counts
#endif
double gyroHeadingDeg() {
  const double h = (double)hd_raw_us / ((double)mpu.gyroLsb() * 1e6) - (double)mpu.getGyroZoffset() * (double)hd_us * 1e-6;
#ifdef DIAG_IMU_SAVECHK
  const double d = fabs(h - hd_ref); hd_checks++; if (d > hd_diff_max) hd_diff_max = d;
#endif
  return h;
}
int16_t imuRoll16() {
  if (roll_seq != sample_seq && mpu_ok) {
    roll_seq = sample_seq;
    imu_roll16 = (int16_t)lroundf(wirefit::fastAtan2f(mpu.getAccY(), mpu.getAccZ()) * (32768.0f / (float)M_PI));
#ifdef DIAG_IMU_SAVECHK
    const int16_t ref = (int16_t)lroundf(atan2f(mpu.getAccY(), mpu.getAccZ()) * (32768.0f / (float)M_PI));
    const uint32_t d = (uint32_t)abs((int)ref - (int)imu_roll16);
    roll_checks++; if (d) roll_diff_n++; if (d > roll_diff_max) roll_diff_max = d;
#endif
  }
  return imu_roll16;
}
static void gyroHeadingSave(uint32_t now_us) {  // as gyroHeadingUpdate(): the same samples and intervals, in whole numbers
  static uint32_t last_us = 0;
  if (last_us && mpu_ok) {
    const int64_t dt = (int64_t)(uint32_t)(now_us - last_us);
    hd_raw_us += (int64_t)mpu.rawGyroZ() * dt;
    hd_us += dt;
#ifdef DIAG_IMU_SAVECHK
    hd_ref += mpu.getGyroZ() * (double)(now_us - last_us) * 1e-6;
#endif
  }
  last_us = now_us;
  sample_seq++;
}
#endif
static bool nbTakeSample() {  // Ready: the newest sample to the firmware (as the Wire path after update())
  i2cnb::Result r;
#ifdef DIAG_IMU_PROF
  static bool mask_next = false;
  uint32_t irq = 0;
  const bool masked = mask_next;
  if (masked) irq = portSET_INTERRUPT_MASK_FROM_ISR();
  uint32_t c[6];
  c[0] = cpu_hal_get_cycle_count();
  if (!i2cnb::take(T_SAMPLE, &nb_seen[T_SAMPLE], &r) || r.status != i2cnb::ST_OK) {
    if (masked) portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
    return false;
  }
  c[1] = cpu_hal_get_cycle_count();
  mpu.convert(r.data);
  c[2] = cpu_hal_get_cycle_count();
#else
  if (!nbConvert(&r)) return false;
#endif
  gyro_sample_ms = (uint32_t)(r.t_us / 1000);  // millis() of the read end (millis() = esp_timer / 1000)
  rtprof::gyroSampleAt(r.t_us);
#ifdef IMU_SAVE
  gyroHeadingSave((uint32_t)r.t_us);
#else
  gyroHeadingUpdate((uint32_t)r.t_us);
#endif
  gyro_ready = isfinite(mpu.getGyroZ());
  mpu_ok = gyro_ready;
#ifdef DIAG_IMU_PROF
  c[3] = cpu_hal_get_cycle_count();
#endif
#ifdef DIAG_IMU_STAT
  if (!is_n) is_t0 = millis();
  is_n++;
  const float gs = imu_gyro_range == 250 ? 131.0f : imu_gyro_range == 1000 ? 32.8f : imu_gyro_range == 2000 ? 16.4f : 65.5f;
  const float gx = mpu.getGyroX(), gy = mpu.getGyroY(), gz = mpu.getGyroZ();
  if (isEq(gx, -1, gs, mpu.getGyroXoffset()) && isEq(gy, -1, gs, mpu.getGyroYoffset()) && isEq(gz, -1, gs, mpu.getGyroZoffset())) is_ff++;
  if (isEq(gx, 0, gs, mpu.getGyroXoffset()) && isEq(gy, 0, gs, mpu.getGyroYoffset()) && isEq(gz, 0, gs, mpu.getGyroZoffset())) is_zero++;
#endif
#ifdef DIAG_IMU_PROF
  c[4] = cpu_hal_get_cycle_count();
  c[5] = c[4];
  ImuProfAcc& a = masked ? ip_masked : ip_open;
  for (int i = 0; i < 4; i++) a.sum[i] += c[i + 1] - c[i];
  a.sum[5] += c[5] - c[0];
  if (c[5] - c[0] > a.max_total) a.max_total = c[5] - c[0];
  a.n++;
  if (masked) {  // the same arithmetic again at once: warm cache, no preemption, no side effects
    ImuNb m = mpu; volatile float sink;
    const uint32_t w0 = cpu_hal_get_cycle_count();
    m.convert(r.data);
    sink = (float)lroundf(atan2f(m.getAccY(), m.getAccZ()) * (32768.0f / (float)M_PI)) + m.getGyroZ() * 1e-6f;
    const float g2 = m.getGyroX(), g3 = m.getGyroY();
    sink = sink + (isEq(g2, -1, 65.5f, m.getGyroXoffset()) ? 1.0f : 0.0f) + (isEq(g3, 0, 65.5f, m.getGyroYoffset()) ? 1.0f : 0.0f);
    ip_warm_sum += cpu_hal_get_cycle_count() - w0; ip_warm_n++;
    (void)sink;
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
  }
  mask_next = !mask_next;
  ip_site[ip_caller < 4 ? ip_caller : 3]++;
#endif
  return true;
}
#ifdef DIAG_IMU_PROF
void imuProfReport() {
  const double mhz = getCpuFrequencyMhz();
  const char* names[6] = {"take", "convert", "heading_roll", "checks", "other", "total"};
  for (int k = 0; k < 2; k++) {
    ImuProfAcc& a = k ? ip_masked : ip_open;
    usbOut.printf("IMUPROF %s n=%lu", k ? "masked" : "open", (unsigned long)a.n);
    for (int i = 0; i < 6; i++) if (i != 4) usbOut.printf(" %s_us=%.1f", names[i], a.n ? a.sum[i] / mhz / a.n : 0.0);
    usbOut.printf(" total_max_us=%.1f\n", a.max_total / mhz);
  }
  usbOut.printf("IMUPROF warm_us=%.1f n=%lu sites: rover=%lu drive=%lu service=%lu other=%lu\n", ip_warm_n ? ip_warm_sum / mhz / ip_warm_n : 0.0,
                (unsigned long)ip_warm_n, (unsigned long)ip_site[0], (unsigned long)ip_site[1], (unsigned long)ip_site[2], (unsigned long)ip_site[3]);
  memset(&ip_open, 0, sizeof(ip_open)); memset(&ip_masked, 0, sizeof(ip_masked)); ip_warm_sum = 0; ip_warm_n = 0;
  memset(ip_site, 0, sizeof(ip_site));
}
#endif
static bool nb_ready = false;
void imuTake(uint8_t site) {
  if (!nb_ready) return;
#ifdef DIAG_IMU_PROF
  ip_caller = site;
#else
  (void)site;
#endif
  const uint8_t prev = last_state.section;  // the IMU work as its own CPU part, then back to the caller's part
  mark(SEC_IMU);
#ifdef DIAG_IMU_STAT
  const uint32_t t0 = (uint32_t)esp_timer_get_time();
  if (nbTakeSample()) isHold(t0);
#else
  nbTakeSample();
#endif
  mark(prev);
}
void serviceMpu() {
  enum class State { Init, Begin, Settling, Sampling, Ready, Failed };
  static State state = State::Init;
  static uint32_t started_ms = 0;
  static uint32_t last_sample_ms = 0;
  static uint16_t samples = 0;
  static float sums[6] = {};
  const uint32_t now = millis();

  if (state == State::Failed) return;
  if (state != State::Init) i2cnb::service();
  if (state == State::Init) {
    // MPU6050_light begin(1, 0): the writes while the chip sleeps, the wake (its status is the result), then
    // (0.6.30) the ranges again and the read-backs. All queued at once; the next passes collect the results.
    if (!i2cnb::begin(SDA_PIN, SCL_PIN, IMU_I2C_HZ)) { mpuReport(false, 4); state = State::Failed; return; }
    i2cnb::write(T_W, ImuNb::ADDR, 0x19, 0x00);  // SMPLRT_DIV
    i2cnb::write(T_W, ImuNb::ADDR, 0x1A, 0x00);  // CONFIG
    i2cnb::write(T_W, ImuNb::ADDR, 0x1B, mpu.gyroScale(1));
    i2cnb::write(T_W, ImuNb::ADDR, 0x1C, mpu.accScale(0));
    i2cnb::write(T_PWR, ImuNb::ADDR, 0x6B, 0x01);  // PWR_MGMT_1: wake, clock from the gyro x PLL
    i2cnb::write(T_W2, ImuNb::ADDR, 0x1B, mpu.gyroScale(1));
    i2cnb::write(T_W2, ImuNb::ADDR, 0x1C, mpu.accScale(0));
    i2cnb::read(T_RGYRO, ImuNb::ADDR, 0x1B, 1);
    i2cnb::read(T_RACC, ImuNb::ADDR, 0x1C, 1);
    i2cnb::read(T_RID, ImuNb::ADDR, 0x75, 1);
    state = State::Begin;
    return;
  }
  if (state == State::Begin) {
    for (uint8_t t = T_W; t <= T_RID; t++) if (i2cnb::pending(t)) return;
    i2cnb::Result r;
    const bool got = i2cnb::take(T_PWR, &nb_seen[T_PWR], &r);
    if (!got || r.status != i2cnb::ST_OK) {  // Wire endTransmission codes: 2 address NACK, 5 timeout, 4 other
      mpuReport(false, !got ? 4 : r.status == i2cnb::ST_NACK ? 2 : r.status == i2cnb::ST_TIMEOUT ? 5 : 4);
      state = State::Failed;
      return;
    }
    imu_reg_gyro = nbReg(T_RGYRO);
    imu_reg_acc = nbReg(T_RACC);
    imu_reg_id = nbReg(T_RID);
    // 0.6.31: the library scale follows the range the chip reports (as the Wire path).
    if ((imu_reg_gyro & ~0x18) == 0 && imu_reg_gyro != 0x08) i2cnb::write(T_W2, ImuNb::ADDR, 0x1B, mpu.gyroScale(imu_reg_gyro >> 3));
    if ((imu_reg_acc & ~0x18) == 0 && imu_reg_acc != 0x00) i2cnb::write(T_W2, ImuNb::ADDR, 0x1C, mpu.accScale(imu_reg_acc >> 3));
    imu_gyro_range = ((imu_reg_gyro & ~0x18) == 0) ? (250 << (imu_reg_gyro >> 3)) : 500;
    mpu.setGyroOffsets(0, 0, 0);
    mpu.setAccOffsets(0, 0, 0);
    started_ms = millis();
    state = State::Settling;
    return;
  }
  if (state == State::Settling) {
    if (now - started_ms < 500) return;
    state = State::Sampling;
  }
  if (state == State::Sampling) {
    i2cnb::Result r;
    if (nbConvert(&r)) {  // calibration: values only; gyro_ready and the log stay off until the offsets (as the Wire path)
      sums[0] += mpu.getAccX();
      sums[1] += mpu.getAccY();
      sums[2] += mpu.getAccZ() - 1.0f;
      sums[3] += mpu.getGyroX();
      sums[4] += mpu.getGyroY();
      sums[5] += mpu.getGyroZ();
      if (++samples == CALIB_OFFSET_NB_MES) {
        mpu.setAccOffsets(sums[0] / samples, sums[1] / samples, sums[2] / samples);
        mpu.setGyroOffsets(sums[3] / samples, sums[4] / samples, sums[5] / samples);
        state = State::Ready;
        nb_ready = true;
#ifdef DIAG_IMU_STAT
        is_n = 0;
#endif
        mpuReport(true, 0);
        return;
      }
    }
    if (now - last_sample_ms >= 1 && !i2cnb::pending(T_SAMPLE)) {
      last_sample_ms = now;
      i2cnb::read(T_SAMPLE, ImuNb::ADDR, ImuNb::REG_ACCEL_OUT, 14);
    }
    return;
  }
  const uint8_t prev = last_state.section;  // the IMU work as its own CPU part
  mark(SEC_IMU);
#ifdef DIAG_IMU_PROF
  ip_caller = 2;
#endif
#ifdef DIAG_IMU_STAT
  const uint32_t t0 = (uint32_t)esp_timer_get_time();
  const bool took = nbTakeSample();
  bool started = false;
#else
  nbTakeSample();
#endif
  if (now - last_sample_ms >= 1 && !i2cnb::pending(T_SAMPLE)) {
    last_sample_ms = now;
    i2cnb::read(T_SAMPLE, ImuNb::ADDR, ImuNb::REG_ACCEL_OUT, 14);
#ifdef DIAG_IMU_STAT
    started = true;
#endif
  }
#ifdef DIAG_IMU_STAT
  if (took || started) isHold(t0);
  if (millis() - is_last_id_ms >= 1000 && !i2cnb::pending(T_ID)) {  // WHO_AM_I once a second, through the queue
    is_last_id_ms = millis();
    i2cnb::read(T_ID, ImuNb::ADDR, 0x75, 1);
  }
  i2cnb::Result r;
  if (i2cnb::take(T_ID, &nb_seen[T_ID], &r)) {
    is_id_n++;
    is_id_last = r.status == i2cnb::ST_OK ? r.data[0] : 0xff;
    if (is_id_last != imu_reg_id) is_id_bad++;
  }
#endif
  mark(prev);
}
#ifdef DIAG_IMU_STAT
void imuStatReport() {
  i2cnb::Counts c;
  i2cnb::counts(&c, true);
  const uint32_t s_ms = millis() - is_t0;
  const double mhz = getCpuFrequencyMhz();
  usbOut.printf("IMUSTAT nb clock_hz=%lu reads=%lu s=%.1f per_s=%.1f hold_us_mean=%.0f hold_us_max=%lu isr_us_mean=%.1f isr_us_max=%.1f isr_n=%lu"
                " xfer_us_mean=%.0f xfer_us_max=%lu fail_nack=%lu fail_tout=%lu fail_arb=%lu fail_stall=%lu recover=%lu resets=%lu full=%lu"
                " fail_ff=%lu fail_zero=%lu id_checks=%lu id_bad=%lu id_last=0x%02x id_start=0x%02x\n",
    (unsigned long)i2cnb::clockHz(), (unsigned long)is_n, s_ms / 1000.0, is_n ? is_n * 1000.0 / s_ms : 0.0,
    is_n ? (double)is_hold_sum / is_n : 0.0, (unsigned long)is_hold_max,
    c.isr_n ? c.isr_cyc_sum / mhz / c.isr_n : 0.0, c.isr_cyc_max / mhz, (unsigned long)c.isr_n,
    c.ok ? (double)c.xfer_us_sum / c.ok : 0.0, (unsigned long)c.xfer_us_max,
    (unsigned long)c.nack, (unsigned long)c.timeout, (unsigned long)c.arb, (unsigned long)c.stall, (unsigned long)c.recover,
    (unsigned long)c.resets, (unsigned long)c.full, (unsigned long)is_ff, (unsigned long)is_zero, (unsigned long)is_id_n,
    (unsigned long)is_id_bad, is_id_last, imu_reg_id);
#ifdef DIAG_IMU_SAVECHK
  usbOut.printf("IMUSAVE heading_checks=%lu heading_diff_max_deg=%.3g heading_deg=%.4f roll_checks=%lu roll_diff_n=%lu roll_diff_max=%lu\n",
                (unsigned long)hd_checks, hd_diff_max, gyroHeadingDeg(), (unsigned long)roll_checks, (unsigned long)roll_diff_n,
                (unsigned long)roll_diff_max);
  hd_checks = 0; hd_diff_max = 0; roll_checks = roll_diff_n = roll_diff_max = 0;
#endif
  is_n = is_ff = is_zero = is_id_n = is_id_bad = 0;
  is_hold_sum = 0; is_hold_max = 0;
}
// Bench "imumath": CPU time of the arithmetic that the Wire path runs per read besides the bus, 1000 times each:
// the fetchData() conversion, the library's update() angle filter, the data checks, the roll angle.
void imuMathBench() {
  uint8_t d[14];
  for (int i = 0; i < 14; i++) d[i] = (uint8_t)(i * 37 + 11);
  ImuNb m = mpu;
  volatile float sink = 0;
  const int N = 1000;
  uint32_t t0 = (uint32_t)esp_timer_get_time();
  for (int i = 0; i < N; i++) { d[13] = (uint8_t)i; m.convert(d); sink = m.getGyroZ(); }
  const uint32_t t_conv = (uint32_t)esp_timer_get_time() - t0;
  float angleX = 0, angleY = 0, angleZ = 0; unsigned long pre = millis();
  t0 = (uint32_t)esp_timer_get_time();
  for (int i = 0; i < N; i++) {  // MPU6050_light update() after fetchData(), as written there
    const float accX = m.getAccX() + i * 1e-6f, accY = m.getAccY(), accZ = m.getAccZ(), gyroX = m.getGyroX(), gyroY = m.getGyroY(), gyroZ = m.getGyroZ();
    float sgZ = (accZ >= 0) - (accZ < 0);
    float angleAccX = atan2(accY, sgZ * sqrt(accZ * accZ + accX * accX)) * RAD_2_DEG;
    float angleAccY = -atan2(accX, sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG;
    unsigned long Tnew = millis();
    float dt = (Tnew - pre) * 1e-3;
    pre = Tnew;
    angleX = (0.98f * (angleX + gyroX * dt)) + ((1.0 - 0.98f) * angleAccX);
    angleY = (0.98f * (angleY + gyroY * dt)) + ((1.0 - 0.98f) * angleAccY);
    angleZ += gyroZ * dt;
  }
  sink = angleX + angleY + angleZ;
  const uint32_t t_filt = (uint32_t)esp_timer_get_time() - t0;
  const float gs = 65.5f; uint32_t hits = 0;
  t0 = (uint32_t)esp_timer_get_time();
  for (int i = 0; i < N; i++) {
    const float gx = m.getGyroX() + i, gy = m.getGyroY(), gz = m.getGyroZ();
    if (isEq(gx, -1, gs, m.getGyroXoffset()) && isEq(gy, -1, gs, m.getGyroYoffset()) && isEq(gz, -1, gs, m.getGyroZoffset())) hits++;
    if (isEq(gx, 0, gs, m.getGyroXoffset()) && isEq(gy, 0, gs, m.getGyroYoffset()) && isEq(gz, 0, gs, m.getGyroZoffset())) hits++;
  }
  const uint32_t t_chk = (uint32_t)esp_timer_get_time() - t0;
  int32_t roll = 0;
  t0 = (uint32_t)esp_timer_get_time();
  for (int i = 0; i < N; i++) roll += (int16_t)lroundf(atan2f(m.getAccY() + i * 1e-4f, m.getAccZ()) * (32768.0f / (float)M_PI));
  const uint32_t t_roll = (uint32_t)esp_timer_get_time() - t0;
  usbOut.printf("IMUMATH n=%d conv_us=%.2f filter_us=%.2f checks_us=%.2f roll_us=%.2f (sink %ld %lu)\n", N, t_conv / (double)N,
                t_filt / (double)N, t_chk / (double)N, t_roll / (double)N, (long)roll + (long)sink, (unsigned long)hits);
}
#endif
#endif  // IMU_I2C_NB
// the main-branch serviceMpu() result (merge of f10cd31): USB text through the queue, and event 7 in the log.
void mpuReport(bool ok, unsigned status){
  if (ok) {
    usbOut.println("MPU6050 calibration complete");
    char t[64];  // want gyro 0x08 (+-500 deg/s), acc 0x00 (+-2 g); id 0x68 for an MPU6050
    snprintf(t, sizeof(t), "offsets set; gyro reg 0x%02x acc reg 0x%02x id 0x%02x", imu_reg_gyro, imu_reg_acc, imu_reg_id);
    usbOut.println(t);
    telemetryEvent(rec::EV_IMU, 2, imu_gyro_range, t);
  } else {
    usbOut.printf("MPU6050 init failed: %u; sensor disabled, motors stay locked%c", status, 10);
    telemetryEvent(rec::EV_IMU, 0, 0, "absent: no gyro, motors stay locked");
  }
}
