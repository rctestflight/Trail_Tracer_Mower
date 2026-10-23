// The gyro and accelerometer (MPU6050): start, calibration, one sample per loop() pass.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

MPU6050 mpu(Wire);
bool mpu_ok = false;  // false on a board with no gyro; the gyro is logged only (the autopilot does not use it)
int16_t imu_roll16 = 0;  // the accelerometer roll, 65 536 = one turn (Rx UART frame angle); 0 with no IMU
double gyro_heading_deg = 0.0;  // log layout 3: the integral of gyro z over every MPU sample, deg; zero at boot
void gyroHeadingUpdate() {
  static uint32_t last_us = 0;
  const uint32_t now_us = micros();
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
    mpu.update();
    gyro_sample_ms = millis();
    rtprof::gyroSample();  // real-time path timing (lib/rt_prof)
    gyroHeadingUpdate();   // log layout 3: heading = integral of gyro z
    gyro_ready = isfinite(mpu.getGyroZ());
    mpu_ok = gyro_ready;  // the log reads the gyro while it is valid
  }
}
// the main-branch serviceMpu() result (merge of f10cd31): USB text through the queue, and event 7 in the log.
void mpuReport(bool ok, unsigned status){
  if (ok) {
    usbOut.println("MPU6050 calibration complete");
    char t[64];  // want gyro 0x08 (+-500 deg/s), acc 0x00 (+-2 g); id 0x68 for an MPU6050
    snprintf(t, sizeof(t), "offsets set; gyro reg 0x%02x acc reg 0x%02x id 0x%02x", imu_reg_gyro, imu_reg_acc, imu_reg_id);
    usbOut.println(t);
    telemetryEvent(rec::EV_IMU, 2, 500, t);
  } else {
    usbOut.printf("MPU6050 init failed: %u; sensor disabled, motors stay locked%c", status, 10);
    telemetryEvent(rec::EV_IMU, 0, 0, "absent: no gyro, motors stay locked");
  }
}
