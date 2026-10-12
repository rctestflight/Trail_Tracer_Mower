// frame_pid: the PID_v1 algorithm (br3ttb Arduino PID 1.2.1), stepped once per STM32 frame with the frame
// period as dt. PID_v1 runs on a millis() gate and scales ki and kd by a fixed sample time; at a frame period
// other than that sample time it skips frames or scales the gains wrong. Here each step uses its own dt, so
// the same Kp, Ki, Kd give the same continuous-time controller at any frame period.
// Same rules as PID_v1: proportional on error, integral clamped to the output limits, derivative on the
// measurement, reverse direction negates all three gains. At dt = the PID_v1 sample time the outputs equal
// PID_v1's (test/telemetry_host/frame_pid_test.cpp). No Arduino dependency.
#pragma once

namespace framepid {

class Pid {
 public:
  // Gains as the user sets them (PID_v1 SetTunings): Kp, Ki in 1/s, Kd in s; negative gains are refused.
  void setTunings(double kp, double ki, double kd) {
    if (kp < 0 || ki < 0 || kd < 0) return;
    kp_ = kp; ki_ = ki; kd_ = kd;
  }
  void setReverse(bool reverse) { reverse_ = reverse; }
  void setOutputLimits(double lo, double hi) {
    if (lo >= hi) return;
    min_ = lo; max_ = hi;
    sum_ = clamp(sum_);
  }
  // The bumpless start of PID_v1 SetMode(AUTOMATIC): the integral starts at the present output.
  void start(double input, double output) {
    sum_ = clamp(output);
    last_input_ = input;
    started_ = true;
  }
  void stop() { started_ = false; }
  bool started() const { return started_; }

  // One step: dt_s = the frame period in effect, in seconds. Returns the new output.
  double step(double setpoint, double input, double dt_s) {
    if (!started_) start(input, 0.0);
    if (!(dt_s > 0)) return output_;
    const double s = reverse_ ? -1.0 : 1.0;
    const double error = setpoint - input;
    const double d_input = input - last_input_;
    sum_ = clamp(sum_ + s * ki_ * dt_s * error);
    output_ = clamp(s * kp_ * error + sum_ - s * (kd_ / dt_s) * d_input);
    last_input_ = input;
    return output_;
  }
  double output() const { return output_; }

 private:
  double clamp(double v) const { return v > max_ ? max_ : (v < min_ ? min_ : v); }
  double kp_ = 0, ki_ = 0, kd_ = 0;
  bool reverse_ = false, started_ = false;
  double min_ = 0, max_ = 255;  // PID_v1's default limits
  double sum_ = 0, last_input_ = 0, output_ = 0;
};

}  // namespace framepid
