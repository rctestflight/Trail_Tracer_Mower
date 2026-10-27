// Rover autopilot: see rover_autopilot.h for the states and the readings.
#include "rover_autopilot.h"
#include <math.h>
#if defined(BASICS_DEBUG) || defined(ROVER_WIRE_SOLVE)
#include <stdio.h>
#endif
#ifdef ROVER_WIRE_SOLVE
#include <stdlib.h>
#include <new>
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
#endif

namespace rover {

namespace {
const float D2R = 0.017453293f;
const float PI_F = 3.14159265f;
// Fixed in code (not settings, operator 2026-10-20)
const float CAPTURE_OFFSET = 0.10f;   // m: |X| at which the open-loop join hands over to FOLLOW
const float RUN_LIMIT = 3.0f;         // open-loop join: hand over after this many times the expected run
const float STOP_SPEED = 0.02f;       // m/s
const float STOP_YAW = 0.25f;         // rad/s: stopped under this (a driving rover's gyro reads up to ~0.2 rad/s)
const float STOP_TIME = 1.5f;         // s: stopped at the latest after this
const float YAW_ACCEL = 3.0f;         // rad/s^2: the pivot brakes in time for this
const float PIVOT_MIN = 0.3f;         // part of the full yaw rate at the end of a pivot
const float WRONG_FRAMES = 20;        // packets of wrong direction in a row before the reversal
const float REVERSE_ARM = 1.0f;       // m of FOLLOW after a reversal before the check acts again
const float FILTER_TAU = 0.05f;       // s, reading low-pass
const float REARM_TAU = 0.3f;         // s, radius low-pass for the bounce re-arm: a rover log reads 1 cm radius noise (95 %)
const float PIVOT_RADIUS = 0.15f;     // m: a turn on the spot turns about a point at most this far from the rover centre.
                                      // In a pivot the inner wheel stands still or turns backward, so the turn centre
                                      // lies within half the wheel track; 0.15 m is under half the track of any rover.
                                      // A driven curve (the wire follower, a roam correction) turns on a larger radius.
// Bounce style basics (Join then leave), tuned in the PC simulation
const float B_LOST_FAR = 99.0f;       // m: a lost frame counts as far from the wire in the median radius
const float B_WN = 1.0f, B_ZETA = 0.9f;  // follow controller: natural frequency rad/s, damping (simulation: 1.5 the same, 2.0 unstable)
const float B_RATE_HZ = 1.0f;         // low-pass on the side distance and on its rate. 3 Hz (simulation) let the Rx noise at
                                      // 0.8 m swing the rate +-0.2 m/s in the emulator (60 % of the approach speed) and
                                      // the steering full scale; at 1 Hz the lag at the follow's 0.16 Hz is under 10 deg
#ifndef JOIN_ROOM_K
#define JOIN_ROOM_K 1.0f
#endif
const float B_SIDE_REL = 0.04f;       // |y| under 4 % of the radius: the wire side of the heading is not readable (emulator,
                                      // 2248 joins at the lawn noise and 4 x it: the side right in every join above, wrong in
                                      // 20-28 % below, within a few degrees of square-on)
const float B_YREL = 0.25f;           // |y| under 25 % of the radius says nothing about the frame (the worldFacts blind line)
const float B_TURN_LIMIT = 3.0f;      // a driven turn ends at the latest after this many times its time at the full rate
const int SJ_MIN_N = 10;              // the planned sweep join takes a solve over at least 10 samples (1 s of drive): one
                                      // over 0.3 s of drive after the start put the follow on the wrong side (emulator)
const float SJ_END_IN = 0.15f;        // m: the planned sweep join ends this far inside the follow line. Planned on the
                                      // line, every join ran 0.10-0.22 m past the wire (emulator, a skid steer: the
                                      // true speed is 1.16 x the speed setting, so the arc is that much wider; solve error)
const float B_NULL_DEG = 15.0f;       // deg from square-on: the band where the field null loses the wire (12-15 deg in the
                                      // field model); the square-on guard acts only for a heading in it

float wrapPi(float a) {
  while (a > PI_F) a -= 2 * PI_F;
  while (a <= -PI_F) a += 2 * PI_F;
  return a;
}
int8_t sgn(float v) { return v < 0 ? -1 : 1; }
}  // namespace

const char* stateName(State s) {
  switch (s) {
    case ROAM: return "roam";
    case RETURN: return "return";
    case JOIN: return "join";
    case FOLLOW: return "follow";
    case DEPART: return "depart";
    case FAULT: return "fault";
    case DOCK: return "dock";
  }
  return "?";
}

const char* decisionName(DecisionCode c) {
  switch (c) {
    case DEC_NONE: return "none";
    case DEC_BOUNCE: return "bounce";
    case DEC_CHECK_FLIP: return "check-flip";
    case DEC_JOIN: return "join";
    case DEC_FOLLOW: return "follow";
    case DEC_REVERSE: return "reverse";
    case DEC_BEACON: return "beacon";
    case DEC_DOCK: return "dock";
    case DEC_DEPART: return "depart";
    case DEC_RETURN: return "return";
    case DEC_FAULT: return "fault";
    case DEC_TX_OFF: return "tx-off";
    case DEC_TX_ON: return "tx-on";
    case DEC_START: return "start";
    case DEC_TURN_BACK: return "turn-back";
    case DEC_PROBE: return "probe";
    case DEC_FOLLOW_END: return "follow-end";
    case DEC_SQUARE_ON: return "square-on";
  }
  return "?";
}

void Autopilot::reset() {
  state = ROAM;
  sub = SUB_START;
  fault = FAULT_NONE;
  rd = Reading();
  txWait = false;
  lostT_ = stopT_ = startT_ = 0;
  armed_ = beaconArmed_ = joinArmed_ = true;
  settle_ = hold_ = wasHold_ = false;
  startCheck_ = true;
  turnBacks_ = 0;
  pivTarget_ = pivTurned_ = pivT_ = 0;
  run_ = departLeft_ = reverseArm_ = 0;
  roamT_ = followT_ = 0;
  wrongCount_ = 0;
  entrySeen_ = false;
  exitArmed_ = true;
  fit_.reset();
  last_ = Output();
  probeLeft_ = 0;
  bb_ = false;
  bP_ = bM_ = bS_ = 1;   // the rover starts inside (rover_join_no_pol "inside")
  bWasLost_ = true;
  bN_ = bI_ = 0;
  bPeak_ = 0;
  bPeakX_ = -1e9f;
  bStopped_ = false;
  dir_ = 1;
  lead_ = leadWant_ = fed_ = 1;
  s2SeenT_ = 99;
  s2LostT_ = 0;
  sjPhase_ = 0;
  sweepJoin_ = false;
  arcGain_ = 1;
#ifdef ROVER_WIRE_SOLVE
  if (wfb_) { wfb_->cfg.sensorFwd = wirefit::Config().sensorFwd; wfb_->cfg.sensorLeft = wirefit::Config().sensorLeft; }
#ifdef ROVER_WIRE_SOLVE_BG0
  if (wfb_) wfb_->bc.bgField = false;   // test build: no uniform vertical field term
#endif
  if (wfb_) wfb_->reset();
  wres_ = wirefit::BounceResult();
  wTotal_ = -1;
#endif
}

void Autopilot::releaseSolve() {
#ifdef ROVER_WIRE_SOLVE
#ifdef ESP_PLATFORM
  // the task must not work on the memory: wait for its answer (the longest measured solve: 262 ms of CPU)
  for (int i = 0; i < 5000 && __atomic_load_n(&sState_, __ATOMIC_ACQUIRE) == 1; i++) vTaskDelay(1);
  if (__atomic_load_n(&sState_, __ATOMIC_ACQUIRE) == 1) return;   // still at work: keep the memory, never free under it
  if (sTask_) { vTaskDelete((TaskHandle_t)sTask_); sTask_ = nullptr; }
  sState_ = 0;
#endif
  delete wfb_;
  wfb_ = nullptr;
  wres_ = wirefit::BounceResult();
  wTotal_ = -1;
#endif
}

#ifdef ROVER_WIRE_SOLVE
// The solve on the ESP32-S2 (no floating-point unit): 34 ms of CPU per solve on a lawn log of the reference rover (most 262 ms), and
// 0.56 s for the direction profile of 0.7.1 (most 0.85 s; bench board, 0.7.0 timing build). In the loop that stopped the motor
// commands and the Rx link for up to a second, so a task at the loop's priority does the work: the loop waits at most
// one tick for it. On the PC (emulator) the same steps run at once
#ifdef ESP_PLATFORM
static Autopilot* sSolveAp = nullptr;
static void solveTaskFn(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (sSolveAp) sSolveAp->solveTaskRun();
  }
}
#endif

void Autopilot::solveTaskRun() {
#ifdef ESP_PLATFORM
  sRes_ = wfb_->iterate();
  __atomic_store_n(&sState_, (uint8_t)2, __ATOMIC_RELEASE);
#endif
}

void Autopilot::solveStep() {
#ifdef ESP_PLATFORM
  if (__atomic_load_n(&sState_, __ATOMIC_ACQUIRE) == 2) {
    wres_ = sRes_;
    sState_ = 0;
  }
  if (sState_ != 0 || wfb_->total() == wTotal_) return;   // busy: the next free step takes the newest sample
  wTotal_ = wfb_->total();
  wirefit::BounceResult r;
  if (!wfb_->prepare(r)) {
    wres_ = r;
    return;
  }
  if (!sTask_) {
    sSolveAp = this;
    TaskHandle_t t = nullptr;
    // 4 kB stack: iterate() needs under 1 kB of locals; the soft-float library calls add little
    if (xTaskCreate(solveTaskFn, "wsolve", 4096, nullptr, uxTaskPriorityGet(nullptr), &t) != pdPASS) {
      toFault(FAULT_NO_MEMORY);
      return;
    }
    sTask_ = t;
  }
  __atomic_store_n(&sState_, (uint8_t)1, __ATOMIC_RELEASE);
  xTaskNotifyGive((TaskHandle_t)sTask_);
#else
  if (wfb_->total() != wTotal_) { wTotal_ = wfb_->total(); wres_ = wfb_->solve(); }
#endif
}

void Autopilot::bounceBegin() {
  bounceWaitT_ = 0;
  sub = SUB_BOUNCE_WAIT;
  stopT_ = 0;
}

bool Autopilot::solveSettled() const {
#ifdef ESP_PLATFORM
  return __atomic_load_n(&sState_, __ATOMIC_ACQUIRE) == 0 && wfb_ && wfb_->total() == wTotal_;
#else
  return true;
#endif
}

// The solve's distance, no less than the Rx radius allows. Inside a loop the rest of the loop only adds field, so the
// Rx radius reads no longer than the true radial distance (emulator, 9104 decisions: 98.5 %; the rest within one
// vertical distance of the wire, the coil null, or the Rx filter lag). Each reading of the last vertical distance of
// travel gives a floor: its horizontal distance at the solve's height, less the travel since it. The least of these
// floors counts, so a null reading over the wire (a long radius, briefly) cannot raise the distance.
bool Autopilot::solveNow(wirefit::BounceResult& r) const {
  if (!wfb_) { r = wirefit::BounceResult(); return false; }
  r = wfb_->now(wres_);
  const int n = wfb_->size();
  if (!r.valid || n == 0) return r.valid;
#ifndef ROVER_WIRE_SOLVE_NOFLOOR
  const float sNow = wfb_->sample(n - 1).s, H = r.Hc;
  float lo = 0;
  bool any = false;
  for (int k = n - 1; k >= 0; k--) {
    const wirefit::BounceSample& b = wfb_->sample(k);
    const float back = sNow - b.s;
    if (back > H) break;
    if (b.r <= 0) continue;
    const float L = sqrtf(fmaxf(0.0f, b.r * b.r - H * H)) - back;
    if (!any || L < lo) lo = L;
    any = true;
  }
  if (any && lo > r.d) r.d = lo;
#endif
  return true;
}

// One line per bounce or join decision to the file WS_LOG names, for the comparison with the emulator truth
// (EMU_TRUTH; the Rx radius R pairs the two): the solve as used
static FILE* wsLogFile() {
  static FILE* f = nullptr;
  static bool tried = false;
  if (!tried) { tried = true; const char* p = getenv("WS_LOG"); if (p && *p) f = fopen(p, "a"); }
  return f;
}

void Autopilot::solveDebug(const char* what, const Inputs& in) const {
  FILE* f = wsLogFile();
  if (!f) return;
  wirefit::BounceResult r;
  const bool ok = solveNow(r);
  fprintf(f, "WS %s ok %d d %.3f psi %.1f beta %.1f H %.3f bg %.3f n %d inr %d R %.3f scan %d psisd %.1f\n", what, ok ? 1 : 0, r.d,
          r.psiDeg, r.betaDeg, r.Hc, r.bg, r.n, (int)in.inRange, in.R, 0, r.psiSd);   // scan: the full-turn scan is gone (0)
  // the cost profile over psi at several cost rises: the calibration of the honest uncertainty against the truth
  if (wfb_)
    fprintf(f, "WP %s %.0f %.0f %.0f %.0f %.0f %.0f\n", what, wfb_->psiProfile(0.5f), wfb_->psiProfile(1), wfb_->psiProfile(2),
            wfb_->psiProfile(4), wfb_->psiProfile(8), wfb_->psiProfile(16));
  fflush(f);
}

namespace {
// A wire direction in the rover frame (deg, counterclockwise positive) folded to (-90, 90]: the nearer way along it
float fold90(float a) {
  while (a > 90) a -= 180;
  while (a <= -90) a += 180;
  return a;
}
}
#endif

float Autopilot::rnd() {
  rng_ = rng_ * 1103515245u + 12345u;
  return (float)(rng_ & 0xFFFFFFu) / (float)0x1000000;
}

void Autopilot::decide(DecisionCode c, float turnDeg, int8_t side) {
  dec.seq++;
  dec.code = c;
  dec.d = rd.d;
  dec.X = rd.X;
  dec.turnDeg = turnDeg;
  dec.side = side;
  dec.reason = (int8_t)fault;
  dec.fit = FitResult();
  if (c == DEC_BOUNCE || c == DEC_JOIN || c == DEC_PROBE) {
    fit_.solve(dec.fit, polNow_);
  }
  dec.psiDeg = dec.fit.valid ? dec.fit.psiDeg : 0;
  dec.solveOk = false;
}

// The approach fit is good enough when its uncertainty is under the limit.
bool Autopilot::fitGood(float limitDeg) const {
  FitResult f;
  return fit_.solve(f, polNow_) && f.uncDeg < limitDeg;
}

// Heading and wire side for a bounce or a join, from the approach fit (or the probe that extended it). No fit at all
// (probe off): square-on. Wire side: polarity when known, else the fit, else the offset sign of this reading.
void Autopilot::planFromFit(const Inputs& in) {
  FitResult f;
  const bool ok = fit_.solve(f, polNow_);
  planPsi_ = ok ? fabsf(f.psiDeg) * D2R : PI_F / 2;
  planToward_ = (ok && f.psiDeg * f.dNow < 0) ? -1 : 1;   // moving away from the wire: heading and side disagree
  planX_ = rd.X;
  const int8_t ps = polSide(in);
  planPolKnown_ = ps != 0;
  planSide_ = ps ? ps : (ok ? (int8_t)(f.dNow < 0 ? -1 : 1) : sgn(rd.X));
}

void Autopilot::probeBegin(ProbeFor why, Output& o, float speed) {
  probeFor_ = why;
  probeLeft_ = cfg.probeLength;
  probeDir_ = 1;
  sub = SUB_PROBE;
  o.v = probeDir_ * speed;
}

void Autopilot::bounceNow(const Inputs& in) {
  planFromFit(in);
#ifdef ROVER_WIRE_SOLVE
  // The solve in place of the approach fit: the angle to the wire and, with no polarity, the wire side (the solve's
  // line has a side: the wire on the left when it runs forward past sensor 1 on its left)
  wirefit::BounceResult ws;
  const bool ok = solveNow(ws);
  solveDebug("bounce", in);
  if (ok) {
    // Leave angle: the mirror of the approach plus the spread, within the limit (as planBounce). Heading: the solve's
    // wire direction (the one with sensor 1 on its right) turned by the leave angle toward sensor 1's side. This holds
    // for any heading at the bounce, also after the scan; for a straight approach it is the turn of planBounce.
    const float a = fabsf(fold90(ws.psiDeg)) * D2R, mn = cfg.bounceMinDeg * D2R;
    float leave = a + (2 * rnd() - 1) * cfg.bounceSpreadDeg * D2R;
    leave = fmaxf(mn, fminf(PI_F - mn, leave));
    planPsi_ = a;
    planLeave_ = leave;
    planSide_ = cosf(ws.psiDeg * D2R) >= 0 ? 1 : -1;
    const float T = wrapPi(ws.psiDeg * D2R - leave);
    planTurn_ = fabsf(T);
    planDir_ = T >= 0 ? 1 : -1;
  } else {
    // No solve answer (no default angle): the rover came from inside, so it turns round and goes
    // back the way it came. A default angle put three start bounces 60 to 90 deg off (emulator)
    planPsi_ = 0;
    planLeave_ = PI_F;
    planSide_ = 0;
    planTurn_ = PI_F;
    planDir_ = 1;
  }
#else
  planBounce();
#endif
  decide(DEC_BOUNCE, planDir_ * planTurn_ / D2R, planSide_);
#ifdef ROVER_WIRE_SOLVE
  if (ok) { dec.solveOk = true; dec.solvePsiDeg = ws.psiDeg; dec.solveD = ws.d; dec.solveSensor = (uint8_t)lead_; }
#endif
  sub = SUB_STOP;
  stopT_ = 0;
}

void Autopilot::joinNow(const Inputs& in) {
  planFromFit(in);
  planJoin();
  planDir_ = fabsf(planRot_) > 1e-3f ? sgn(planRot_) : 1;
  decide(DEC_JOIN, planRot_ / D2R, planSide_);
  sub = SUB_STOP;
  stopT_ = 0;
}

// The approach fit takes every frame in range of a straight drive: no commanded turn, not the wire follower. A
// commanded turn starts a new fit; the gyro does not (straight drive on a real rover reads several deg/s). Lost frames
// carry no equation; the distance driven still counts.
void Autopilot::feedFit(const Inputs& in) {
  if (last_.external || last_.w != 0.0f) { fit_.reset(); return; }
  fit_.advance(in.v * in.dt);
  if (in.inRange && in.R > 0) fit_.add(in.x, in.y, in.R, cfg.sensorHeight, in.inside);
}

// The leading sensor (the best available sensor): in a sweeping mode the rover roams either
// way round; in reverse, sensor 2 (at the back) leads and feeds every reading of the roam, the solve included. The
// join, the follow and the depart drive forward on sensor 1. No distance between the sensors is assumed: the
// solve models sensor 2 at the turn centre. Each sensor keeps its own Rx sign; the solve aligns each reading to the last
// one of the same sensor, and its buffer starts again at each change of sensor (leadApply)
Inputs Autopilot::leadInputs(const Inputs& in) const {
  Inputs l = in;
  if (lead_ == 2 && !bb_) { l.inRange = in.inRange2; l.x = in.x2; l.y = in.y2; l.R = in.R2; }
  return l;
}

// After a change of drive direction: sensor 2 when the rover drives in reverse and sensor 2 has read in the last 2 s
void Autopilot::leadPick() {
  leadWant_ = (dir_ < 0 && s2SeenT_ < 2.0f) ? 2 : 1;
  s2LostT_ = 0;
}

void Autopilot::leadApply() {
#ifdef ROVER_WIRE_SOLVE
#ifdef ESP_PLATFORM
  if (__atomic_load_n(&sState_, __ATOMIC_ACQUIRE) == 1) return;   // the task works on the old sensor's window: wait
  sState_ = 0;                                                      // its answer belongs to the old sensor
#endif
  if (wfb_) {
    wfb_->cfg.sensorFwd = leadWant_ == 1 ? wirefit::Config().sensorFwd : 0.0f;
    wfb_->cfg.sensorLeft = leadWant_ == 1 ? wirefit::Config().sensorLeft : 0.0f;
  }
  wres_ = wirefit::BounceResult();
  wTotal_ = -1;   // the next frame starts the buffer again
#endif
  lead_ = leadWant_;
}

Output Autopilot::step(const Inputs& in0) {
  s2SeenT_ = (in0.inRange2 && in0.R2 > 0) ? 0 : s2SeenT_ + in0.dt;
  // Sensor 2 leads but reads nothing while sensor 1 reads, for 0.5 s: sensor 1 takes over
  if (lead_ == 2 && !bb_ && !(in0.inRange2 && in0.R2 > 0) && in0.inRange) {
    s2LostT_ += in0.dt;
    if (s2LostT_ > 0.5f) leadWant_ = 1;
  } else s2LostT_ = 0;
  if (bb_) leadWant_ = 1;
  const Inputs in = leadInputs(in0);
  // A change of the sensor that feeds the readings: its wire side starts again from the polarity (basicsSide)
  const int8_t fed = (lead_ == 2 && !bb_) ? 2 : 1;
  if (fed != fed_) { bWasLost_ = true; bM_ = in.inside < 0 ? -1 : 1; fed_ = fed; }
  polNow_ = polSide(in);
  feedFit(in);
  last_ = stepStates(in);
  return last_;
}

// The end of the start: on the wire -> FOLLOW (returns true), else ROAM or RETURN.
bool Autopilot::startDone(const Inputs& in, Output& o) {
  if (sweeping() && rd.valid && (fabsf(rd.X) <= CAPTURE_OFFSET || rd.d <= 0.5f * cfg.sensorHeight)) {
    // A sweeping mode on the wire: the own follow (no turn on the spot), its short follow, then the depart arc
    bb_ = true; state = FOLLOW; sub = SUB_NONE; bLong_ = false;
    bT_ = bTurned_ = 0; bPlanned_ = bLeaveDeg();
    bOnLine_ = bHavePrev_ = bStopped_ = false; bMinXs_ = 1e9f; bGuardTurnT_ = 0;
    bYShot_ = bYActive_ = bGuardWas_ = false; sweepJoin_ = false; sjPhase_ = 0;
    startCheck_ = false;
    decide(DEC_START, 0, 0);
    return true;
  }
  if (rd.valid && (fabsf(rd.X) <= CAPTURE_OFFSET || rd.d <= 0.5f * cfg.sensorHeight)) {
    state = FOLLOW; sub = SUB_NONE;
    startCheck_ = false;
    wrongCount_ = 0; entrySeen_ = false; exitArmed_ = !in.exitBeacon; reverseArm_ = 0;
    decide(DEC_START, 0, 0);
    o.external = true;
    return true;
  }
  sub = SUB_NONE;
  if (cfg.followOnly) state = RETURN;
  decide(DEC_START, 0, 0);
  return false;
}

void Autopilot::toFault(FaultCode why) {
  if (state == FAULT) return;
  bb_ = false;
  fault = why;
  state = FAULT;
  sub = SUB_NONE;
  decide(DEC_FAULT, 0, 0);
}

// One reading from x, y and radius as the Rx sends them.
void Autopilot::read(const Inputs& in) {
  if (!in.inRange || !(in.R > 0)) {
    rd.valid = false;
    return;
  }
  const float H = cfg.sensorHeight, x = in.x, y = in.y, R = in.R;
  const float X = fabsf(y) > 1e-4f ? H * x / y : (((x < 0) != (y < 0)) ? -1e4f : 1e4f);
  // Distance from the vertical field part: d / (d^2 + H^2) = |x| / R^2 (a straight wire); the root nearer
  // sqrt(R^2 - H^2).
  const float q = fabsf(x) / (R * R);
  const float d1 = sqrtf(fmaxf(0.0f, R * R - H * H));
  float d;
  if (q < 1e-6f) d = 0;
  else {
    const float disc = 1 - 4 * H * H * q * q;
    if (disc <= 0) d = H;
    else {
      const float r1 = (1 + sqrtf(disc)) / (2 * q), r0 = H * H / r1;
      d = fabsf(r0 - d1) < fabsf(r1 - d1) ? r0 : r1;
    }
  }
  const float k = 1 - expf(-in.dt / FILTER_TAU), kr = 1 - expf(-in.dt / REARM_TAU);
  if (!rd.valid) { rd.Xf = X; rd.df = d; rd.Rf = rd.Rs = R; }
  else { rd.Xf += k * (X - rd.Xf); rd.df += k * (d - rd.df); rd.Rf += k * (R - rd.Rf); rd.Rs += kr * (R - rd.Rs); }
  rd.X = X; rd.d = d;
  rd.valid = true;
}

// The end of a turn: the bounce re-arm starts from the radius here (the slow radius restarts, so its lag through the
// turn does not read as a fall)
void Autopilot::rearmStart() {
  rd.Rs = rd.Rf;
  rMax_ = rd.valid ? rd.Rs : 99;
}

// The pivot rule: no event of any kind in a turn on the spot. In a turn on the spot the sensor
// swings round the turn centre: its radius and side change with no drive toward or away from the wire. So no event of
// any kind triggers while the rover turns on the spot, or while a pivot that has ended still turns. A turn on the spot
// is a commanded pivot, or a measured one (onSpot). The end of each pivot sets settle_ until the turn has ended
// (settled). Held: every decision and fault (bounce, re-arm, join, outside turn-back, beacon, wrong direction, follow
// end, return, depart turn, stuck, lost, lost search, Tx off, dock, the start polarity check). The side filter
// (src/main.cpp insideFilter) still reads in a turn: the sensor's side is real, and a read there corrects a wrong
// side before the turn ends (emulator 2026-10-24: with the reads dropped, a side wrong since the join stayed through
// the follow, and the rover turned back inside the loop 4 times in 4 runs). The bounce re-arm starts again from the
// reading at the end of the hold.
void Autopilot::settleStart() {
  settle_ = true;
  settleT_ = 0;
}

// A measured turn on the spot: the yaw rate over the stop limit, and the turn centre within PIVOT_RADIUS (v = w r).
bool Autopilot::onSpot(const Inputs& in) const {
  return fabsf(in.yawRate) >= STOP_YAW && fabsf(in.v) <= PIVOT_RADIUS * fabsf(in.yawRate);
}

// After a pivot: the yaw rate under the stop limit (the rover has stopped turning), or STOP_TIME at the latest.
bool Autopilot::settled(const Inputs& in) {
  settleT_ += in.dt;
  return fabsf(in.yawRate) < STOP_YAW || settleT_ > STOP_TIME;
}

bool Autopilot::stopped(const Inputs& in) {
  stopT_ += in.dt;
  return (fabsf(in.v) < STOP_SPEED && fabsf(in.yawRate) < STOP_YAW) || stopT_ > STOP_TIME;
}

void Autopilot::startPivot(float angle, int8_t dir) {
  pivTarget_ = angle;
  pivDir_ = dir;
  pivTurned_ = pivNet_ = 0;
  pivT_ = 0;
}

bool Autopilot::pivotStep(const Inputs& in, Output& o) {
  pivNet_ += in.yawRate * in.dt;   // net angle: gyro noise averages out (a sum of |rate| counts it as turning)
  pivTurned_ = fabsf(pivNet_);
  pivT_ += in.dt;
  const float left = pivTarget_ - pivTurned_;
  const float limit = 3.0f + pivTarget_ / (PIVOT_MIN * cfg.maxYawRate);  // a pivot that cannot turn ends anyway
  if (left <= 0.01f || pivT_ > limit) { o.v = 0; o.w = 0; return true; }
  float w = sqrtf(2 * YAW_ACCEL * left);
  w = fminf(cfg.maxYawRate, fmaxf(PIVOT_MIN * cfg.maxYawRate, w));
  o.v = 0;
  o.w = pivDir_ * w;
  return false;
}

// Polarity: the wire x sign (+1 left) = inside x direction (inside the loop, travelling clockwise: the wire is on the
// left). 0 when either fact is not known.
int8_t Autopilot::polSide(const Inputs& in) const {
  return (in.inside != 0 && in.heading != 0) ? (int8_t)(in.inside * in.heading) : 0;
}

// Mirror bounce from one reading: leave the wire at the wire angle on the other side, plus a random spread, kept at
// least the minimum leave angle from the wire; turn away from the wire; the shorter way round.
void Autopilot::planBounce() {
  const float psi = planPsi_;
  float leave = psi + (2 * rnd() - 1) * cfg.bounceSpreadDeg * D2R;
  const float mn = cfg.bounceMinDeg * D2R;
  leave = fmaxf(mn, fminf(PI_F - mn, leave));
  planLeave_ = leave;
  float T = psi + leave;
  int8_t dir = (int8_t)-planSide_;
  if (T > PI_F) { T = 2 * PI_F - T; dir = (int8_t)-dir; }
  planTurn_ = T;
  planDir_ = dir;
}

// Join from one reading: the estimated wire line in the goal direction, turned by the approach angle toward the
// wire. g: the side the wire keeps when the rover travels the goal way (clockwise inside: the wire on the left).
void Autopilot::planJoin() {
  const int8_t g = (int8_t)((cfg.goalCW ? 1 : -1) * planInside_);
  // The wire direction against the centreline: turned by the heading away from the wire side when closing in, toward
  // it when moving away (the closest-approach reading comes after the rover starts to move away)
  const float tangent = -planSide_ * planToward_ * planPsi_ + (planSide_ == g ? 0 : PI_F);
  planRot_ = wrapPi(tangent + g * cfg.approachAngleDeg * D2R);
}

Output Autopilot::stepStates(const Inputs& in) {
  Output o;
  read(in);
#ifdef ROVER_WIRE_SOLVE
  // Every frame in both roam modes (the join pivots by the solve's angle, basicsJoin); one solve per buffer sample (10 Hz)
  {
    if (!wfb_) {   // the first frame of a run that uses the solve: its memory from the heap
      wfb_ = new (std::nothrow) wirefit::WireFitBounce();
      if (!wfb_) { toFault(FAULT_NO_MEMORY); return Output(); }
#ifdef ROVER_WIRE_SOLVE_BG0
      wfb_->bc.bgField = false;
#endif
      wTotal_ = -1;
    }
    if (leadWant_ != lead_) leadApply();
    if (wTotal_ < 0) {   // the first frame: the height starts at the setting, spread by the wire extra depth
      wfb_->bc.hStart = cfg.sensorHeight;
      wfb_->bc.hStartSd = cfg.wireMaxDepth;
      wfb_->reset();
    }
    // the gyro input reads clockwise positive (the reference rover logs; the emulator copies it); the solve takes
    // counterclockwise
    wfb_->frame(in.v * in.dt, -in.yawRate * in.dt, in.inRange && in.R > 0, in.x, in.y, in.R);
    solveStep();
  }
#endif
  // The wire side and the radius median: basics, and the estimator return join (it is the basics join)
  if (basics() || kWireSolve) {
    bHist_[bI_] = (in.inRange && in.R > 0) ? in.R : B_LOST_FAR;
    bI_ = (bI_ + 1) % B_MED;
    if (bN_ < B_MED) bN_++;
    basicsSide(in);
  }
  if (state == FAULT) return o;

  // The pivot rule (settleStart): a commanded pivot last step, the end of one, or a measured turn on the spot
  if (settle_ && settled(in)) settle_ = false;
  wasHold_ = hold_;
  hold_ = (!last_.external && last_.v == 0.0f && last_.w != 0.0f) || settle_ || onSpot(in);

  // Every state
  if (in.stuck && in.inRange && !hold_) { toFault(FAULT_STUCK); return o; }
  lostT_ = in.inRange ? 0 : lostT_ + in.dt;
  if (cfg.lostTimeout > 0 && lostT_ > cfg.lostTimeout && !hold_ && (state == JOIN || state == FOLLOW || state == DEPART)) {
    toFault(FAULT_LOST);
    return o;
  }
  // Lost in FOLLOW or DEPART for rover_no_tx_s: one 30 deg turn in place (alternate sides). Square-on over
  // the wire the coils sit in the field null; the turn ends it. In place, so safe with the Tx off too.
  const bool wireState = (state == FOLLOW || state == DEPART) && !bb_ && !sweeping();   // basics: stop while lost, no search
  if (in.inRange) { searched_ = searching_ = false; }
  else if (wireState) {
    if (searching_) {
      Output so;
      if (pivotStep(in, so)) { searching_ = false; settleStart(); so = Output(); }
      return so;
    }
    if (!searched_ && lostT_ > cfg.noTxS && !hold_) {
      searched_ = searching_ = true;
      startPivot(30 * D2R, searchDir_);
      searchDir_ = (int8_t)-searchDir_;
      return o;
    }
  }
  // No Tx signal: ROAM and RETURN wait the roam limit (far from the wire the signal can drop out)
  const float noTxLimit = (state == ROAM || state == RETURN) ? cfg.noTxRoamS : cfg.noTxS;
  if (in.txOffS > noTxLimit && (txWait || !hold_)) {
    if (!txWait) decide(DEC_TX_OFF, 0, 0);
    txWait = true;
    return o;
  }
  if (txWait) { txWait = false; decide(DEC_TX_ON, 0, 0); }
  if (in.docked && state != DOCK && !hold_) {
    if (bb_) { bb_ = false; state = ROAM; }
    dockFrom_ = state;
    state = DOCK;
    sub = SUB_NONE;
    decide(DEC_DOCK, 0, 0);
  }

  const float approachV = cfg.approachFraction * cfg.roamSpeed;
  const float H = cfg.sensorHeight;
  if (bb_) return basicsStep(in);

  // Probe: a straight drive of the probe length on the line of the approach (the fit keeps the approach frames), then
  // the decision that asked for it
  if (sub == SUB_PROBE) {
    probeLeft_ -= fabsf(in.v) * in.dt;
    if (probeLeft_ > 0) { o.v = probeDir_ * approachV; return o; }
    decide(DEC_PROBE, 0, 0);
    if (dec.fit.valid) dec.side = (int8_t)(dec.fit.dNow < 0 ? -1 : 1);
    sub = SUB_PROBE_STOP;
    stopT_ = 0;
    return o;
  }
  if (sub == SUB_PROBE_STOP) {
    if (!stopped(in)) return o;
    if (probeFor_ == PROBE_BOUNCE) { bounceNow(in); return o; }
    if (probeFor_ == PROBE_JOIN) { joinNow(in); return o; }
    startDone(in, o);
    return o;
  }

  switch (state) {
    case ROAM:
    case RETURN: {
      if (sub == SUB_START) {
        // Forward only, and only when it cannot reach the wire (a reverse backs over the wire when the nose points away)
        if (rd.valid && cfg.probeLength > 0 && rd.d > cfg.probeLength) { probeBegin(PROBE_START, o, approachV); return o; }
        if (startDone(in, o)) return o;
      }
      // Loud and early: the rover starts inside, so polarity "outside" before the first turn means the Tx direction
      // setting is the wrong way round (a silent inversion would bounce it out of the loop)
      if (startCheck_ && in.inside < 0 && !hold_) { toFault(FAULT_POL_START); return Output(); }
      if ((cfg.followOnly || in.needCharge) && state == ROAM && !hold_) { state = RETURN; decide(DEC_RETURN, 0, 0); }
      if (state == ROAM && timed()) {
        roamT_ += in.dt;
        if (roamT_ >= cfg.roamTimeS && !hold_) { state = RETURN; followT_ = 0; decide(DEC_RETURN, 0, 0); }
      }
      // Beacon re-arm: past the beacon re-arm distance
      if (!beaconArmed_ && in.beaconRange > cfg.beaconRearm) beaconArmed_ = true;
      switch (sub) {
        case SUB_BSTOP:
          if (!stopped(in)) return o;
          if (twoWay()) {   // change the drive direction, the nose turned by the spread
            const float sp = (2 * rnd() - 1) * cfg.beaconSpreadDeg * D2R;
            decide(DEC_BEACON, sp / D2R, 0);
            reverseTurn(sp, false);
            return o;
          }
          {
            float T = PI_F + (2 * rnd() - 1) * cfg.beaconSpreadDeg * D2R;
            int8_t dir = rnd() < 0.5f ? 1 : -1;
            if (T > PI_F) { T = 2 * PI_F - T; dir = (int8_t)-dir; }
            startPivot(T, dir);
            decide(DEC_BEACON, dir * T / D2R, 0);
          }
          sub = SUB_BPIVOT;
          return o;
        case SUB_BPIVOT:
          if (pivotStep(in, o)) {
            sub = SUB_NONE;
            armed_ = joinArmed_ = false;  // as after a bounce turn
            settleStart();
            o = Output();
          }
          return o;
        case SUB_STOP:
          if (!stopped(in)) return o;
          if (reverseBounce()) {
            // Reverse bounce: the same decision; the rover changes its drive direction and sweeps to the planned heading
            // (the travel direction, rover frame). No solve answer: straight back the way it came
            float nose = 0;
            if (dec.solveOk || !kWireSolve) nose = wrapPi(planDir_ * planTurn_ + (dir_ > 0 ? PI_F : 0.0f));
            reverseTurn(nose, true);
            return o;
          }
          startPivot(planTurn_, planDir_);
          sub = SUB_PIVOT;
          return o;
#ifdef ROVER_WIRE_SOLVE
        case SUB_ENTRY_ARC:
          // The probe on entry: the arc ends early at the bounce point, outside or out of range; the roam goes on
          if (!rd.valid || in.inside < 0) { sub = SUB_NONE; break; }
          {
            wirefit::BounceResult ws;
            if (solveNow(ws) && ws.d <= cfg.bounceDistance) { sub = SUB_NONE; break; }
          }
          if (arcStep(in, o)) { sub = SUB_NONE; o.v = dir_ * approachV; o.w = 0; }
          return o;
#endif
        case SUB_SWEEP:
          if (arcStep(in, o)) {
            sub = SUB_NONE;
            armed_ = joinArmed_ = false;
            rearmStart();
            bPeak_ = bMedian();
            bPeakX_ = -1e9f;
            if (swBounce_) { startCheck_ = false; bounces++; }
            o = Output();
          }
          return o;
#ifdef ROVER_WIRE_SOLVE
        case SUB_BOUNCE_WAIT:   // the bounce point: stop, then decide on the solve of the last sample
          bounceWaitT_ += in.dt;
          if (!stopped(in)) return o;
          // 5 s: well over the longest measured solve (262 ms of CPU) at the 41 % of the CPU the loop leaves free with
          // Wi-Fi and both sensors (bench board); past 5 s the bounce goes on with the last answer
          if (!solveSettled() && bounceWaitT_ < 5.0f) return o;
          bounceNow(in);
          return o;
#endif
        case SUB_PIVOT:
          if (pivotStep(in, o)) {
            sub = SUB_NONE;
            armed_ = joinArmed_ = false;
            settleStart();
            startCheck_ = false;
            bounces++;
            o = Output();
          }
          return o;
        default:
          break;
      }
      // The pivot rule: stand still while the turn lasts; then the re-arm starts from this reading (settleStart)
      if (hold_) return o;
      if (wasHold_) { rearmStart(); bPeak_ = bMedian(); bPeakX_ = -1e9f; }
      // Driving straight. Polarity says outside the loop: no bounce until inside again.
      // Outside: turn round, and again each time the distance grows by the re-arm drop since the last turn
      // A square-on approach to the wire reads a false rise in the null band (emulator: a turn-back 6 cm short of the
      // wire every 4.6 s for 6 min, never a crossing): no rise counts there. A depart that
      // ends outside has already turned toward the inside (bGoalSense), so it counts as the turn (basicsStep)
      const bool nullBand = in.inRange && fabsf(in.y) < cfg.sensorHeight * sinf(B_NULL_DEG * D2R);
      if (in.inside < 0) {
        if (!outsideTurned_ || (rd.valid && !nullBand && rd.df > outsideD_ + cfg.rearmDrop)) {
          outsideTurned_ = true;
          outsideD_ = rd.valid ? rd.df : 0;
          decide(DEC_TURN_BACK, 180, 0);
          insideRun_ = 0;
          if (twoWay()) {   // a sweeping mode: change the drive direction, no turn
            dir_ = (int8_t)-dir_;
            leadPick();
            o.v = dir_ * approachV;
            return o;
          }
          startPivot(PI_F, 1); sub = SUB_BPIVOT; return o;
        }
        if (rd.valid) outsideD_ = fminf(outsideD_, rd.df);
        o.v = dir_ * approachV;
        return o;
      }
      // A sweeping mode turns back by a change of drive direction, so the rover stays at the wire, where polarity
      // flickers: the turn-back re-arms after the re-arm drop of travel with polarity inside (emulator: 137 turn-backs
      // at the wire and an escape when each inside reading re-armed it)
      if (in.inside > 0) {
        insideRun_ += fabsf(in.v) * in.dt;
        if (!twoWay() || insideRun_ >= cfg.rearmDrop) outsideTurned_ = false;
      }
      if (cfg.beaconBounce && beaconArmed_ && in.beaconRange <= cfg.beaconDistance) {
        beaconArmed_ = false;
        sub = SUB_BSTOP;
        stopT_ = 0;
        return o;
      }
      if (!armed_) entryDone_ = false;   // a new approach: its own probe on entry
      if (!armed_ || !joinArmed_) {
        if (!rd.valid) armed_ = joinArmed_ = true;
        else {
          // The radius passes its highest value since the turn and falls by the re-arm fall: the rover closes on the
          // wire again. The radius only: no height, no fixed distance, so any track shape, leave angle or wire depth
          rMax_ = fmaxf(rMax_, rd.Rs);
          const bool closing = rd.Rs < rMax_ - cfg.rearmFall;
          if (closing) armed_ = true;
          if (rd.df > cfg.approachDistance + 0.1f || closing) joinArmed_ = true;  // the join starts on entry: closing in
        }
      }
#ifdef ROVER_WIRE_SOLVE
      // A sweeping mode: every join is the planned sweep join (Join then leave: each arrival; Reverse bounce: the
      // return, from forward travel only: a reverse leg bounces first, and the bounce leaves forward). Reverse bounce
      // with no solve answer goes on to the bounce
      if (sweeping() && (basics() || (state == RETURN && dir_ > 0))) {
        // the join arm too (the radius falls again after a turn): 1 s after a Reverse bounce, a fresh solve over
        // 0.3 m of drive away from the wire read closing at 1 m and joined 1.5 m from the wire (emulator)
        if (dir_ > 0 && joinArmed_ && sweepJoinDue(in)) {
          bLong_ = state == RETURN;
          basicsJoin(in, o);
          return o;
        }
        wirefit::BounceResult ws;
        const bool solved = solveNow(ws) && ws.n >= SJ_MIN_N;
        // Join then leave in reverse, or with no solve answer yet (the run start): at the plan's square-on join
        // distance (the solve's, else by the Rx radius), closing, the rover drives back the way it came; it joins on
        // a forward arrival only (a join planned from a reverse arrival ran against the goal direction, emulator).
        // Reverse bounce goes on to its bounce
        const float dW = solved ? ws.d : sqrtf(fmaxf(0.0f, rd.Rf * rd.Rf - H * H));
        if (basics() && (dir_ < 0 || !solved) && joinArmed_ && rd.valid && (!solved || dir_ * ws.betaDeg < 0)
            && dW <= cfg.followInset + cfg.turnRadius + 0.3f * cfg.roamSpeed) {
          dir_ = (int8_t)-dir_;
          leadPick();
          joinArmed_ = false;
          rearmStart();
          decide(DEC_BOUNCE, 180, 0);
          o.v = dir_ * approachV;
          return o;
        }
        if (basics() || solved) {
          o.v = dir_ * ((!basics() && rd.valid && rd.df <= cfg.approachDistance) ? approachV : cfg.roamSpeed);
          return o;
        }
      }
#endif
      // Bounce style basics: a timed or charge return joins the same way as a bounce, and its follow is the long follow
      if (basics() && state == RETURN) {
        const float medR = bMedian();
        if (medR > bPeak_) bPeak_ = medR;
        const float dJ = sqrtf(fmaxf(0.0f, medR * medR - cfg.sensorHeight * cfg.sensorHeight));
        if (in.inRange && bP_ * in.x > 0 && medR <= bPeak_ - cfg.rearmFall && dJ <= bJoinDist()) {
          bLong_ = true;
          basicsJoin(in, o);
          return o;
        }
        o.v = cfg.roamSpeed;
        return o;
      }
#ifdef ROVER_WIRE_SOLVE
      // Estimator return join with the solve, once the solve puts the wire inside the join distance and the rover
      // closes on it: stop and pivot to the goal direction (basicsJoin). Its follow and depart are the basics ones.
      if (state == RETURN) {
        wirefit::BounceResult ws;
        if (joinArmed_ && in.inRange && solveNow(ws) && ws.d <= bJoinDist() && ws.betaDeg < 0) {
          bLong_ = true;
          basicsJoin(in, o);
          return o;
        }
      } else
#endif
      if (state == RETURN && joinArmed_ && rd.valid && rd.df <= cfg.approachDistance) {
        state = JOIN;
        sub = SUB_APPROACH;
        lastDf_ = rd.df;
        decide(DEC_JOIN, 0, 0);
        o.v = approachV;
        return o;
      }
      // Bounce style basics: the join point replaces the bounce point (basicsStep). The radius, median of 5, passes its
      // highest value since the last turn and falls by the re-arm fall (the rover closes on the wire again), inside the
      // join distance (horizontal, from the radius and the height setting), with the rover inside (no re-arm while outside). The side distance
      // reading (the Rx x toward the inside) must also fall by the re-arm fall below its highest value since the turn:
      // inside a small loop the field of the whole loop adds and the radius falls on the way to the middle (emulator,
      // the 8 m loop: a join 1.9 m from the wire with the rover driving away from it, then an escape)
      if (basics() && state == ROAM) {
        const float medR = bMedian();
        if (medR > bPeak_) bPeak_ = medR;
        const float xs = bP_ * in.x;
        if (in.inRange && xs > bPeakX_) bPeakX_ = xs;
        const float dJ = sqrtf(fmaxf(0.0f, medR * medR - cfg.sensorHeight * cfg.sensorHeight));
        if (in.inRange && xs > 0 && medR <= bPeak_ - cfg.rearmFall && xs <= bPeakX_ - cfg.rearmFall
            && dJ <= bJoinDist()) {
          bLong_ = false;
          basicsJoin(in, o);
          return o;
        }
        o.v = cfg.roamSpeed;   // roam is a straight drive
        return o;
      }
      // Bounce distance from the radius and the height setting. The offset x reads high near the wire (the rest of the
      // loop adds field); the radius does not. A height error of 120 mm moves the bounce point by under 6 cm at 0.5 m.
      float dR = rd.valid ? sqrtf(fmaxf(0.0f, rd.Rf * rd.Rf - cfg.sensorHeight * cfg.sensorHeight)) : 99;
#ifdef ROVER_WIRE_SOLVE
      {   // the solve's distance (its own height, no height setting) while it has an answer
        wirefit::BounceResult ws;
        if (rd.valid && solveNow(ws)) {
          dR = ws.d;
          // A return drives on to its join, which lies inside the bounce distance (join distance 0.39 m against
          // bounce 0.56 m on the reference rover): no bounce while the solve has an answer (without one, the bounce and
          // its full turn stay)
          if (state == RETURN && dir_ > 0) dR = 99;
        }
      }
#endif
#ifdef ROVER_WIRE_SOLVE
      if (entryProbe(in)) return o;   // Bounce and Reverse bounce: the probe on entry
#endif
      if (armed_ && rd.valid && dR <= cfg.bounceDistance) {
#ifdef ROVER_WIRE_SOLVE
        bounceBegin();  // stop; the decision follows on the settled solve (SUB_BOUNCE_WAIT)
#else
        bounceNow(in);  // the fitted heading, or square-on with no fit
#endif
        return o;
      }
      o.v = dir_ * ((rd.valid && rd.df <= cfg.approachDistance) ? approachV : cfg.roamSpeed);
      return o;
    }

    case JOIN: {
      // Drive straight to the wire; on it, turn along the wire in the goal direction by the fitted heading; FOLLOW.
      switch (sub) {
        case SUB_STOP:
          if (!stopped(in)) return o;
          startPivot(fabsf(planRot_), planDir_);
          sub = SUB_PIVOT;
          return o;
        case SUB_PIVOT:
          if (pivotStep(in, o)) {
            state = FOLLOW;
            sub = SUB_NONE;
            settleStart();
            wrongCount_ = 0; entrySeen_ = false; exitArmed_ = !in.exitBeacon; reverseArm_ = 0;
            decide(DEC_FOLLOW, 0, 0);
            o = Output();
            o.external = true;
          }
          return o;
        default: {
          if (hold_) return o;  // the pivot rule: no join decision while the rover turns on the spot
          // The wire under the sensor: close by the radius, or lost right after a close reading (the field null)
          // or polarity says outside (the rover crossed the wire)
          const bool onWire = in.inside < 0 ||
              (rd.valid ? (rd.d <= 0.5f * H || fabsf(rd.X) <= CAPTURE_OFFSET) : (lastDf_ <= 2 * H));
          if (rd.valid) lastDf_ = rd.d;
          // The Rx x sign 1-3 sensor heights from the wire (x is large there; the Rx state holds on a straight run)
          if (rd.valid && rd.d >= H && rd.d <= 3 * H) xSignNear_ = in.x < 0 ? -1 : 1;
          if (!onWire) { o.v = approachV; return o; }  // lost far from the wire: drive on (Tx off stops by the no-Tx rule)
          if (in.inside != 0) planInside_ = in.inside;
          else if (cfg.joinNoPolarityStop) { toFault(FAULT_NO_POLARITY); return Output(); }
          else planInside_ = 1;
          planFromFit(in);
          // The follower's x sign from known geometry: the wire was on side planSide_ (left +) on the way in, so raw x
          // matches it when xSignNear_ == planSide_; on the wire it is below, so the sign of y now marks that Rx state
          if (xSignNear_ != 0 && rd.valid && fabsf(in.y) > fabsf(in.x))
            yBelow = (int8_t)((in.y < 0 ? -1 : 1) * (xSignNear_ == planSide_ ? 1 : -1));
          xSignNear_ = 0;
          const int8_t g = (int8_t)((cfg.goalCW ? 1 : -1) * planInside_);
          planRot_ = wrapPi(-planSide_ * planToward_ * planPsi_ + (planSide_ == g ? 0 : PI_F));
          planDir_ = fabsf(planRot_) > 1e-3f ? sgn(planRot_) : 1;
          decide(DEC_JOIN, planRot_ / D2R, planSide_);
          startCheck_ = false;
          sub = SUB_STOP;
          stopT_ = 0;
          return o;
        }
      }
    }

    case FOLLOW: {
      if (sub == SUB_REVERSE_STOP) {
        if (!stopped(in)) return o;
        startPivot(PI_F, cfg.goalCW ? 1 : -1);  // turn round on the wire, the nose over the inside
        sub = SUB_REVERSE;
        return o;
      }
      if (sub == SUB_REVERSE) {
        if (pivotStep(in, o)) {
          sub = SUB_NONE;
          settleStart();
          turnarounds++;
          reverseArm_ = REVERSE_ARM;
          wrongCount_ = 0;
          entrySeen_ = false;
          exitArmed_ = false;
          o = Output();
        }
        return o;
      }
      if (!rd.valid) return o;  // lost: stop (one turn out of the field null after rover_no_tx_s)
      if (hold_) { o.external = true; return o; }  // the pivot rule: no wrong-direction count, no follow end
      reverseArm_ = fmaxf(0.0f, reverseArm_ - fabsf(in.v) * in.dt);
      if (in.entryBeacon) entrySeen_ = true;
      if (!in.exitBeacon) exitArmed_ = true;
      if (cfg.wrongDirCheck && reverseArm_ <= 0) {
        const int8_t goal = cfg.goalCW ? 1 : -1;
        if (in.heading == -goal && fabsf(in.v) > STOP_SPEED) wrongCount_++;
        else wrongCount_ = 0;
        const bool beaconWrong = in.exitBeacon && exitArmed_ && !entrySeen_;
        if (wrongCount_ >= WRONG_FRAMES || beaconWrong) {
          decide(DEC_REVERSE, 180, 0);
          sub = SUB_REVERSE_STOP;
          stopT_ = 0;
          return o;
        }
      }
      // Timed roam and follow: the follow time is up (a charge return follows on to the dock)
      if (timed() && !in.needCharge) {
        followT_ += in.dt;
        if (followT_ >= cfg.followTimeS) {
          decide(DEC_FOLLOW_END, 0, 0);
          state = DEPART;
          sub = SUB_TURN_STOP;
          stopT_ = 0;
          return o;
        }
      }
      o.external = true;
      return o;
    }

    case DOCK:
      if (in.docked) return o;
      charges++;
      roamT_ = followT_ = 0;  // timed roam and follow: both times start again after a charge
      if (cfg.followOnly) { state = FOLLOW; sub = SUB_NONE; reverseArm_ = REVERSE_ARM; o.external = true; return o; }
      if (dockFrom_ == ROAM || dockFrom_ == RETURN) { state = dockFrom_; sub = SUB_NONE; return o; }  // not a wire dock
      state = DEPART;
      sub = SUB_NONE;
      departLeft_ = cfg.departMin + (fmaxf(cfg.departMax, cfg.departMin) - cfg.departMin) * rnd();
      decide(DEC_DEPART, 0, 0);
      o.external = true;
      return o;

    case DEPART: {
      if (sub == SUB_TURN_STOP) {
        if (!stopped(in)) return o;
        // Each change from follow to roam (a follow end, or after a charge): the leave angle, then the bounce spread and
        // minimum as at a bounce
        const float mn = cfg.bounceMinDeg * D2R;
        float turn = cfg.followExitDeg * D2R + (2 * rnd() - 1) * cfg.bounceSpreadDeg * D2R;
        turn = fmaxf(mn, fminf(PI_F - mn, turn));
        if (sweeping()) arcStart(turn, cfg.goalCW ? -1 : 1, 1, cfg.turnRadius);
        else startPivot(turn, cfg.goalCW ? -1 : 1);  // goal clockwise: the inside is on the right
        sub = SUB_TURN;
        return o;
      }
      if (sub == SUB_TURN) {
        if (sweeping() ? arcStep(in, o) : pivotStep(in, o)) {
          state = ROAM;
          sub = SUB_NONE;
          roamT_ = followT_ = 0;
          armed_ = joinArmed_ = false;
          settleStart();
          o = Output();
        }
        return o;
      }
      if (!rd.valid) return o;  // lost: stop and wait
      if (hold_) { o.external = true; return o; }  // the pivot rule: no depart turn
      if (!in.onContacts) departLeft_ -= fabsf(in.v) * in.dt;  // counted off the dock contacts
      if (departLeft_ <= 0) { sub = SUB_TURN_STOP; stopT_ = 0; return o; }
      o.external = true;
      return o;
    }

    case FAULT:
      return o;
  }
  return o;
}

// ===== Bounce style basics =====
// From the PC simulation (join on
// the approach fit, turn sooner, square-on guard re-armed as the roam re-arms), with these differences:
//   - the depart starts only once the rover no longer closes on the wire: the follow has brought the heading to
//     parallel, so the planned angle counts from the wire;
//   - turns end on the measured turn (the gyro, else the command), not on time alone;
//   - the inside filter (src/main.cpp) sets the wire side in ROAM when it knows it, in place of the raw polarity.
// All driven at the approach speed; full steering = the full yaw rate (steering_rate_max_dps). Turn radius R =
// approach speed / full yaw rate. A turn from an angle a to parallel moves the rover R (1 - cos a) toward the wire.

float Autopilot::bMedian() const {
  float a[B_MED];
  const int n = bN_;
  for (int i = 0; i < n; i++) a[i] = bHist_[i];
  for (int i = 1; i < n; i++) {
    const float v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return n ? a[n / 2] : B_LOST_FAR;
}

// The full turn rate: measured (bTurnMeasure) once the rover has driven a full-rate turn; before that the setting
// (max steering rate). The steering rate loop reaches less than the commanded rate (the reference rover's lawn logs, 4 runs,
// full-rate departs: gyro median 52 deg/s against 90 commanded; the emulator 53).
float Autopilot::bYawRate() const { return bWMeas_ > 0 ? bWMeas_ : cfg.maxYawRate; }

// The join turn radius: the join speed over the full turn rate (a skid steer). The first join of a run (before the
// depart measures the turn rate) runs faster (the reference rover: 90 deg/s set, 52 deg/s true, radius 0.32 m against
// 0.185 m)
float Autopilot::bJoinRadius() const {
  const float w = bYawRate();
  return w > 1e-3f ? bJoinSpeed() / w : 1e3f;
}

// The turn radius at the approach speed (the follow and the depart), from the full turn rate
float Autopilot::bTurnRadius() const {
  const float v = cfg.approachFraction * cfg.roamSpeed, w = bYawRate();
  const float r = w > 1e-3f ? v / w : 1e3f;
  return sweeping() && cfg.turnRadius > r ? cfg.turnRadius : r;
}

// The join speed (a skid steer turns almost on the spot; in the join turn it slows so its turn
// fits inside the follow inset): the speed whose turn radius at the full turn rate is half the follow inset, so the
// worst join turn (nearly 180 deg) spans the inset; at most the approach speed. A least radius of 5 cm keeps the rover
// moving with a follow inset of 0
float Autopilot::bJoinSpeed() const {
  const float v = cfg.approachFraction * cfg.roamSpeed;
  const float vj = bYawRate() * 0.5f * bJoinRoom();
  return vj < v ? vj : v;
}

// The room for the worst join turn: the follow inset, and at least the deepest wire distance (sensor height + wire
// extra depth, maximum): nearer, a square-on approach reads the wire lost (follow inset 0: the rover drove over the wire
// with no join in every basics run, emulator)
float Autopilot::bJoinRoom() const {
  const float hd = JOIN_ROOM_K * (cfg.sensorHeight + cfg.wireMaxDepth);
  return cfg.followInset > hd ? cfg.followInset : hd;
}

// In the depart (a full-rate turn at the approach speed), from 0.3 s in (the rate has settled): the measured yaw rate
// (gyro), low-passed over about 10 readings
void Autopilot::bTurnMeasure(const Inputs& in) {
  const float w = fabsf(in.yawRate);
  if (bT_ < 0.3f || w < 0.1f) return;
  bWMeas_ = bWMeas_ > 0 ? bWMeas_ + 0.1f * (w - bWMeas_) : w;
}

// Three-point turn (a car may reverse when needed): when the rest of the join turn does
// not fit before the follow line, reverse with the turn the same way round (a car: the opposite steering), until it
// fits, then forward into the join again. The rest of the turn closes R (1 - cos a) with the goal direction and R (1 +
// cos a) against it (it passes square-on); a: the closing angle, from the side distance rate against the measured
// speed; with or against from the wire side of the heading (square-on, not readable: R). The forward leg starts again
// once the room exceeds the need by the closing in the rate filter delay. A skid steer joins on a turn of half the
// follow inset from a start at the inset + 2 radii, so it never needs it. Returns true in the reverse leg
// 0.7.1: every vehicle joins on the spot (basicsJoin); the arc join and this reverse leg run only in a build without
// the wire solve
bool Autopilot::bThreePoint(const Inputs& in, float xs) {
  const float sp = fabsf(in.v);
  if (sp < 0.05f) return bRev_;   // the speed changes sign: no closing angle reading
  const float sa = fminf(1.0f, fabsf(bRate_) / sp), c = sqrtf(1.0f - sa * sa);
  const int8_t side = bWireSide(in);
  const float Rj = bJoinRadius(), room = xs - cfg.followInset;
  const float need = side == 0 ? Rj : Rj * (side == (int8_t)bGoalSense() ? 1 + c : 1 - c);
  const float delay = cfg.approachFraction * cfg.roamSpeed / (2 * PI_F * B_RATE_HZ);
  if (!bRev_ && in.v > 0 && bRate_ < 0 && room < need) { bRev_ = true; bClosed_ = false; }
  else if (bRev_ && in.v < 0 && room >= need + delay) bRev_ = false;
  return bRev_;
}

// Join distance (the join started too far out; the turn ends on the follow line, not
// across the wire): the room for the worst join turn and the follow line. The worst arrival runs nearly parallel to
// the wire against the goal direction; its join turn of nearly 180 deg closes two join turn radii: follow inset + 2 x
// the radius at the join speed (the reference rover: 2 x the inset, 0.6 m). So the join needs no angle reading.
// The estimator stops and pivots: follow inset + R as before. The
// approach distance no longer sets the join point; inside a loop the Rx radius reads about R / pi near the loop middle
// (1.27 m in an 8 m loop), so a join at the approach distance fired anywhere inside.
float Autopilot::bJoinDist() const {
  if (!basics()) return cfg.followInset + bTurnRadius();
  // At least the join room (sensor height + wire extra depth): nearer, a square-on approach reads the distance flat (the
  // field null; emulator, 12 m circle at a 90 deg depart: least reading 0.27 m against a join distance of 0.27 m from
  // the measured join radius, no join, the rover drove over the wire). The join then ends parallel and short of the
  // wire, and the follow slides onto it
  // The larger of the two, not the sum: the sum (0.74 m on the reference skid steer) started the join too far out in the
  // small loops (emulator: 11 escapes and 31 lost faults in 114 runs). A turn that does not fit takes the three-point
  // turn (bThreePoint)
  const float turn = 2 * bJoinRadius(), room = bJoinRoom();
  return cfg.followInset + (turn > room ? turn : room);
}

// Largest closing angle (deg) from which the rover still turns to parallel by the follow line, from the side distance
// xs: room (xs - inset) >= R (1 - cos a); at most the approach angle. Inside a loop xs under-reads the true distance,
// so the limit is on the safe side.
float Autopilot::bCloseDeg(float xs) const {
  const float room = xs - cfg.followInset, Rt = bTurnRadius();
  if (room <= 0) return 0;
  const float c = 1 - room / Rt, a = c <= -1 ? 180.0f : acosf(c) / D2R;
  return a < cfg.approachAngleDeg ? a : cfg.approachAngleDeg;
}

// The side distance low-passed, and its rate (both at B_RATE_HZ); returns the low-passed side distance
float Autopilot::bRate(float xs, float dt) {
  if (bHavePrev_ && dt > 0) {
    const float k = 1 - expf(-2 * PI_F * B_RATE_HZ * dt), prev = bXsF_;
    bXsF_ += k * (xs - bXsF_);
    bRate_ += k * ((bXsF_ - prev) / dt - bRate_);
  } else bXsF_ = xs;
  bHavePrev_ = true;
  return bXsF_;
}

// Wire side. The Rx x sign changes only when the sensor crosses the wire, or when the Rx restarts its quadrant tracking
// (after a loss). bP_ is the x sign while inside; bM_ the side held since the signal was lost (+1 inside). At each
// regain bP_ = bM_ x the sign of x. An x sign change with |x| beyond the crossing radius on both frames is an Rx restart
// (the inside filter's crossing radius, src/main.cpp). In ROAM the inside filter, when it knows the side and |x| is off
// its blind line, sets bP_. bS_: the sign of y with the wire below, followed where |y| carries it (25 % of the radius).
void Autopilot::basicsSide(const Inputs& in) {
  if (in.inRange && in.R > 0) {
    const float hd = cfg.sensorHeight + cfg.wireMaxDepth, m = 0.26f * hd + cfg.roamSpeed * in.dt + 0.1f;
    const float rCross = sqrtf(hd * hd + m * m);
    if (bWasLost_) { bP_ = (int8_t)(bM_ * sgn(in.x)); bS_ = 1; }
    else if (sgn(in.x) != sgn(bXLast_) && fminf(fabsf(in.x), fabsf(bXLast_)) > rCross) bP_ = (int8_t)(bM_ * sgn(in.x));
    bXLast_ = in.x;
    if (fabsf(in.y) >= B_YREL * in.R && in.y * bS_ < 0) bS_ = (int8_t)-bS_;
    if (state == ROAM && !bb_ && in.inside != 0 && fabsf(in.x) >= B_YREL * in.R) bP_ = (int8_t)(in.inside * sgn(in.x));
    if (in.x != 0) bM_ = (int8_t)(bP_ * sgn(in.x));
  }
  bWasLost_ = !(in.inRange && in.R > 0);
#ifdef BASICS_DEBUG
  fprintf(stderr, "BS st %d P %d S %d M %d x %.3f y %.3f R %.3f in %d pol %d\n", (int)state, bP_, bS_, bM_, in.x, in.y, in.R, (int)in.inRange, (int)in.inside);
#endif
}

// The depart turn: the wire leave angle + the bounce spread, kept between the bounce minimum and 180 deg minus it
float Autopilot::bLeaveDeg() {
  const float mn = cfg.bounceMinDeg;
  const float a = cfg.followExitDeg + (2 * rnd() - 1) * cfg.bounceSpreadDeg;
  return a < mn ? mn : (a > 180 - mn ? 180 - mn : a);
}

// The side of the heading the wire lies on: +1 left, -1 right, 0 not readable (square-on or lost). The sign of the
// left/right reading against the vertical reading: both reverse with the current, so polarity does not enter
int8_t Autopilot::bWireSide(const Inputs& in) const {
  if (!in.inRange || !(in.R > 0) || fabsf(in.y) < B_SIDE_REL * in.R) return 0;
  return (in.x > 0) == (in.y > 0) ? 1 : -1;
}

// The join (every arrival joins; every follow runs in the goal direction). Basics:
// the join turns in the goal direction's sense (goal clockwise: to the right), at the full rate, until the rover no
// longer closes on the wire: it then runs along the wire with the inside on the goal side. From a heading toward the
// wire that sense is always the short way to the goal direction: 90 deg less the angle from square-on for an arrival
// with the goal direction, 90 deg plus it for one against it (it passes square-on), never over 180 deg. So the basics
// join needs no side reading. Estimator: stop and pivot in the goal
// direction's sense by the angle from the goal direction: the solve's angle to the wire line when the side reading
// says with the goal direction, else 180 deg less it (square-on: 90).
void Autopilot::basicsJoin(const Inputs& in, Output& o) {
  bb_ = true;
  state = JOIN;
  sub = SUB_NONE;
  bT_ = bTurned_ = 0;
  followT_ = 0;
  bPlanned_ = bLeaveDeg();
  bOnLine_ = bHavePrev_ = bStopped_ = false;
  bRev_ = bClosed_ = false;
  bRate_ = -bJoinSpeed();   // the rover closes at the join: the turn runs until the rate says it no longer does
  bMinXs_ = 1e9f;
  bGuardTurnT_ = 0;
  bYShot_ = bYActive_ = bGuardWas_ = false;
  bSteer_ = bGoalSense();
  bTurnDeg_ = 180;   // basics: the most the turn needs
  bPivot_ = false;
  bRateCmd_ = 0;
  sweepJoin_ = false;
  sjPhase_ = 0;
  dir_ = 1;          // the follow and the depart drive forward on sensor 1
  leadWant_ = 1;
#ifdef ROVER_WIRE_SOLVE
  if (sweeping()) {
    sweepJoin_ = true;
    sweepJoinPlan();
    bTurnDeg_ = (sjTurn1_ + sjTurn2_) / D2R;
    decide(DEC_JOIN, sjRot_ * bTurnDeg_, bP_);
    startCheck_ = false;
    if (sjPhase_ == 2) {   // one arc, on in the arrival direction (the first arc away from the wire stops first)
      o.v = sjArrive_ * arcSpeed(cfg.turnRadius);
      o.w = sjRot_ * fabsf(o.v) / cfg.turnRadius;
    }
    return;
  }
  // Join then leave on a skid steer (0.7.0): the join stops and turns on the spot by the
  // solve's angle, as the Bounce mode join. An arc join of an arrival against the goal direction turns through
  // square-on near the wire, where the reading is lost or flat, and it ended over the wire (emulator, the reference settings:
  // 3 escapes in 72 runs). Every vehicle joins this way in this build: a car (no turn on the spot) is not supported
  {
    solveDebug("join", in);
    wirefit::BounceResult ws;
    const int8_t s = bWireSide(in);
    float a = 90;
    if (solveNow(ws)) { a = fabsf(wrapPi(ws.psiDeg * D2R) / D2R); if (a > 90) a = 180 - a; }
    bTurnDeg_ = s == 0 ? 90.0f : (s == (int8_t)-bGoalSense() ? a : 180 - a);
    bPivot_ = true;
  }
#endif
  decide(DEC_JOIN, bSteer_ * bTurnDeg_, bP_);
  startCheck_ = false;
  o.v = bPivot_ ? 0 : bJoinSpeed();
  o.w = bSteer_ * cfg.maxYawRate;
}

// The turn toward the inside in a follow in the goal direction: goal clockwise, the inside is on the right (-1,
// counterclockwise positive); goal counterclockwise, on the left (+1)
float Autopilot::bGoalSense() const { return cfg.goalCW ? -1.0f : 1.0f; }

// A driven turn of bTurnDeg_: done on the measured turn, or at B_TURN_LIMIT x its time at the full rate
bool Autopilot::bTurnDone(const Inputs& in) {
  bTurned_ += fabsf(in.yawRate) * in.dt;
  const float w = bRateCmd_ > 0 && bRateCmd_ < bYawRate() ? bRateCmd_ : bYawRate();
  return bTurned_ >= bTurnDeg_ * D2R || bT_ * w >= B_TURN_LIMIT * bTurnDeg_ * D2R;
}

// Square-on guard, in JOIN and FOLLOW. Near: the side
// distance under 2 x (sensor height + wire extra depth), the least side distance of a square-on approach at the
// deepest wire. Side field small: |y| under sensor height x sin(null band) (far from a straight wire y = H x
// cos(angle to the wire)). Early check: near, closing and the side field small: steer fully toward the inside until
// the rover no longer closes; then it arms again as the roam re-arms (the median radius falls the re-arm fall below
// its highest value since). Late check, before the follow line: the side field small and the side distance risen the
// re-arm fall above its least value, that least value near: a full turn toward the inside of 90 deg minus the
// approach angle (square-on to the approach angle).
bool Autopilot::bGuard(const Inputs& in, float xs) {
  // Side field small: |y| under sensor height x sin(null band), y = H x cos(angle to the wire) for a heading within
  // the null band of square-on. At the approach angle the check held the rover off the follow line: in the emulator
  // (the reference rover, 8 m loop) y reads 0.12 m at 0.6 m from the wire near parallel and 0.02-0.05 m within 15 deg of
  // square-on, so a trip at the approach angle (0.18 m) fired near parallel and made each short follow 25-30 s
  const float nearL = 2 * (cfg.sensorHeight + cfg.wireMaxDepth);
  const bool ySmall = fabsf(in.y) < cfg.sensorHeight * sinf(B_NULL_DEG * D2R);
  if (xs < bMinXs_) bMinXs_ = xs;
  bool g = false;
  if (bHavePrev_) {
    if (bGuardTurnT_ > 0) {
      bGuardTurnT_ -= in.dt;
      if (bGuardTurnT_ <= 0) bMinXs_ = xs;
      g = true;
    } else if (!bOnLine_ && ySmall && xs > bMinXs_ + cfg.rearmFall && bMinXs_ < nearL) {
      float wg = cfg.maxYawRate;   // the guard turn's rate (a sweeping mode: at most speed / turn radius)
      if (sweeping() && cfg.approachFraction * cfg.roamSpeed / cfg.turnRadius < wg) wg = cfg.approachFraction * cfg.roamSpeed / cfg.turnRadius;
      bGuardTurnT_ = (90.0f - cfg.approachAngleDeg) * D2R / (wg > 1e-3f ? wg : 1e-3f);
      g = true;
    }
    if (!g) {
      if (!bYShot_ && !bYActive_ && bRate_ < 0 && ySmall && xs < nearL) bYActive_ = true;
      if (bYActive_) {
        if (bRate_ < 0) g = true;
        else { bYActive_ = false; bYShot_ = true; bGuardPeakR_ = bMedian(); }
      }
      if (bYShot_ && !bYActive_) {
        const float mr = bMedian();
        if (mr > bGuardPeakR_) bGuardPeakR_ = mr;
        else if (mr <= bGuardPeakR_ - cfg.rearmFall) bYShot_ = false;
      }
    }
  }
  if (g && !bGuardWas_) decide(DEC_SQUARE_ON, 0, bP_);
  bGuardWas_ = g;
  return g;
}

// JOIN: the join turn in the goal direction's sense until the rover no longer closes (basicsJoin). Inside: the goal
// side (goal clockwise: right) in JOIN, FOLLOW and DEPART. FOLLOW: drive the side distance to the follow inset; the
// closing speed is at most speed x sin(the angle the rover can still turn out of, bCloseDeg), so it never closes
// square-on; the bounce follow time runs from the follow line; then, once not closing, DEPART. Lost in JOIN or
// FOLLOW: stop and wait (the lost timeout faults); the follow time does not run while stopped. DEPART: full
// steering toward the inside by the planned angle; no reading changes it; then ROAM.
Output Autopilot::basicsStep(const Inputs& in) {
#ifdef BASICS_DEBUG
  const Output od = basicsStep2(in);
  fprintf(stderr, "BD st %d P %d S %d M %d x %.3f y %.3f R %.3f rate %.3f w %.2f on %d t %.2f\n", (int)state, bP_, bS_, bM_, in.x, in.y, in.R, bRate_, od.w, (int)bOnLine_, bT_);
  return od;
}
Output Autopilot::basicsStep2(const Inputs& in) {
#endif
  Output o;
  const float v = cfg.approachFraction * cfg.roamSpeed, wFull = cfg.maxYawRate;
  // a sweeping mode: no commanded turn tighter than the turn radius (the follow, its guards and the depart)
  const float w = sweeping() && v / cfg.turnRadius < wFull ? v / cfg.turnRadius : wFull;
  // the rate loop reaches less than a part command: the command rises by the arcs' learned ratio (arcStep), so the
  // follow turns on the radius its turning room assumes (bTurnRadius)
  const float wGain = w < wFull ? fminf(arcGain_, wFull / w) : 1.0f;
  // a sweeping mode: the turn follows the measured speed (from a stop the rover first rolls, then turns)
  const float wSpeed = sweeping() ? fabsf(in.v) / cfg.turnRadius : wFull;
  const bool lost = !(in.inRange && in.R > 0);
  const float inward = bGoalSense();   // the inside, from the goal direction (every follow runs in it)
  if (!bStopped_) bT_ += in.dt;
  switch (state) {
    case JOIN: {
      if (sweepJoin_) {   // the planned sweep join: its arcs on the measured turn, no reading changes them
        if (sjPhase_ == 1 && arcStep(in, o)) {   // the next arc changes the drive direction: it stops first
          sjPhase_ = 2;
          arcStart(sjTurn2_, sjRot_, sjArrive_, cfg.turnRadius);
          return Output();
        }
        if (sjPhase_ == 1) return o;
        if (sjPhase_ == 2 && !arcStep(in, o)) return o;
        sjPhase_ = 0;
        sweepJoin_ = false;
        state = FOLLOW; bT_ = 0; bHavePrev_ = false; bStopped_ = false;
        decide(DEC_FOLLOW, 0, bP_);
        return Output();
      }
      if (lost) { state = FOLLOW; bStopped_ = true; decide(DEC_FOLLOW, 0, bP_); return o; }
      const float xs = bRate(bP_ * in.x, in.dt);
      if (!bPivot_ && bThreePoint(in, xs)) {   // reverse leg of a three-point turn, at the approach speed
        o.v = -v;
        o.w = bSteer_ * w;
        return o;
      }
      // the join turn ends once the rover no longer closes on the wire (it runs along it), or at 180 deg; the pivot
      // on its angle. After a reverse leg the rover first closes again
      if (bRate_ < 0) bClosed_ = true;
      if ((!bPivot_ && bHavePrev_ && bClosed_ && bRate_ >= 0) || bTurnDone(in)) { state = FOLLOW; bT_ = 0; bHavePrev_ = false; decide(DEC_FOLLOW, 0, bP_); }
      o.v = bPivot_ ? 0 : bJoinSpeed();
      o.w = bSteer_ * w;
      return o;
    }
    case FOLLOW: {
      if (lost) {
        bStopped_ = true;
        // A sweeping mode: the field null over the wire (square-on) holds a stopped rover lost, and it may not turn on
        // the spot out of it (emulator: a follow lost 0.4 m from the wire waited 47 s to the lost fault). After 1 s
        // it backs straight up to 0.25 m along its own track, where it read the wire
        if (sweeping() && lostT_ > 1.0f && bBack_ < 0.25f) { bBack_ += fabsf(in.v) * in.dt; o.v = -0.5f * v; }
        return o;
      }
      bBack_ = 0;
      if (bStopped_) { bStopped_ = false; bHavePrev_ = false; }
      // Direction: the join decides it. No wire side check in the follow: at a follow inset of 0 the wire lies under
      // sensor 1 and its side of the heading flips with the Rx noise. The follow steers toward
      // the inside on the goal side, so a follow against the goal direction steers away from the inside and turns
      // round to the goal direction
      const float xs = bRate(bP_ * in.x, in.dt), e = xs - cfg.followInset;
      const float vmax = v * sinf(cfg.approachAngleDeg * D2R), vclose = v * sinf(bCloseDeg(xs) * D2R);
      float rstar = -B_WN / (2 * B_ZETA) * e;   // wanted rate of the side distance
      rstar = rstar > vmax ? vmax : (rstar < -vclose ? -vclose : rstar);
      const float wIn = 2 * B_ZETA * B_WN * (rstar - bRate_) / (v > 0.05f ? v : 0.05f);   // yaw rate toward the inside
      float steer = inward * wIn / (w > 1e-3f ? w : 1e-3f);
      steer = steer > 1 ? 1 : (steer < -1 ? -1 : steer);
      // Turn sooner: the controller turns at less than the full rate, and the closing limit assumes the full
      // rate. Within the turning room of the follow line (the turning circle from the closing angle, plus the closing
      // distance in the rate filter delay) steer fully to parallel. The closing angle uses the set speed, at most the
      // true speed, so it is never under the true angle
      // Only before the follow line is first met: on the line the closing angle is small, and the full turn at each
      // touch of the line made a limit cycle (emulator, estimator long follow: a full steering kick each 1.7 s)
      if (bRate_ < 0 && !bOnLine_) {
        const float sa = fminf(1.0f, -bRate_ / (v > 0.05f ? v : 0.05f));
        const float room = bTurnRadius() * (1.0f - sqrtf(1.0f - sa * sa)) - bRate_ / (2 * 3.14159265f * B_RATE_HZ);
        if (e <= room) steer = inward;
      }
      if (bGuard(in, xs)) steer = inward;
      // The bounce follow time starts at the follow line only (as in the simulation). A rise of the
      // reading also counted before, and a follow that drove away from the wire departed 0.95 m from it (emulator)
      // The line counts as met within the closing in the rate filter delay: the closing limit falls to 0 at the line, so
      // the side distance only nears it (follow inset 0: the follow never met the wire and never ended, emulator)
      if (!bOnLine_ && e <= v / (2 * PI_F * B_RATE_HZ)) { bOnLine_ = true; bT_ = 0; }
      // The long follow of a timed return lasts the follow time; a charge return follows on to the dock, follow-only
      // mode follows on
      const float followFor = bLong_ ? cfg.followTimeS : cfg.bounceFollowS;
      if (bOnLine_ && bT_ >= followFor && bRate_ >= 0 && !(bLong_ && (in.needCharge || cfg.followOnly))) {
        if (bLong_) decide(DEC_FOLLOW_END, 0, bP_);
        state = DEPART;
        bT_ = bTurned_ = 0;
        bTurnDeg_ = bPlanned_;
        bSteer_ = inward;
        bRateCmd_ = w;
        decide(DEC_DEPART, bSteer_ * bPlanned_, bP_);
      }
      if (w < wFull && fabsf(steer) >= 0.99f) rateGain(w, fabsf(in.v), v, in);   // a turn at the limit: learn
      o.v = v;
      o.w = steer * fminf(w, wSpeed) * wGain;
      return o;
    }
    case DEPART:
      if (w < wFull) rateGain(w, fabsf(in.v), v, in);
      o.v = v;
      o.w = bSteer_ * fminf(w, wSpeed) * wGain;
      if (w >= wFull) bTurnMeasure(in);   // the full turn rate: only from a turn at the full command
      if (bTurnDone(in)) {
        state = ROAM;
        sub = SUB_NONE;
        bb_ = false;
        bPeak_ = bMedian();
        bPeakX_ = -1e9f;
        if (bPlanned_ > 90) bS_ = (int8_t)-bS_;   // the turn takes the heading past square to the wire
        outsideTurned_ = true;   // the depart turned toward the inside: outside, it is the turn toward the wire
        outsideD_ = 1e9f;        // the next reading sets the least distance
        fit_.reset();
        bounces++;
        if (bLong_) { roamT_ = followT_ = 0; bLong_ = false; }   // both times start again at each new roam
#ifdef ROVER_WIRE_SOLVE
        // Estimator mode after its return join: the bounce re-arms as after a bounce turn (the depart ends inside the
        // bounce distance)
        if (!basics()) { armed_ = joinArmed_ = false; rearmStart(); }
#endif
        o.w = 0;
      }
      return o;
    default:
      bb_ = false;
      return o;
  }
}

#ifdef ROVER_WIRE_SOLVE
// The probe on entry: a known perturbation for the solve, and null avoidance. Bounce and Reverse bounce, before a
// bounce: once per approach, inside the approach distance and short of the bounce point, when the solve's angle between the drive direction and the wire is steeper than the depart
// angle limit (a mirror depart of a steeper arrival takes the limit; square-on, the field null loses the wire), the
// rover drives a 15 deg arc toward the nearer wire direction without a stop: the solve takes the known heading
// change, and the rover leaves the square-on null. The radius: the turn radius, and no less than one vertical
// distance range (sensor height + wire extra depth) over the 15 deg. True: the arc starts
bool Autopilot::entryProbe(const Inputs& in) {
  // every approach that can end in a bounce: roam, and a return driving in reverse (Reverse bounce: a reverse return
  // bounces first; a forward return joins). Roam only left 20 of 34 bounces steeper than 75 deg unprobed (emulator)
  if (basics() || (state == RETURN && dir_ > 0) || !armed_ || entryDone_ || sub != SUB_NONE || !rd.valid || in.inside < 0)
    return false;
  wirefit::BounceResult ws;
  if (!solveNow(ws) || ws.n < SJ_MIN_N || ws.d > cfg.approachDistance || ws.d <= cfg.bounceDistance) return false;
  entryDone_ = true;
  const float a = fold90(ws.psiDeg);   // the nearer wire direction, rover frame (deg): 0 along, +-90 square-on
  if (fabsf(a) <= cfg.bounceMinDeg) return false;
  const int8_t rot = a >= 0 ? 1 : -1;
  const float ang = 15.0f * D2R, r = fmaxf(cfg.turnRadius, (cfg.sensorHeight + cfg.wireMaxDepth) / ang);
  arcStart(ang, rot, dir_, r);
  sub = SUB_ENTRY_ARC;
  solveDebug("entry", in);
  decide(DEC_PROBE, rot * 15.0f, 0);
  dec.solveOk = true; dec.solvePsiDeg = ws.psiDeg; dec.solveD = ws.d; dec.solveSensor = (uint8_t)lead_;
  return true;
}
#endif

// ===== Sweeping turns =====
// A driven arc: the nose turns by angle (rad) in the sense rot (+1 counterclockwise) while the rover drives in the
// direction travel, on the radius. Done on the measured net turn, or at B_TURN_LIMIT x its time at the commanded rate.
// The speed: the approach speed, less when the radius needs more than 80 % of the full turn rate
float Autopilot::arcSpeed(float radius) const {
  const float v = cfg.approachFraction * cfg.roamSpeed, w = 0.8f * bYawRate();
  return (radius > 0 && v > w * radius) ? w * radius : v;
}

// The rate ratio: up while the measured rate is under the wanted rate, down while over (the rate loop's own integral
// catches up in a long turn), learned only near the set speed of the turn
void Autopilot::rateGain(float want, float speed, float set, const Inputs& in) {
  if (want < 1e-3f || speed < 0.5f * set || speed > 1.05f * set) return;
  const float gMax = cfg.maxYawRate > want ? cfg.maxYawRate / want : 1.0f;
  arcGain_ += in.dt * 2.0f * (want - fabsf(in.yawRate)) / want;
  arcGain_ = arcGain_ < 1 ? 1 : (arcGain_ > gMax ? gMax : arcGain_);
}

void Autopilot::arcStart(float angle, int8_t rot, int8_t travel, float radius) {
  arcTarget_ = angle;
  arcRot_ = rot;
  arcTravel_ = travel;
  arcRadius_ = radius > 0.01f ? radius : 0.01f;
  arcNet_ = arcT_ = 0;
}

// The turn rate follows the measured speed: rate = speed / radius, so the turn never runs ahead of the speed (from a
// stop the rover first rolls, then turns). The speed rises to the arc speed at 1 m/s^2. The steering rate loop reaches
// less than the commanded rate at part commands (emulator, a skid steer: 19 deg/s of 34 commanded after 4 s;
// lawn logs: 52 of 90 at full steering), and the arc then turns on twice the radius: the rate command rises by a
// learned ratio (arcGain_) until the measured rate reaches speed / radius
bool Autopilot::arcStep(const Inputs& in, Output& o) {
  const float va = arcSpeed(arcRadius_), wt = va / arcRadius_, wm = fabsf(in.yawRate);
  // a change of drive direction: stop first (no turn while the speed passes through 0)
  if (arcT_ == 0.0f && arcTravel_ * in.v < -STOP_SPEED) { o = Output(); return false; }
  arcNet_ += in.yawRate * in.dt;   // net angle, as the pivot (the gyro sign does not enter)
  arcT_ += in.dt;
  if (fabsf(arcNet_) >= arcTarget_ - 0.01f || arcT_ * wt >= 2 * B_TURN_LIMIT * arcTarget_ + 1.0f) { o = Output(); return true; }
  const float vm = fabsf(in.v), ww = vm / arcRadius_;   // also while it slows from the roam speed: the radius holds
  rateGain(ww, vm, va, in);
  const float v = vm + 1.0f * in.dt;
  o.v = arcTravel_ * (v < va ? v : va);
  o.w = arcRot_ * fminf(ww * arcGain_, cfg.maxYawRate);
  return false;
}

// Change the drive direction, and turn the nose by nose (rad, counterclockwise positive) on an arc of the turn radius
// in the new direction (a turn radius of 0: a pivot, then the new direction). The leading sensor of the new direction
// feeds the solve from the start of the arc
void Autopilot::reverseTurn(float nose, bool bounce) {
  swTravel_ = (int8_t)-dir_;
  swBounce_ = bounce;
  const int8_t rot = nose >= 0 ? 1 : -1;
  if (cfg.turnRadius > 0) {
    arcStart(fabsf(nose), rot, swTravel_, cfg.turnRadius);
    sub = SUB_SWEEP;
  } else {
    startPivot(fabsf(nose), rot);
    sub = bounce ? SUB_PIVOT : SUB_BPIVOT;
  }
  dir_ = swTravel_;
  leadPick();
}

#ifdef ROVER_WIRE_SOLVE
// The planned sweep join. In the wire frame (along the goal direction; n: the distance inside the wire), a driven turn
// of the nose from angle a to b on radius R moves the turn centre by R (cos a - cos b) away from the wire when it
// drives away, toward it when it drives on. The join ends on the follow line, the nose along the goal direction:
// one arc driving on (from an arrival with the goal direction: R (1 - cos a) of room), or an arc driving back
// first, then an arc driving on (any arrival, from R of room: an arrival against the goal direction needs up to 2 R
// in one arc). The solve gives the line (distance from the leading sensor, direction), the sensor position the
// turn centre. Due when the room to the follow line reaches the need of the plan, plus 0.3 s of roam
bool Autopilot::sweepJoinDue(const Inputs& in) {
  wirefit::BounceResult ws;
  if (!in.inRange || !solveNow(ws) || ws.n < SJ_MIN_N) return false;
  const float sg = cfg.goalCW ? -1.0f : 1.0f;   // the inside: left (+1) or right (-1) of the goal direction
  // the solve's wire direction has the sensor (inside) on its right: the clockwise way round
  const float th = wrapPi(-(ws.psiDeg * D2R + (cfg.goalCW ? 0.0f : PI_F)));   // nose against the goal direction
  if (sg * dir_ * sinf(th) > -0.05f) return false;   // not closing on the wire
  const float sf = wfb_->cfg.sensorFwd, sl = wfb_->cfg.sensorLeft;
  const float nc = ws.d - sg * (sf * sinf(th) + sl * cosf(th));        // turn centre
  // turn centre with sensor 1 on the line, plus SJ_END_IN: the follow closes the rest
  const float nf = cfg.followInset - sg * wirefit::Config().sensorLeft + SJ_END_IN;
  const float R = cfg.turnRadius, c0 = cosf(th);
  if (nc - nf > R * fminf(1.0f - c0, 1.0f) + 0.3f * cfg.roamSpeed) return false;
  sjTheta_ = th;
  sjNeed_ = nf - nc;
  return true;
}

void Autopilot::sweepJoinPlan() {
  const float R = cfg.turnRadius, c0 = cosf(sjTheta_);
  float cm = 0.5f * (sjNeed_ / R + 1.0f + c0);   // the nose between the arcs: R (2 cm - c0 - 1) = the move
  cm = cm < c0 ? c0 : (cm > 1.0f ? 1.0f : cm);
  const float am = acosf(cm);
  sjTurn1_ = fabsf(sjTheta_) - am;
  if (sjTurn1_ < 0) sjTurn1_ = 0;
  sjTurn2_ = am;
  sjRot_ = sjTheta_ > 0 ? -1 : 1;   // toward the goal direction
  sjArrive_ = dir_;
  sjPhase_ = sjTurn1_ > 0.02f ? 1 : 2;
  if (sjPhase_ == 1) arcStart(sjTurn1_, sjRot_, (int8_t)-sjArrive_, R);
  else arcStart(sjTurn2_, sjRot_, sjArrive_, R);
}
#endif

}  // namespace rover
