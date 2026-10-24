// Rover autopilot: see rover_autopilot.h for the states and the readings.
#include "rover_autopilot.h"
#include <math.h>

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
  settle_ = false;
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
}

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
  planBounce();
  decide(DEC_BOUNCE, planDir_ * planTurn_ / D2R, planSide_);
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

Output Autopilot::step(const Inputs& in) {
  polNow_ = polSide(in);
  feedFit(in);
  last_ = stepStates(in);
  return last_;
}

// The end of the start: on the wire -> FOLLOW (returns true), else ROAM or RETURN.
bool Autopilot::startDone(const Inputs& in, Output& o) {
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

// 0.6.36: the end of a pivot that goes on to roam. A pivot ends while the rover still turns, and in a pivot the sensor
// swings round the rover centre: its radius and side change with no drive (rover log 2026-10-24: after turns of 65 to
// 115 deg the swing-back read as closing on the wire, and the rover bounced 0.3 s after the turn). So nothing acts on
// a pivot (operator 2026-10-24): the rover stops turning first, and the re-arm starts from the stopped reading.
void Autopilot::settleStart() {
  settle_ = true;
  stopT_ = 0;
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
  if (state == FAULT) return o;

  // Every state
  if (in.stuck && in.inRange) { toFault(FAULT_STUCK); return o; }
  lostT_ = in.inRange ? 0 : lostT_ + in.dt;
  if (cfg.lostTimeout > 0 && lostT_ > cfg.lostTimeout && (state == JOIN || state == FOLLOW || state == DEPART)) {
    toFault(FAULT_LOST);
    return o;
  }
  // Lost in FOLLOW or DEPART for rover_no_tx_s: one 30 deg turn in place (alternate sides). Square-on over
  // the wire the coils sit in the field null; the turn ends it. In place, so safe with the Tx off too.
  const bool wireState = state == FOLLOW || state == DEPART;
  if (in.inRange) { searched_ = searching_ = false; }
  else if (wireState) {
    if (searching_) {
      Output so;
      if (pivotStep(in, so)) { searching_ = false; so = Output(); }
      return so;
    }
    if (!searched_ && lostT_ > cfg.noTxS) {
      searched_ = searching_ = true;
      startPivot(30 * D2R, searchDir_);
      searchDir_ = (int8_t)-searchDir_;
      return o;
    }
  }
  // No Tx signal: ROAM and RETURN wait the roam limit (far from the wire the signal can drop out)
  const float noTxLimit = (state == ROAM || state == RETURN) ? cfg.noTxRoamS : cfg.noTxS;
  if (in.txOffS > noTxLimit) {
    if (!txWait) decide(DEC_TX_OFF, 0, 0);
    txWait = true;
    return o;
  }
  if (txWait) { txWait = false; decide(DEC_TX_ON, 0, 0); }
  if (in.docked && state != DOCK) {
    dockFrom_ = state;
    state = DOCK;
    sub = SUB_NONE;
    decide(DEC_DOCK, 0, 0);
  }

  const float approachV = cfg.approachFraction * cfg.roamSpeed;
  const float H = cfg.sensorHeight;

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
      if (startCheck_ && in.inside < 0) { toFault(FAULT_POL_START); return Output(); }
      if ((cfg.followOnly || in.needCharge) && state == ROAM) { state = RETURN; decide(DEC_RETURN, 0, 0); }
      if (state == ROAM && timed()) {
        roamT_ += in.dt;
        if (roamT_ >= cfg.roamTimeS) { state = RETURN; followT_ = 0; decide(DEC_RETURN, 0, 0); }
      }
      // Beacon re-arm: past the beacon re-arm distance
      if (!beaconArmed_ && in.beaconRange > cfg.beaconRearm) beaconArmed_ = true;
      switch (sub) {
        case SUB_BSTOP:
          if (!stopped(in)) return o;
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
          startPivot(planTurn_, planDir_);
          sub = SUB_PIVOT;
          return o;
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
      // After a pivot: stop turning before anything acts (settleStart)
      if (settle_) {
        if (!stopped(in)) return o;
        settle_ = false;
        rearmStart();
      }
      // Driving straight. Polarity says outside the loop: no bounce until inside again.
      // Outside: turn round, and again each time the distance grows by the re-arm drop since the last turn
      if (in.inside < 0) {
        if (!outsideTurned_ || (rd.valid && rd.df > outsideD_ + cfg.rearmDrop)) {
          outsideTurned_ = true;
          outsideD_ = rd.valid ? rd.df : 0;
          startPivot(PI_F, 1); sub = SUB_BPIVOT; decide(DEC_TURN_BACK, 180, 0); return o;
        }
        if (rd.valid) outsideD_ = fminf(outsideD_, rd.df);
        o.v = approachV;
        return o;
      }
      if (in.inside > 0) outsideTurned_ = false;
      if (cfg.beaconBounce && beaconArmed_ && in.beaconRange <= cfg.beaconDistance) {
        beaconArmed_ = false;
        sub = SUB_BSTOP;
        stopT_ = 0;
        return o;
      }
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
      if (state == RETURN && joinArmed_ && rd.valid && rd.df <= cfg.approachDistance) {
        state = JOIN;
        sub = SUB_APPROACH;
        lastDf_ = rd.df;
        decide(DEC_JOIN, 0, 0);
        o.v = approachV;
        return o;
      }
      // Bounce distance from the radius and the height setting. The offset x reads high near the wire (the rest of the
      // loop adds field); the radius does not. A height error of 120 mm moves the bounce point by under 6 cm at 0.5 m.
      const float dR = rd.valid ? sqrtf(fmaxf(0.0f, rd.Rf * rd.Rf - cfg.sensorHeight * cfg.sensorHeight)) : 99;
      if (armed_ && rd.valid && dR <= cfg.bounceDistance) {
        bounceNow(in);  // the fitted heading, or square-on with no fit
        return o;
      }
      o.v = (rd.valid && rd.df <= cfg.approachDistance) ? approachV : cfg.roamSpeed;
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
            wrongCount_ = 0; entrySeen_ = false; exitArmed_ = !in.exitBeacon; reverseArm_ = 0;
            decide(DEC_FOLLOW, 0, 0);
            o = Output();
            o.external = true;
          }
          return o;
        default: {
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
        // minimum as at a bounce (operator 2026-10-23)
        const float mn = cfg.bounceMinDeg * D2R;
        float turn = cfg.followExitDeg * D2R + (2 * rnd() - 1) * cfg.bounceSpreadDeg * D2R;
        turn = fmaxf(mn, fminf(PI_F - mn, turn));
        startPivot(turn, cfg.goalCW ? -1 : 1);  // goal clockwise: the inside is on the right
        sub = SUB_TURN;
        return o;
      }
      if (sub == SUB_TURN) {
        if (pivotStep(in, o)) {
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

}  // namespace rover
