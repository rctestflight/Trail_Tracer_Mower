// Rover autopilot. See rover_autopilot.h for the behaviour; comments here cite the
// rover-sim source each block was ported from (Documentation repo, commit c1c6401).

#include "rover_autopilot.h"
#include <cmath>

namespace rover {

#define K(x) static_cast<ap_real>(x)
static const ap_real PI_ = K(3.14159265358979323846);
static const ap_real D2R = PI_ / K(180);

static inline ap_real clampr(ap_real x, ap_real lo, ap_real hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline ap_real sgn(ap_real x) { return x > 0 ? K(1) : (x < 0 ? K(-1) : K(0)); }
static inline ap_real wrapPi(ap_real a) { return std::atan2(std::sin(a), std::cos(a)); }

const char* stateName(State s) {
  switch (s) {
    case ROAM: return "roam";
    case SEEK: return "seek";
    case ACQUIRE: return "acquire";
    case FOLLOW: return "follow";
    case DEPART: return "depart";
    case HALTED: return "halted";
  }
  return "?";
}

// ---- Biquad low-pass (rover_rx.js biquadLowpass / biquadStep) ----

void Biquad::lowpass(ap_real fc, ap_real fs) {
  const ap_real q = K(0.70710678118654752);
  const ap_real w0 = 2 * PI_ * fc / fs, c = std::cos(w0), al = std::sin(w0) / (2 * q), a0 = 1 + al;
  b0 = (1 - c) / 2 / a0; b1 = (1 - c) / a0; b2 = (1 - c) / 2 / a0;
  a1 = -2 * c / a0; a2 = (1 - al) / a0;
  x1 = x2 = y1 = y2 = 0; primed = false;
}

ap_real Biquad::step(ap_real x) {
  if (!primed) { x1 = x2 = y1 = y2 = x; primed = true; }   // start at the first value, no step
  const ap_real y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
  x2 = x1; x1 = x; y2 = y1; y1 = y;
  return y;
}

// ---- Autopilot ----

void Autopilot::reset(State start) {
  const Config keep = cfg;
  *this = Autopilot();
  cfg = keep;
  state = start;
  lpX.lowpass(5, cfg.packetHz);
  lpD.lowpass(5, cfg.packetHz);
  lpT.lowpass(5, cfg.packetHz);
  lpDist.lowpass(5, cfg.packetHz);
  lpU.lowpass(5, cfg.packetHz);
  lpV.lowpass(5, cfg.packetHz);
}

// rover-sim's LCG. JavaScript evaluates m.rng * 1103515245 + 12345 in double precision and
// then takes it mod 2^32; doing the same keeps the random sequence identical to the sim.
ap_real Autopilot::rand01() {
  double p = rng * 1103515245.0 + 12345.0;
  rng = std::fmod(p, 4294967296.0);
  const uint32_t r = static_cast<uint32_t>(rng);
  return K((r & 0xffffffu) / 16777216.0);
}

// One STM32 packet: wire-below rule, crossing point, distance, angle, filters.
// (receiver.js roverEstimate + rover_rx.js roverRxStep)
void Autopilot::processPacket(const Inputs& in) {
  const ap_real H = cfg.sensorHeight;
  inRange = in.inRange;
  lastR = in.fwR;
  if (!in.inRange || !(in.fwR > 0)) {
    lpX.lowpass(5, cfg.packetHz); lpD.lowpass(5, cfg.packetHz); lpT.lowpass(5, cfg.packetHz); lpDist.lowpass(5, cfg.packetHz);
    lpU.lowpass(5, cfg.packetHz); lpV.lowpass(5, cfg.packetHz);
    havePrev = false; rd.dXds = 0; rd.dXdt = 0; rd.valid = false;
    return;
  }
  // 1. The wire is below the sensor: the pair only knows the field's axis.
  ap_real x = in.fwX, y = in.fwY;
  if (y < 0) { x = -x; y = -y; }
  rd.xw = x;
  // 2. Crossing point on the sensor's sideways axis (exact at any yaw).
  const ap_real X = y > K(1e-4) ? H * x / y : (x < 0 ? K(-1e4) : K(1e4));
  // 3. Distance from the vertical field. In units where each coil reads 1 over the wire,
  //    |w| = sqrt2 H |x| / R^2 (the firmware's own x and R carry it), and a straight wire gives
  //    |w| = sqrt2 H d / (d^2 + H^2): two roots d and H^2/d; keep the one nearer sqrt(R^2 - H^2).
  const ap_real R = in.fwR;
  const ap_real w = K(1.41421356237309505) * H * std::fabs(x) / (R * R);
  const ap_real d1 = std::sqrt(std::fmax(K(0), R * R - H * H));
  ap_real d;
  if (w < K(1e-12)) d = 0;
  else {
    const ap_real disc = 2 - 4 * w * w;
    if (disc <= 0) d = H;
    else {
      const ap_real r1 = H * (K(1.41421356237309505) + std::sqrt(disc)) / (2 * w), r0 = H * H / r1;
      d = std::fabs(r0 - d1) < std::fabs(r1 - d1) ? r0 : r1;
    }
  }
  // 4. |angle| between heading and wire.
  const ap_real c = std::fabs(X) > K(1e-9) ? std::fmin(K(1), d / std::fabs(X)) : K(1);
  rd.X = clampr(X, -3, 3);
  rd.d = d;
  rd.psi = std::acos(c);
  rd.valid = true;
  // Filters: X low-passed; dX/ds from the filtered difference over one packet's travel.
  rd.Xf = lpX.step(rd.X);
  const ap_real ds = std::fabs(in.v) / cfg.packetHz;
  ap_real raw;
  if (havePrev && ds > K(1e-3)) raw = clampr((rd.Xf - prevXf) / ds * (in.v < 0 ? K(-1) : K(1)), -1, 1);
  else raw = rd.dXds * K(0.9);
  rd.dXds = lpD.step(raw);
  // Change of X per second (the D term's input): the filtered difference times the packet
  // rate, with its own low-pass. No speed in it (rover-sim 512a748).
  rd.dXdt = lpT.step(havePrev ? (rd.Xf - prevXf) * cfg.packetHz : K(0));
  prevXf = rd.Xf;
  havePrev = true;
  // Low-passed readings for the mission decisions. The bearing is averaged as an axis
  // (cos 2th, sin 2th): noise near the sensor's horizon flips (x, y) to (-x, -y), which the
  // doubled angle ignores, so the average keeps the side. Halving the averaged angle gives
  // a bearing within 90 deg of straight down: the wire-below rule.
  rd.df = lpDist.step(d);
  const ap_real R2 = R * R;
  const ap_real uf = lpU.step((in.fwY * in.fwY - in.fwX * in.fwX) / R2);
  const ap_real vf = lpV.step(2 * in.fwX * in.fwY / R2);
  const ap_real th = std::atan2(vf, uf) / 2;
  rd.Xa = std::fabs(th) < K(1.5706) ? H * std::tan(th) : (th < 0 ? K(-1e4) : K(1e4));
  rd.psif = std::acos(std::fabs(rd.Xa) > K(1e-9) ? std::fmin(K(1), rd.df / std::fabs(rd.Xa)) : K(1));
}

// Distance to the wire for the mission logic: the vertical-field distance when in range.
ap_real Autopilot::dist() const {
  if (rd.valid) return cfg.filterReadings ? rd.df : rd.d;
  const ap_real H = cfg.sensorHeight;
  return std::sqrt(std::fmax(K(0), lastR * lastR - H * H));
}

// Bounce: mirror off the guessed wire line, plus a random spread, the shorter way round
// (autopilot.js bounceTurn, shortestTurn).
void Autopilot::bounceTurn() {
  const ap_real psi = !rd.valid ? PI_ / 2 : (cfg.filterReadings ? rd.psif : rd.psi);
  const ap_real xw = cfg.filterReadings ? rd.Xa : rd.xw;
  ap_real leave = psi + (2 * rand01() - 1) * cfg.bounceRandomDeg * D2R;
  leave = clampr(leave, cfg.bounceMinAngleDeg * D2R, PI_ - cfg.bounceMinAngleDeg * D2R);
  const ap_real side = (std::fabs(xw) > K(0.02) && psi < 80 * D2R) ? sgn(xw) : (rand01() < K(0.5) ? K(1) : K(-1));
  turnDir = side > 0 ? -1 : 1;
  turnLeft = psi + leave;
  // A pivot over 180 deg one way is the same heading change as 360 deg minus it the other way.
  if (turnLeft > PI_) { turnLeft = 2 * PI_ - turnLeft; turnDir = -turnDir; }
}

// Transition: the wire line is psi off the heading on side s (+1 left). Of the two mirror
// lines, the one crossing ahead is real (the rover is closing from the inside). The goal way
// keeps the wire on side g (CW: inside on the right, wire on the left, g = +1). Target:
// that tangent turned approachAngle toward the wire (autopilot.js approachTurn).
void Autopilot::approachTurn() {
  const ap_real psi = !rd.valid ? PI_ / 2 : (cfg.filterReadings ? rd.psif : rd.psi);
  const ap_real xw = cfg.filterReadings ? rd.Xa : rd.xw;
  const ap_real s = std::fabs(xw) > K(0.02) ? sgn(xw) : K(1);
  bool cw = cfg.goalCW;
  if (cfg.testWrongWay) cw = !cw;
  const ap_real g = cw ? K(1) : K(-1);
  const ap_real tangent = -s * psi + (s == g ? K(0) : PI_);
  pending = wrapPi(tangent + g * cfg.approachAngleDeg * D2R);
}

// Drive target plus the slewed command for drives without their own ramp.
Output Autopilot::out(ap_real v, ap_real w, ap_real accel, ap_real dt) {
  Output o;
  if (cfg.speedLimit > 0) v = clampr(v, -cfg.speedLimit, cfg.speedLimit);
  o.drive = clampr(v / cfg.maxSpeed, -1, 1);
  o.steer = clampr(w / cfg.maxYawRate, -1, 1);
  o.accel = accel;
  const ap_real tv = o.drive * cfg.maxSpeed, tw = o.steer * cfg.maxYawRate;
  const ap_real dv = accel * dt, dw = cfg.yawAccel * dt;
  vOut = tv - vOut > dv ? vOut + dv : (tv - vOut < -dv ? vOut - dv : tv);
  wOut = tw - wOut > dw ? wOut + dw : (tw - wOut < -dw ? wOut - dw : tw);
  o.v = vOut; o.w = wOut;
  return o;
}

Output Autopilot::step(const Inputs& in) {
  const ap_real dt = in.dt;
  if (in.packet) processPacket(in);
  const ap_real av = std::fabs(in.v), aw = std::fabs(in.yawRate);

  // Safety: wire lost too long (an escape) -> stop for good.
  if (state != HALTED && cfg.lostTimeout > 0) {
    lostTime = inRange ? K(0) : lostTime + dt;
    if (lostTime > cfg.lostTimeout) { state = HALTED; turnLeft = 0; }
  }
  if (state == HALTED) return out(0, 0, cfg.bounceAccel, dt);

  // Docked: hold still while charging. Charged: leave along the wire.
  if (in.docked) { turnLeft = 0; return out(0, 0, cfg.followAccel, dt); }
  // A charge in any other state also means the rover sits in the dock on the wire, facing
  // the goal way (rover-sim only acts on it in FOLLOW).
  if (in.charged) {
    charges++;
    if (state != DEPART) { state = DEPART; sub = SUB_NONE; turnLeft = 0; departLeft = cfg.departLength; }
  }

  // A pivot in progress runs to completion first; brake in time to end on the angle.
  if (turnLeft > 0) {
    turnLeft -= aw * dt;
    const ap_real wMax = std::sqrt(2 * cfg.yawAccel * std::fmax(K(0), turnLeft));
    const ap_real w = turnDir * std::fmin(K(1), wMax / cfg.maxYawRate) * cfg.maxYawRate;
    if (turnLeft <= K(0.005)) turnLeft = 0;
    return out(0, w, cfg.bounceAccel, dt);
  }

  if (state == ROAM && in.needCharge) state = SEEK;

  ap_real v = 0, w = 0, acc = cfg.bounceAccel;
  const ap_real d = dist();
  switch (state) {
    case ROAM:
    case SEEK: {
      acc = cfg.bounceAccel;
      if (sub == SUB_LEAVING) {
        // Drive away without bouncing until the distance has passed a maximum (closing on
        // wire again), has fallen 0.15 m below its start (a curve bending toward the rover),
        // or is past pivotDistance + 0.25 m.
        if (!haveLeave) { leaveDist = d; leaveMax = d; leaveStart = d; haveLeave = true; }
        leaveDist = std::fmin(leaveDist, d);
        leaveMax = d <= leaveDist + K(1e-9) ? d : std::fmax(leaveMax, d);
        const bool peaked = leaveMax > leaveDist + K(0.05) && d < leaveMax - K(0.05);
        const bool closing = d < leaveStart - K(0.15);
        if (!inRange || d >= cfg.bouncePivotDistance + K(0.25) || peaked || closing) sub = SUB_NONE;
        else { v = cfg.bounceSpeed; break; }
      }
      if (inRange && state == SEEK && d <= cfg.transPivotDistance + K(0.5)) { state = ACQUIRE; sub = SUB_NONE; break; }
      if (sub == SUB_STOPPING) {
        if (av > K(0.02)) break;                       // stop first, then pivot
        turnLeft = pending; sub = SUB_LEAVING; haveLeave = false; bounces++;
        break;
      }
      if (inRange && d <= cfg.bouncePivotDistance) {
        // The reading at the pivot distance, not after stopping (near d = H it is ill-conditioned).
        bounceTurn(); pending = turnLeft; turnLeft = 0; sub = SUB_STOPPING;
        break;
      }
      v = cfg.bounceSpeed;
      break;
    }

    case ACQUIRE: {
      acc = cfg.transAccel;
      if (!inRange || (sub == SUB_NONE && d > cfg.transPivotDistance + K(0.7))) { state = SEEK; sub = SUB_NONE; break; }
      if (sub == SUB_NONE) {
        if (d > cfg.transPivotDistance) { v = cfg.transSpeed; break; }
        approachTurn();
        sub = SUB_STOPPING;
      }
      if (sub == SUB_STOPPING) {
        if (av > K(0.02)) break;
        turnLeft = std::fabs(pending); turnDir = pending < 0 ? -1 : 1;
        sub = SUB_OPEN; openRun = 0;
        break;
      }
      // Open loop straight on until the crossing point is within followRadius.
      openRun += av * dt;
      const ap_real expect = cfg.transPivotDistance / std::fmax(K(0.2), std::sin(cfg.approachAngleDeg * D2R));
      const ap_real crossX = rd.valid ? std::fabs(rd.X) : K(1e9);
      if (crossX <= cfg.followRadius || d <= K(0.5) * cfg.sensorHeight || openRun > 3 * expect) {
        state = FOLLOW; sub = SUB_NONE; lockRun = 0; locked = false;
        followRun = 0; exitHold = false; entrySeen = false;
        break;
      }
      v = cfg.transSpeed;
      break;
    }

    case FOLLOW:
    case DEPART: {
      acc = cfg.followAccel;
      if (!inRange) { state = SEEK; sub = SUB_NONE; locked = false; break; }
      followRun += av * dt;
      if (in.entryBtx) { entrySeen = true; entryRun = 0; creepLeft = cfg.creepTime; }
      else entryRun += av * dt;

      // Exit beacon guard: the exit beacon before the dock means the wrong way.
      if (state == FOLLOW && cfg.exitGuard) {
        if (exitHold && !in.exitBtx) exitHold = false;
        const bool rightWay = entrySeen && entryRun < cfg.entryWindow;
        if (sub == SUB_NONE && in.exitBtx && !exitHold && followRun >= cfg.guardArm && !rightWay) {
          sub = SUB_TURNAROUND; exitHold = true; turnarounds++;
        }
        if (sub == SUB_TURNAROUND) {
          if (av > K(0.02)) break;                     // stop, then 180 deg toward the inside
          turnDir = cfg.goalCW ? 1 : -1;               // wrong way for CW travel: inside on the left
          turnLeft = PI_; sub = SUB_NONE; followRun = 0; locked = false; lockRun = 0;
          break;
        }
      }

      // Lock test on X and the heading angle asin(dX/ds); the steering PD uses dX/dt.
      const ap_real X = rd.Xf, ePsi = std::asin(clampr(rd.dXds, -1, 1));
      if (std::fabs(X) < cfg.lockOffset && std::fabs(ePsi) < cfg.lockHeadingDeg * D2R) lockRun += av * dt;
      else if (!locked) lockRun = 0;
      if (!locked && lockRun >= cfg.lockLength) locked = true;
      // Once locked the speed stays fast until the wire is lost (rover-sim 87fa177).
      v = locked ? cfg.fastSpeed : cfg.followSpeed;
      if (creepLeft > 0) { creepLeft -= dt; if (state == FOLLOW) v = std::fmin(v, cfg.padSpeed); }
      w = cfg.kOffset * X + cfg.kHeading * rd.dXdt;  // PD: P on X, D on dX/dt
      if (state == DEPART) {
        departLeft -= av * dt;
        if (departLeft <= 0) {
          // Turn inward (CW travel has the inside on the right) by a random angle.
          turnDir = cfg.goalCW ? -1 : 1;
          turnLeft = (cfg.departTurnMinDeg + (cfg.departTurnMaxDeg - cfg.departTurnMinDeg) * rand01()) * D2R;
          state = ROAM; sub = SUB_LEAVING; haveLeave = false; locked = false;
        }
      }
      break;
    }

    case HALTED:
      break;
  }
  return out(v, w, acc, dt);
}

}  // namespace rover
