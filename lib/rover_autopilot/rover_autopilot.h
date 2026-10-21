// Rover autopilot for a tank-steer mower inside a perimeter wire.
//
// Behaviour (port of rover-sim src/core/autopilot.js and rover_rx.js, Documentation repo):
//   ROAM     drive straight; at bounce.pivotDistance from the wire stop, pivot away (mirror
//            bounce off the guessed wire line), drive on.
//   SEEK     battery low: like ROAM, but near the wire hand over to ACQUIRE.
//   ACQUIRE  approach to transition.pivotDistance, stop, pivot onto the wire at approachAngle
//            in the goal direction, drive straight until the wire is under the sensor.
//   FOLLOW   PD on the wire's crossing point X and dX/ds until the dock.
//   DEPART   after charging: follow depart length further, pivot inward, ROAM.
//   HALTED   wire lost for longer than lostTimeout: stop until reset().
//
// Direction along the wire: the rover is always inside the perimeter, and the wire is always
// below the sensor. When ACQUIRE stops at the wire, the side of the wire plus the inside
// assumption give the goal direction; FOLLOW keeps it by continuity.
//
// Exit beacon guard (not in rover-sim): a beacon transmitter (BTx) on the wire just past the
// dock exit. Travelling the goal way the rover meets the dock first; travelling the wrong
// way it meets the beacon first. In FOLLOW, the exit beacon before the dock means wrong way:
// stop, pivot 180 deg toward the inside, follow on. The guard arms only after guardArm metres
// of FOLLOW, so a rover that joins the wire between dock and beacon going the right way does
// not turn around. An entry beacon seen within entryWindow metres before means the right way
// (the dock was passed without a charge), so the exit beacon is then ignored.
//
// No Arduino dependencies: the same file builds on the host for the closed-loop tests in
// test/sim_harness. Units: metres, seconds, radians. Yaw rate and turns are CCW positive.

#pragma once
#include <stdint.h>

#ifdef ROVER_AP_DOUBLE
typedef double ap_real;
#else
typedef float ap_real;
#endif

namespace rover {

struct Config {
  // Geometry and rover
  ap_real sensorHeight = 0.15f;     // m, sensor centre above the wire
  ap_real maxSpeed = 1.0f;          // m/s at full drive command
  ap_real maxYawRate = 2.0f;        // rad/s at full steer command
  ap_real yawAccel = 5.0f;          // rad/s^2, pivot braking and the steer slew
  ap_real packetHz = 100.0f;        // STM32 packet rate
  bool goalCW = true;               // dock direction along the wire, seen from above

  // ROAM / SEEK bounce
  ap_real bounceSpeed = 1.0f, bounceAccel = 2.0f;
  ap_real bouncePivotDistance = 0.5f; // m
  ap_real bounceRandomDeg = 20.0f, bounceMinAngleDeg = 25.0f;

  // ACQUIRE (transition onto the wire)
  ap_real transSpeed = 0.3f, transAccel = 1.0f;
  ap_real transPivotDistance = 0.6f;  // m
  ap_real approachAngleDeg = 30.0f;
  ap_real followRadius = 0.1f;        // m, crossing point at which FOLLOW takes over

  // FOLLOW
  ap_real followSpeed = 0.5f, fastSpeed = 1.0f, followAccel = 1.0f;
  ap_real kOffset = 4.0f;             // rad/s per m of crossing point
  ap_real kHeading = 1.5f;            // D gain: rad/s per m/s of dX/dt (rover-sim keeps the old key name)
  ap_real lockLength = 0.5f, lockOffset = 0.05f, lockHeadingDeg = 10.0f;

  // Entry beacon: creep toward the dock
  ap_real padSpeed = 0.2f;            // m/s
  ap_real creepTime = 4.0f;           // s after the last in-range reading

  // Exit beacon guard
  bool exitGuard = true;
  ap_real guardArm = 1.5f;            // m of FOLLOW before the guard is armed
  ap_real entryWindow = 3.0f;         // m: an entry beacon this recently means the right way

  // DEPART
  ap_real departLength = 1.0f;        // m followed past the dock
  ap_real departTurnMinDeg = 60.0f, departTurnMaxDeg = 140.0f;

  // Noise handling (not in rover-sim; false reproduces rover-sim exactly). Bounce and
  // transition decisions use low-passed distance and an axis-averaged bearing.
  bool filterReadings = true;

  // Safety
  ap_real lostTimeout = 30.0f;        // s without the wire before HALTED (0: never)
  ap_real speedLimit = 0.0f;         // m/s cap on every driving speed (0: none); pivots are not affected

  // Test hook: pick the wrong goal direction at ACQUIRE (exercises the exit guard)
  bool testWrongWay = false;
};

// One call per STM32 packet (or more often; set packet only when a new one arrived).
struct Inputs {
  ap_real dt = 0;          // s since the previous call
  bool packet = false;     // a new STM32 packet arrived since the previous call
  bool inRange = false;    // packet: S1 not lost (lost bit also covers the regain wait)
  ap_real fwX = 0, fwY = 0, fwR = 0;  // packet: s1_x, s1_y, s1_radius in metres
  ap_real v = 0;           // measured forward speed, m/s (or the commanded estimate)
  ap_real yawRate = 0;     // measured yaw rate, rad/s (only its magnitude is used)
  bool needCharge = false; // battery low: go home
  bool docked = false;     // charge contacts made: hold still
  bool charged = false;    // event: charging has just ended
  bool entryBtx = false;   // entry beacon in range (creep)
  bool exitBtx = false;    // exit beacon in range (wrong-way guard)
};

struct Output {
  ap_real drive = 0;       // target, -1..1 of maxSpeed
  ap_real steer = 0;       // target, -1..1 of maxYawRate, CCW (left) positive
  ap_real accel = 0;       // m/s^2 toward the drive target
  ap_real v = 0, w = 0;    // slewed command in m/s and rad/s (for velocity-passthrough drives)
};

enum State : uint8_t { ROAM, SEEK, ACQUIRE, FOLLOW, DEPART, HALTED };
enum Sub : uint8_t { SUB_NONE, SUB_STOPPING, SUB_LEAVING, SUB_OPEN, SUB_TURNAROUND };

const char* stateName(State s);

// Per-packet processing of the STM32 output (rover-sim rover_rx.js + roverEstimate).
struct Biquad {
  ap_real b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  bool primed = false;
  void lowpass(ap_real fc, ap_real fs);
  ap_real step(ap_real x);
};

struct Reading {
  bool valid = false;      // in range, estimate formed
  ap_real X = 0;           // crossing point on the sensor's sideways axis, m, left positive
  ap_real d = 0;           // perpendicular distance to the wire, m (vertical field)
  ap_real psi = 0;         // |angle| between heading and wire, rad
  ap_real xw = 0;          // wire's sideways offset with the wire-below rule, m, left positive
  ap_real Xf = 0, dXds = 0;  // filtered X and its change per metre of travel (lock test only)
  ap_real dXdt = 0;          // change of Xf per second, filtered: the follow D term's input
  ap_real df = 0, psif = 0;  // low-passed distance, and the angle from df and Xa
  ap_real Xa = 0;          // crossing point from the axis-averaged bearing (side and angle)
};

class Autopilot {
 public:
  Config cfg;
  State state = ROAM;
  Sub sub = SUB_NONE;
  Reading rd;

  // Counters for logs and tests
  uint32_t bounces = 0, turnarounds = 0, charges = 0;

  void reset(State start = ROAM);
  Output step(const Inputs& in);
  bool isLocked() const { return locked; }  // FOLLOW at fastSpeed
  void halt() { state = HALTED; turnLeft = 0; }  // stop for good, until reset() (a stall stop from the caller)

 private:
  // Per-packet filters
  Biquad lpX, lpD, lpT, lpDist, lpU, lpV;
  bool havePrev = false;
  ap_real prevXf = 0;
  void processPacket(const Inputs& in);

  // Mission state
  ap_real turnLeft = 0;       // rad of pivot still to turn
  int turnDir = 1;            // +1 left (CCW), -1 right
  ap_real pending = 0;        // signed pivot set before stopping
  bool haveLeave = false;
  ap_real leaveDist = 0, leaveMax = 0, leaveStart = 0;
  ap_real openRun = 0, lockRun = 0, followRun = 0, departLeft = 0, entryRun = 0;
  bool locked = false, exitHold = false, entrySeen = false, inRange = false;
  ap_real creepLeft = 0, lostTime = 0, lastR = 0;
  double rng = 12345;         // rover-sim's LCG, evaluated in double as JavaScript does
  ap_real vOut = 0, wOut = 0; // slewed outputs

  ap_real rand01();
  ap_real dist() const;
  void bounceTurn();
  void approachTurn();
  Output out(ap_real v, ap_real w, ap_real accel, ap_real dt);
};

}  // namespace rover
