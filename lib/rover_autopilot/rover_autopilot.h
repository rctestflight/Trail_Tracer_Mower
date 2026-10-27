// Rover autopilot: the rover state machine of the agreed design (2026-10-20).
//
// States:
//   ROAM     drive straight at roam speed (approach speed inside the approach distance). At the bounce distance: one
//            fitted heading (probe first when needed), stop, re-acquisition check, pivot (mirror bounce away from the
//            wire). Bounce disarmed until the
//            distance reaches the re-arm distance or falls by the re-arm drop below its highest value since the
//            bounce. Beacon at the beacon bounce distance: stop, pivot 180 deg +- the beacon spread. Lost: drive on.
//            Polarity says outside: turn round (again each time the distance grows by the re-arm drop); no bounce.
//   RETURN   as ROAM; on entry into the approach distance -> JOIN.
//   JOIN     drive straight to the wire (lost: drive on); on the wire: stop, turn along the wire in the goal direction by
//            the signed fitted heading, FOLLOW. Sets the follower x sign from the known wire side.
//   FOLLOW   the vehicle's own wire follower drives (Output::external). Wrong direction (polarity or beacons): stop, turn
//            180 deg on the wire, follow on. Lost: stop; after rover_no_tx_s one 30 deg turn in place. Dock contacts -> DOCK.
//   DOCK     hold still while charging; charge end -> DEPART (follow-only: FOLLOW). A dock reached from ROAM or
//            RETURN goes back to that state.
//   DEPART   follow a random distance (depart minimum to maximum) after the dock contacts open, stop, pivot inward to
//            the leave angle (plus the bounce spread, at least the bounce minimum), ROAM with the bounce disarmed.
// Timed roam and follow (roam time and follow time both over 0, not follow-only): after the roam time in ROAM,
// RETURN; in FOLLOW after the follow time: stop, pivot inward to the leave angle, ROAM with the bounce disarmed (the
// end of DEPART). A charge return comes first; both times start again at each new roam.
//   FAULT    stopped until the drive mode is entered again: stuck detector (signal not lost), or lost longer than the
//            lost timeout in JOIN, FOLLOW or DEPART, or no polarity at a join with "join without polarity" = stop.
// No Tx signal -> stop and wait (the state holds): ROAM and RETURN after rover_no_tx_roam_s, others after rover_no_tx_s.
// Follow-only mode: RETURN, JOIN, FOLLOW only.
//
// Readings, once per packet, from x, y and radius as the Rx sends them (never flipped):
//   lateral wire offset X = H x / y (left positive), distance d from the vertical field part (|x| / R^2), root nearer
//   sqrt(R^2 - H^2). No wire angle from one reading (decision 2026-10-21).
// Heading: the approach fit (approach_fit.h); every straight drive in range feeds it. The bounce and the join turn by
// the fitted heading. When its uncertainty is over the limit (bounce: the least leave angle; join: the approach angle),
// or at the start in range, the probe drives straight for the probe length (forward when the distance to the wire is
// over it, else reverse) on the same line, and the fit with the probe frames decides.
// Wrong direction in follow: stop, pivot 90 deg to the inside, leave the wire to the approach distance, turn round
// and join again (JOIN).
// Re-acquisition check: the first 10 deg of the planned pivot is the perturbation; the distance change over it
// confirms the wire x sign (turning toward the wire must bring it closer). Polarity (inside x direction), when known,
// decides the sign.
//
// Bounce style "basics" (rover_bounce_style): the bounce
// is a join, a short follow and a depart, all driven (no stop, no turn on the spot). ROAM drives straight; at the join
// point (the radius, median of 5, at most the approach distance and fallen the re-arm fall below its highest value since
// the last turn, the rover inside) the rover joins toward the inside at the approach angle and follows the wire on its
// own steering at the follow inset for the bounce follow time; once it no longer closes on the wire, the depart drives
// an arc into the loop by the planned angle (the wire leave angle + the bounce spread, at least the bounce minimum).
// State codes: JOIN, FOLLOW, DEPART (bb_ set). Lost in JOIN or FOLLOW: stop and wait. basicsStep() has the
// detail.
//
// No Arduino dependencies: the same file builds on the PC for test/rover_emu.
#pragma once
#include <stdint.h>
#include "approach_fit.h"
#ifdef ROVER_WIRE_SOLVE
// The wire position solve drives the Bounce mode (estimator; firmware since 0.7.0). Its buffer comes from the heap
// at the start of a Bounce mode run and goes back at the end (releaseSolve): the ESP32-S2 update page needs that
// memory when the rover does not run.
#include "wire_fit_bounce.h"
#endif

namespace rover {

// Telemetry codes 0-5 keep their earlier meaning (seek = return, acquire = join, halted = fault).
enum State : uint8_t { ROAM = 0, RETURN = 1, JOIN = 2, FOLLOW = 3, DEPART = 4, FAULT = 5, DOCK = 6 };
enum Sub : uint8_t { SUB_NONE, SUB_START, SUB_STOP, SUB_CHECK, SUB_PIVOT, SUB_APPROACH, SUB_OPEN, SUB_WAIT, SUB_BSTOP,
                     SUB_BPIVOT, SUB_REVERSE_STOP, SUB_REVERSE, SUB_TURN_STOP, SUB_TURN, SUB_PROBE, SUB_PROBE_STOP, SUB_LEAVE, SUB_LEAVE_STOP, SUB_LEAVE_TURN,
                     SUB_PERTURB_STOP, SUB_PERTURB_TURN, SUB_PERTURB_BACK, SUB_PERTURB_END, SUB_BOUNCE_WAIT
};
enum ProbeFor : uint8_t { PROBE_START, PROBE_BOUNCE, PROBE_JOIN };
enum FaultCode : uint8_t { FAULT_NONE, FAULT_STUCK, FAULT_LOST, FAULT_NO_POLARITY, FAULT_POL_START, FAULT_NO_MEMORY };  // POL_START: polarity says
// outside before the first turn in rover mode (the rover starts inside): the Tx direction setting is the wrong way
// NO_MEMORY: the heap has no room for the wire solve at the start of a Bounce mode run
enum DecisionCode : uint8_t { DEC_NONE, DEC_BOUNCE, DEC_CHECK_FLIP, DEC_JOIN, DEC_FOLLOW, DEC_REVERSE, DEC_BEACON,
                              DEC_DOCK, DEC_DEPART, DEC_RETURN, DEC_FAULT, DEC_TX_OFF, DEC_TX_ON, DEC_START,
                              DEC_TURN_BACK, DEC_PROBE, DEC_FOLLOW_END, DEC_SQUARE_ON };

struct Config {
  float sensorHeight = 0.15f;      // m
  bool goalCW = true;              // goal direction round the loop, seen from above
  bool followOnly = false;         // RETURN, JOIN and FOLLOW only
  bool joinNoPolarityStop = false; // join without polarity: false = assume inside, true = stop
  float roamSpeed = 0.5f;          // m/s: the one top speed; the vehicle's speed at full drive command
  float approachFraction = 0.6f;   // approach speed as a part of the roam speed
  float bounceDistance = 0.5f;     // m
  float bounceSpreadDeg = 45;      // +- deg on the mirror bounce leave angle
  float bounceMinDeg = 35;         // least leave angle to the wire, deg
  float rearmDrop = 0.3f;          // m: outside the loop, the next turn-back after this much more distance
  float rearmFall = 0.03f;         // m: the bounce re-arms when the radius falls this much below its peak since the turn
  bool beaconBounce = true;
  float beaconDistance = 0.15f;    // m
  float beaconRearm = 0.25f;       // m
  float beaconSpreadDeg = 30;      // +- deg on the 180 deg beacon turn
  float approachDistance = 1.1f;   // m
  float approachAngleDeg = 45;     // deg: the re-entry angle (45 deg nominal)
  bool wrongDirCheck = true;
  float departMin = 0.5f, departMax = 3.0f;   // m
  float probeLength = 0.5f;        // m: the straight drive of the probe at the start in range (0 = no probe)
  float roamTimeS = 60;            // s of roam before a timed return and follow (0 = off)
  float followTimeS = 30;          // s of follow after a timed return, then turn inward and roam (0 = off)
  float followExitDeg = 90;        // leave angle to the wire at each change from follow to roam, before the bounce
                                   // spread (90 = straight into the loop, adjustable)
  float noTxRoamS = 60;            // s without Tx signal before ROAM and RETURN stop
  float noTxS = 2;                 // s without Tx signal before the other states stop
  float lostTimeout = 30;          // s in JOIN, FOLLOW or DEPART (0 = never)
  float maxYawRate = 1.5708f;      // rad/s at full steering command
  uint8_t bounceStyle = 1;         // roam mode: 0 = Bounce (estimator), 1 = Join then leave (basics)
  float followInset = 0.30f;       // m: basics: the follow line lies this far inside the wire
  float bounceFollowS = 3;         // s: basics: the short follow, from the moment the follow line is reached
  float wireMaxDepth = 0.12f;      // m: the most the wire lies deeper than the sensor height setting
};

struct Inputs {
  float dt = 0.01f;                // s since the last step
  bool inRange = false;            // sensor 1: not lost, not overload, not disabled
  float x = 0, y = 0, R = 0;       // m, as the Rx sends them
  int8_t inside = 0;               // polarity: +1 inside the loop, -1 outside, 0 not known
  int8_t heading = 0;              // polarity: +1 clockwise round the loop, -1 counterclockwise, 0 not known
  float v = 0;                     // measured forward speed, m/s
  float yawRate = 0;               // measured yaw rate, rad/s, counterclockwise positive
  float txOffS = 0;                // s since the Tx signal was last present
  bool stuck = false;              // stuck detector (the caller gives it only while the signal is not lost)
  bool docked = false;             // charging in progress
  bool onContacts = false;         // the dock contacts are closed (charger voltage present)
  bool needCharge = false;         // return voltage reached (a charge is due)
  float beaconRange = 99;          // m, the nearest beacon (filtered)
  bool entryBeacon = false, exitBeacon = false;
};

struct Output {
  float v = 0;                     // m/s target
  float w = 0;                     // rad/s target, counterclockwise positive
  bool external = false;           // the vehicle's own wire follower drives (FOLLOW, DEPART)
};

struct Reading {
  bool valid = false;
  float X = 0, d = 0;              // this packet
  float Xf = 0, df = 0;            // low-pass, 0.05 s
  float Rf = 0, Rs = 0;            // radius: low-pass 0.05 s, and 0.3 s for the bounce re-arm
};

struct Decision {
  uint32_t seq = 0;
  DecisionCode code = DEC_NONE;
  float d = 0, X = 0, psiDeg = 0, turnDeg = 0;
  int8_t side = 0;                 // wire x sign used (+1 left)
  int8_t reason = 0;               // FAULT: FaultCode
  FitResult fit;                   // the approach fit at this decision (BOUNCE, JOIN, PROBE)
  bool solveOk = false;            // the wire position solve at this decision (estimator bounce and perturb)
  float solvePsiDeg = 0, solveD = 0;   // its wire direction in the rover frame (+ counterclockwise) and distance, m
};

const char* stateName(State s);
const char* decisionName(DecisionCode c);

class Autopilot {
 public:
  Config cfg;
  State state = ROAM;
  Sub sub = SUB_START;
  FaultCode fault = FAULT_NONE;
  Reading rd;
  Decision dec;
  uint32_t bounces = 0, turnarounds = 0, charges = 0;
  bool txWait = false;
  int8_t yBelow = 0;               // the sign of Rx y on the wire when raw x matches the wire side (set at each join)

  void reset();
  Output step(const Inputs& in);
  void releaseSolve();              // the end of a run: the wire solve's memory back to the heap (no effect without it)
  void solveTaskRun();              // ESP32: the background solve task's work (rover_autopilot.cpp); not for callers
  void halt(FaultCode why) { toFault(why); }
  bool isLocked() const { return false; }  // no follow lock in this design (telemetry flag)
  bool holding() const { return hold_; }   // a turn on the spot, or its end: no event (the pivot rule)

 private:
  Output stepStates(const Inputs& in);
  void feedFit(const Inputs& in);
  void read(const Inputs& in);
  bool startDone(const Inputs& in, Output& o);
  void decide(DecisionCode c, float turnDeg, int8_t side);
  void toFault(FaultCode why);
  float rnd();                      // 0..1
  bool stopped(const Inputs& in);
  bool onSpot(const Inputs& in) const;  // the measured turn is a turn on the spot
  bool settled(const Inputs& in);       // after a pivot: the turn has ended
  void startPivot(float angle, int8_t dir);
  bool pivotStep(const Inputs& in, Output& o);   // true when the pivot is done
  void planBounce();
  void planJoin();
  bool fitGood(float limitDeg) const;
  void rearmStart();
  void settleStart();
  void planFromFit(const Inputs& in);
  void probeBegin(ProbeFor why, Output& o, float speed);
  void bounceNow(const Inputs& in);
  void joinNow(const Inputs& in);
  int8_t polSide(const Inputs& in) const;

  uint32_t rng_ = 12345;
  float lostT_ = 0, stopT_ = 0, startT_ = 0;
  // pivot
  float pivTarget_ = 0, pivTurned_ = 0, pivNet_ = 0, pivT_ = 0;
  bool startCheck_ = true;      // until the first bounce, join or follow: polarity outside = wrong Tx direction   // m of straight drive since the last bounce, beacon or depart turn
  int8_t pivDir_ = 1;
  // bounce and join plan
  bool armed_ = true, beaconArmed_ = true, joinArmed_ = true, settle_ = false;  // settle_: after a pivot, wait for the turn to end (settleStart)
  bool hold_ = false, wasHold_ = false;  // the pivot rule: this step and the last
  float settleT_ = 0;
  float rMax_ = 0, dMin_ = 0, dRef_ = 0;   // rMax_: the highest slow radius since the last turn
  float planPsi_ = 0, planX_ = 0, planLeave_ = 0, planRot_ = 0, planTurn_ = 0;
  int8_t planSide_ = 1, planDir_ = 1, planInside_ = 1;
  bool planPolKnown_ = false;
  float run_ = 0, departLeft_ = 0, reverseArm_ = 0, dMinOpen_ = 0;
  float roamT_ = 0, followT_ = 0;  // s in ROAM, s in FOLLOW (timed roam and follow)
  bool timed() const { return cfg.roamTimeS > 0 && cfg.followTimeS > 0 && !cfg.followOnly; }
  int turnBacks_ = 0;
  State dockFrom_ = FOLLOW;
  int wrongCount_ = 0;
  bool entrySeen_ = false, exitArmed_ = true;
  // approach fit and probe
  ApproachFit fit_;
#ifdef ROVER_WIRE_SOLVE
  static const bool kWireSolve = true;
  wirefit::WireFitBounce* wfb_ = nullptr;   // heap, a Bounce mode run only; null: not allocated
  wirefit::BounceResult wres_;    // the last solve
  long wTotal_ = -1;
  bool solveNow(wirefit::BounceResult& r) const;   // the last solve moved to the current pose; false: no valid solve
  void solveDebug(const char* what, const Inputs& in) const;
  void solveStep();                 // each Bounce mode frame: the next solve when the last one is done
  void bounceBegin();               // the bounce point: stop, and ask for the direction profile (SUB_BOUNCE_WAIT)
  float profileNow();               // the direction profile at a rise of 2 for the bounce (90: no direction information)
  bool profileReady() const;
  bool profWant_ = false, profReady_ = false;
  float prof_ = 90, bounceWaitT_ = 0;
#ifdef ESP_PLATFORM
  // the background solve (one FreeRTOS task at the loop's priority): 0 idle, 1 the task runs iterate() on the prepared
  // window (and the profile when asked), 2 its answer waits for the loop
  uint8_t sState_ = 0;
  bool sWithProf_ = false;
  float sProf_ = 90;
  wirefit::BounceResult sRes_;
  void* sTask_ = nullptr;           // TaskHandle_t
#endif
#else
  static const bool kWireSolve = false;
#endif
  Output last_;
  int8_t polNow_ = 0;
  float probeLeft_ = 0;
  int8_t probeDir_ = 1;
  ProbeFor probeFor_ = PROBE_START;
  bool leaveFlipped_ = false, rejoin_ = false, outsideTurned_ = false;
  bool perturbed_ = false;   // estimator: this approach had its perturb (15 deg turn, short reverse, solve again)
  float perturbLeft_ = 0;    // m of the perturb reverse still to drive
  bool searched_ = false, searching_ = false;   // lost in join, follow or depart: one turn out of the field null
  int8_t searchDir_ = 1;
  float outsideD_ = 0;      // ROAM: the least distance since the last outside turn
  float lastDf_ = 99;       // JOIN: the last distance while in range
  int8_t xSignNear_ = 0;    // JOIN: the Rx x sign 1-3 sensor heights from the wire
  int8_t planToward_ = 1;    // +1 closing on the wire at the reading, -1 moving away

  // Bounce style basics (basicsStep)
  bool basics() const { return cfg.bounceStyle == 1; }
  void basicsSide(const Inputs& in);
  void basicsJoin(const Inputs& in, Output& o);
  Output basicsStep(const Inputs& in);
#ifdef BASICS_DEBUG
  Output basicsStep2(const Inputs& in);
#endif
  float bMedian() const;
  float bTurnRadius() const;
  float bCloseDeg(float xs) const;
  float bJoinDist() const;
  float bGoalSense() const;
  float bLeaveDeg();
  int8_t bWireSide(const Inputs& in) const;
  void bTurnMeasure(const Inputs& in);
  float bYawRate() const;
  float bJoinSpeed() const;
  float bJoinRoom() const;
  float bJoinRadius() const;
  bool bThreePoint(const Inputs& in, float xs);
  float bRate(float xs, float dt);
  bool bGuard(const Inputs& in, float xs);
  bool bTurnDone(const Inputs& in);
  bool bb_ = false;                 // a basics bounce runs (state JOIN, FOLLOW or DEPART)
  bool bLong_ = false;              // the basics join of a timed or charge return: the long follow
  int8_t bP_ = 1, bM_ = 1, bS_ = 1; // wire side: inside where bP_ x > 0; side memory; sign of y with the wire below
  bool bWasLost_ = true;
  float bXLast_ = 0;
  static const int B_MED = 5;
  float bHist_[B_MED] = {0};
  int bN_ = 0, bI_ = 0;
  float bPeakX_ = -1e9f;     // highest side distance reading (bP_ x) since the last turn (the roam join)
  float bPeak_ = 0, bT_ = 0, bPlanned_ = 0, bTurnT_ = 0, bSteer_ = 0, bMinXs_ = 1e9f, bXsF_ = 0, bRate_ = 0;
  float bTurned_ = 0, bTurnDeg_ = 0;  // a driven turn: measured so far, planned (deg)
  bool bOnLine_ = false, bHavePrev_ = false, bStopped_ = false;
  bool bPivot_ = false;             // estimator join: a pivot on the spot
  float bWMeas_ = 0;                // measured full turn rate in the depart, rad/s (0: none yet)
  bool bRev_ = false;               // in the reverse leg of a three-point turn
  bool bClosed_ = false;            // the join closed on the wire since its start or its last reverse leg
  float bGuardTurnT_ = 0, bGuardPeakR_ = 0;
  bool bYShot_ = false, bYActive_ = false, bGuardWas_ = false;
};

}  // namespace rover
