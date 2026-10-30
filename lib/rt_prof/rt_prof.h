// rt_prof: timing of the real-time path (design priority, 2026-10-12): STM32 frame -> control -> CAN motor command.
//
// Timestamps in us (esp_timer). The nDATA_READY interrupt records the edge time of each frame in a ring of 16, so a
// frame read late (from the UART backlog) still gets its own edge time.
//   wait     edge -> frame read starts          (how late the loop picks the frame up)
//   decode   frame read -> frame decoded
//   control  frame decoded -> drive command set  (the main-branch steering control, odrive_setDriveOutput)
//   latency  edge -> first CAN send that carries that command (odrive_outputUpdate, its own period)
//   can      duration of one odrive_outputUpdate CAN burst (all nodes)
//   gyro age at the CAN burst: time since the last MPU sample (serviceMpu)
// Loop passes: wall time of each loop() pass and of each loop part (cpu_prof marks), with the part that held the
// longest pass. 'rt' prints the window since the last 'rt' and starts a new one.
// Passes longer than the frame period, by the part that took the most time in them: count, that part's longest
// time, the longest such pass.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rtprof {

void edgeIsr();          // nDATA_READY interrupt (IRAM)
void resync();           // the frame buffer was drained: edges so far have no frame
void frameRead();        // a frame read starts
void frameDecoded();
/** @brief Real-time path timing (bench command "rt"): the drive command now comes from this frame. */
void commandSet();
/** @brief Real-time path timing: odrive_outputUpdate starts its CAN burst. */
void canStart();
/** @brief Real-time path timing: odrive_outputUpdate ends its CAN burst. */
void canEnd();
void gyroSample();       // an MPU sample was taken
void gyroSampleAt(int64_t t_us);  // an MPU sample read at esp_timer time t_us (test build IMU_I2C_NB)
void part(uint8_t part); // loop part mark (called by the cpu_prof section mark); part 0 = loop start
int report(char* buf, size_t len, const char* const* part_names, int n_parts, uint32_t period_us);
// Loop pass kinds and the end-of-pass wait (bench "rt", second line "RT2"). wait_part: the loop part of the wait; its
// time is not active time. kind of the pass that the next part(0) ends: 0 = no frame read, 1 = frames decoded only,
// 2 = the control step ran. wake: 0 = the wait timed out, 1 = a frame edge woke it, 2 = no wait (input waited).
void setWaitPart(uint8_t wait_part);
void passKind(uint8_t kind);
void wake(uint8_t how);
// "RT2": the window since the last report2: passes and active wall time per kind, wakes, wall time per part in total.
int report2(char* buf, size_t len, const char* const* part_names, int n_parts);
#ifdef DIAG_LAT2
void stmTxAt();                         // a command to the Rx STM32 goes now (test build)
int lat2Line(int i, char* buf, size_t len);  // bench "lat2": line i of the raw stamps of the slow frames (test build)
#endif

}  // namespace rtprof
