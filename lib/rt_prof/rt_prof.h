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
void commandSet();       // the drive command now comes from this frame
void canStart();         // odrive_outputUpdate starts its CAN burst
void canEnd();
void gyroSample();       // an MPU sample was taken
void part(uint8_t part); // loop part mark (called by the cpu_prof section mark); part 0 = loop start
int report(char* buf, size_t len, const char* const* part_names, int n_parts, uint32_t period_us);

}  // namespace rtprof
