// cpu_prof: estimates of CPU use per FreeRTOS task and per part of loop(), by sampling.
//
// The prebuilt Arduino core has no FreeRTOS run-time statistics (CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
// is off). So a hardware timer (group 1, timer 0) interrupts at random intervals, uniform in
// 0.5..1.5 x the mean (default 500 us, 2 kHz; random so that no periodic task aliases with it). Each
// interrupt counts the task that it interrupted, and, when that task is loopTask, the loop part that
// the firmware set last with section(). The share of samples estimates the CPU share: the standard
// error of a share p from N samples is sqrt(p (1 - p) / N), 0.07 % for p = 1 % and N = 20000 (10 s).
//
// Limits: the interrupt is not IRAM-safe, so it cannot run while the flash cache is off (flash erase
// and write) or while interrupts are masked (critical sections). That time gets no samples; report()
// gives it as "unsampled", from the sum of the intervals against the elapsed time. Interrupt time
// counts to the task that it interrupted. The sampler costs about 1 % of the CPU at 2 kHz.
// Diagnostic build (CPUPROF_LEVEL3, env diag_cpu): the interrupt runs at level 3, so it also lands inside level 1 and 2
// interrupt handlers; those samples count as "isr" (interrupt nesting above 1), not to the interrupted task.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace cpuprof {

const int MAX_TASKS = 24;
const int MAX_SECTIONS = 24;  // diag_cpu: 24 (radio sub-parts)

// Start the sampler. Call from loopTask (setup()): that task becomes the one with loop parts.
bool begin(uint32_t mean_us = 500);
// loopTask: the loop part that runs from now (0..MAX_SECTIONS-1).
void section(uint8_t s);
// diag_fp: the loop part that runs now (IRAM: the soft-float counters call it)
uint8_t sectionNow();
// One text line of the shares since the last report (or begin), then a new window starts.
// names: the loop part names, n of them. Returns the length written.
int report(char* buf, size_t len, const char* const* names, int n);
bool running();
// loopTask, at the start of each loop() pass: the pass before it ended. kind: 0 = no frame read, 1 = frames decoded
// only, 2 = the control step ran. report() gives the loopTask samples of each kind ("pass:").
const int PASS_KINDS = 3;
void passEnd(uint8_t kind);
// 0.5.97: the CPU load of the window since the last call (its own window; report() does not touch it), in %:
// busy = all but the idle task; loop = loopTask; drive = the loop parts in drive_mask (bit s = part s). Samples
// that hit no known task count as busy. false: no sample yet.
bool load(uint32_t drive_mask, uint8_t* busy, uint8_t* loop, uint8_t* drive);

#ifdef DIAG_STEPTIME
// DIAG_STEPTIME (2026-10-29, control step timing, observation only): each sample also goes to a ring of the last
// RING_N samples (256 at 2 kHz: 128 ms): its CPU cycle count, the task slot (RING_ISR inside another interrupt
// handler, RING_OTHER beyond MAX_TASKS) and the loop part (0xFF outside loopTask).
struct Ring { uint32_t cyc; uint8_t task, sec, lsec; };  // lsec: the loop's part at the sample, also while the loop waits
const uint32_t RING_N = 256;
const uint8_t RING_ISR = 0xFE, RING_OTHER = 0xFD;
const Ring* ringBase(uint32_t* head);  // the ring and the count of samples written (head % RING_N is the next slot)
const char* taskName(uint8_t i);
uint32_t meanUs();
#endif

}  // namespace cpuprof
