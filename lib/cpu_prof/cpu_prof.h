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
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace cpuprof {

const int MAX_TASKS = 24;
const int MAX_SECTIONS = 16;

// Start the sampler. Call from loopTask (setup()): that task becomes the one with loop parts.
bool begin(uint32_t mean_us = 500);
// loopTask: the loop part that runs from now (0..MAX_SECTIONS-1).
void section(uint8_t s);
// One text line of the shares since the last report (or begin), then a new window starts.
// names: the loop part names, n of them. Returns the length written.
int report(char* buf, size_t len, const char* const* names, int n);
bool running();

}  // namespace cpuprof
