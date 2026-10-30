// Diagnostic build diag_fp (2026-10-28, float costs, observation only): counts every soft-float operation and libm call
// by loop part (loopTask), wire solve task and other tasks. The ESP32-S2 has no FPU: each float or double operation is a
// call into the ROM soft-float routines (__adddf3, __mulsf3 and the rest), and the linker sends every such call here
// first (-Wl,--wrap, platformio.ini env diag_fp). Bench commands: "fp" (counts since the last "fp"), "fp bench" (CPU
// cycles of one call of each group, typical operands). Calls made inside a libm function count to that function only.
#ifdef DIAG_FP_COUNT
#include <Arduino.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_timer.h"
#include "hal/cpu_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cpu_prof.h"
#include "usb_text.h"

namespace {
enum : uint8_t { DADD, DMUL, DDIV, DCMP, DCONV, F2D, D2F, FADD, FMUL, FDIV, FCMP, FCONV, DLIBM, FTRIG, FSQRT, FEXP, FROUND, NG };
const char* const GN[NG] = {"d_add", "d_mul", "d_div", "d_cmp", "d_conv", "f2d", "d2f", "f_add", "f_mul", "f_div", "f_cmp",
                            "f_conv", "d_libm", "f_trig", "f_sqrt", "f_exp", "f_round"};
const int SLOTS = cpuprof::MAX_SECTIONS + 2;  // loop parts, then the wire solve task, then every other task
const int S_SOLVE = cpuprof::MAX_SECTIONS, S_OTHER = cpuprof::MAX_SECTIONS + 1;
const uint32_t CAP = 20000;  // cycles: a longer call was preempted; it counts, its cycles do not
DRAM_ATTR uint32_t cnt[SLOTS][NG];
DRAM_ATTR uint64_t cyc[NG];
DRAM_ATTR uint32_t capped = 0;
// inside a libm call: its inner operations do not count. One flag per task class: a shared flag broke when the loop and
// the solve task (same priority, time sliced) were both inside libm calls (first bench run: every count 0)
DRAM_ATTR volatile uint8_t depth_loop = 0, depth_solve = 0, depth_other = 0;
DRAM_ATTR TaskHandle_t solve_task = nullptr, loop_task = nullptr;
DRAM_ATTR bool on = true;
int64_t win_t0 = 0;

inline volatile uint8_t& IRAM_ATTR depthRef() {
  TaskHandle_t h = xTaskGetCurrentTaskHandle();
  return h == loop_task ? depth_loop : h == solve_task ? depth_solve : depth_other;
}
inline int IRAM_ATTR slotNow() {
  TaskHandle_t h = xTaskGetCurrentTaskHandle();
  if (h == loop_task) return cpuprof::sectionNow();
  if (h == solve_task) return S_SOLVE;
  return S_OTHER;
}
inline void IRAM_ATTR account(uint8_t g, uint32_t c0) {
  const uint32_t d = cpu_hal_get_cycle_count() - c0;
  cnt[slotNow()][g]++;
  if (d < CAP) cyc[g] += d; else capped++;
}
}  // namespace

#define FP_WRAP2(G, R, NAME, T1, T2) \
  extern "C" R __real_##NAME(T1, T2); \
  extern "C" R IRAM_ATTR __wrap_##NAME(T1 a, T2 b) { \
    if (!on || depthRef()) return __real_##NAME(a, b); \
    const uint32_t c0 = cpu_hal_get_cycle_count(); R r = __real_##NAME(a, b); account(G, c0); return r; }
#define FP_WRAP1(G, R, NAME, T1) \
  extern "C" R __real_##NAME(T1); \
  extern "C" R IRAM_ATTR __wrap_##NAME(T1 a) { \
    if (!on || depthRef()) return __real_##NAME(a); \
    const uint32_t c0 = cpu_hal_get_cycle_count(); R r = __real_##NAME(a); account(G, c0); return r; }
// libm: the inner soft-float calls do not count (depth)
#define LM_WRAP1(G, R, NAME, T1) \
  extern "C" R __real_##NAME(T1); \
  extern "C" R __wrap_##NAME(T1 a) { \
    if (!on || depthRef()) return __real_##NAME(a); \
    volatile uint8_t& dp = depthRef(); dp = dp + 1; const uint32_t c0 = cpu_hal_get_cycle_count(); R r = __real_##NAME(a); account(G, c0); dp = dp - 1; return r; }
#define LM_WRAP2(G, R, NAME, T1, T2) \
  extern "C" R __real_##NAME(T1, T2); \
  extern "C" R __wrap_##NAME(T1 a, T2 b) { \
    if (!on || depthRef()) return __real_##NAME(a, b); \
    volatile uint8_t& dp = depthRef(); dp = dp + 1; const uint32_t c0 = cpu_hal_get_cycle_count(); R r = __real_##NAME(a, b); account(G, c0); dp = dp - 1; return r; }

FP_WRAP2(DADD, double, __adddf3, double, double)
FP_WRAP2(DADD, double, __subdf3, double, double)
FP_WRAP2(DMUL, double, __muldf3, double, double)
FP_WRAP2(DDIV, double, __divdf3, double, double)
FP_WRAP2(DCMP, int, __eqdf2, double, double)
FP_WRAP2(DCMP, int, __nedf2, double, double)
FP_WRAP2(DCMP, int, __ltdf2, double, double)
FP_WRAP2(DCMP, int, __ledf2, double, double)
FP_WRAP2(DCMP, int, __gtdf2, double, double)
FP_WRAP2(DCMP, int, __gedf2, double, double)
FP_WRAP2(DCMP, int, __unorddf2, double, double)
FP_WRAP1(DCONV, double, __floatsidf, int)
FP_WRAP1(DCONV, double, __floatunsidf, unsigned)
FP_WRAP1(DCONV, double, __floatdidf, long long)
FP_WRAP1(DCONV, double, __floatundidf, unsigned long long)
FP_WRAP1(DCONV, int, __fixdfsi, double)
FP_WRAP1(DCONV, unsigned, __fixunsdfsi, double)
FP_WRAP1(F2D, double, __extendsfdf2, float)
FP_WRAP1(D2F, float, __truncdfsf2, double)
FP_WRAP2(FADD, float, __addsf3, float, float)
FP_WRAP2(FADD, float, __subsf3, float, float)
FP_WRAP2(FMUL, float, __mulsf3, float, float)
FP_WRAP2(FDIV, float, __divsf3, float, float)
FP_WRAP2(FCMP, int, __eqsf2, float, float)
FP_WRAP2(FCMP, int, __nesf2, float, float)
FP_WRAP2(FCMP, int, __ltsf2, float, float)
FP_WRAP2(FCMP, int, __lesf2, float, float)
FP_WRAP2(FCMP, int, __gtsf2, float, float)
FP_WRAP2(FCMP, int, __gesf2, float, float)
FP_WRAP2(FCMP, int, __unordsf2, float, float)
FP_WRAP1(FCONV, float, __floatsisf, int)
FP_WRAP1(FCONV, float, __floatunsisf, unsigned)
FP_WRAP1(FCONV, float, __floatundisf, unsigned long long)
FP_WRAP1(FCONV, int, __fixsfsi, float)
FP_WRAP1(FCONV, unsigned, __fixunssfsi, float)
LM_WRAP1(DLIBM, double, sin, double)
LM_WRAP1(DLIBM, double, sqrt, double)
LM_WRAP2(DLIBM, double, fmod, double, double)
LM_WRAP1(DLIBM, double, floor, double)
LM_WRAP1(DLIBM, double, round, double)
LM_WRAP1(DLIBM, long, lround, double)
LM_WRAP1(FTRIG, float, sinf, float)
LM_WRAP1(FTRIG, float, cosf, float)
LM_WRAP2(FTRIG, float, atan2f, float, float)
LM_WRAP1(FTRIG, float, asinf, float)
LM_WRAP1(FTRIG, float, acosf, float)
LM_WRAP1(FTRIG, float, atanf, float)
LM_WRAP1(FSQRT, float, sqrtf, float)
LM_WRAP2(FSQRT, float, hypotf, float, float)
LM_WRAP1(FEXP, float, expf, float)
LM_WRAP1(FEXP, float, logf, float)
LM_WRAP2(FEXP, float, powf, float, float)
LM_WRAP1(FEXP, float, log10f, float)
LM_WRAP1(FROUND, long, lroundf, float)
LM_WRAP1(FROUND, float, floorf, float)
LM_WRAP1(FROUND, float, ceilf, float)

extern UsbOut usbOut;

// bench "fp": counts by slot since the last "fp", the mean cycles of each group, the CPU share of each group.
void fpCountSetLoopTask() { loop_task = xTaskGetCurrentTaskHandle(); }

void fpCountReport(const char* const* sec_names, int n_sec) {
  if (!loop_task) loop_task = xTaskGetCurrentTaskHandle();
  if (!solve_task) solve_task = xTaskGetHandle("wsolve");
  static uint32_t c[SLOTS][NG];
  static uint64_t y[NG];
  const int64_t now = esp_timer_get_time();
  portDISABLE_INTERRUPTS();
  memcpy(c, cnt, sizeof(c)); memcpy(y, cyc, sizeof(y));
  const uint32_t cp = capped;
  memset(cnt, 0, sizeof(cnt)); memset(cyc, 0, sizeof(cyc)); capped = 0;
  portENABLE_INTERRUPTS();
  const double win_cyc = win_t0 ? (double)(now - win_t0) * getCpuFrequencyMhz() : 0;
  win_t0 = now;
  usbOut.printf("FP window_ms=%.0f capped=%lu (counted, no cycles) depth=%u/%u/%u on=%d\n", win_cyc / getCpuFrequencyMhz() / 1000,
                (unsigned long)cp, depth_loop, depth_solve, depth_other, (int)on);
  // per group: calls, mean cycles, CPU %
  char b[700];
  int k = snprintf(b, sizeof(b), "FPG");
  for (int g = 0; g < NG; g++) {
    uint32_t n = 0;
    for (int s = 0; s < SLOTS; s++) n += c[s][g];
    if (n) k += snprintf(b + k, sizeof(b) - k, " %s=%lu/%.0f/%.2f%%", GN[g], (unsigned long)n, (double)y[g] / n,
                         win_cyc > 0 ? 100.0 * (double)y[g] / win_cyc : 0.0);
  }
  usbOut.printf("%s\n", b);
  // per slot: calls of each group
  for (int s = 0; s < SLOTS; s++) {
    uint32_t tot = 0;
    for (int g = 0; g < NG; g++) tot += c[s][g];
    if (!tot) continue;
    const char* nm = s == S_SOLVE ? "wsolve" : s == S_OTHER ? "other" : (s < n_sec ? sec_names[s] : "?");
    k = snprintf(b, sizeof(b), "FPS %s", nm);
    for (int g = 0; g < NG; g++) if (c[s][g]) k += snprintf(b + k, sizeof(b) - k, " %s=%lu", GN[g], (unsigned long)c[s][g]);
    if (usbOut.room() < 300) usbOut.drain(100);
    usbOut.printf("%s\n", b);
  }
}

// bench "fp bench": CPU cycles of one call of each group (typical operands, 2000 calls each, the loop counted out).
void fpCountBench() {
  volatile double da = 1.2345678, db = 0.98765, dr = 0;
  volatile float fa = 1.2345678f, fb = 0.98765f, fr = 0;
  volatile int ir = 0;
  const int N = 2000;
  const bool was = on;
  on = false;
#define FP_RUN(NM, EXPR) do {     const uint32_t c0 = cpu_hal_get_cycle_count();     for (int i = 0; i < N; i++) { EXPR; }     const uint32_t c1 = cpu_hal_get_cycle_count();     for (int i = 0; i < N; i++) { ir = ir + 1; }     const uint32_t c2 = cpu_hal_get_cycle_count();     usbOut.printf(" %s=%.0f", NM, ((double)(c1 - c0) - (double)(c2 - c1)) / N); } while (0)
  usbOut.printf("FPBENCH cycles per call:");
  FP_RUN("d_add", dr = da + db);
  FP_RUN("d_mul", dr = da * db);
  FP_RUN("d_div", dr = da / db);
  FP_RUN("d_cmp", ir = da < db);
  FP_RUN("d_conv", dr = (double)ir);
  FP_RUN("f2d", dr = (double)fa);
  FP_RUN("d2f", fr = (float)da);
  FP_RUN("f_add", fr = fa + fb);
  FP_RUN("f_mul", fr = fa * fb);
  FP_RUN("f_div", fr = fa / fb);
  FP_RUN("f_cmp", ir = fa < fb);
  FP_RUN("f_conv", fr = (float)ir);
  FP_RUN("sin", dr = sin(da));
  FP_RUN("sqrt", dr = sqrt(da));
  FP_RUN("sinf", fr = sinf(fa));
  FP_RUN("cosf", fr = cosf(fa));
  FP_RUN("atan2f", fr = atan2f(fa, fb));
  FP_RUN("asinf", fr = asinf(fb * 0.5f));
  FP_RUN("sqrtf", fr = sqrtf(fa));
  FP_RUN("hypotf", fr = hypotf(fa, fb));
  FP_RUN("expf", fr = expf(fb));
  FP_RUN("logf", fr = logf(fa));
  FP_RUN("lroundf", ir = (int)lroundf(fa));
  usbOut.printf("\n");
  (void)dr; (void)fr;
  on = was;
}
#endif
