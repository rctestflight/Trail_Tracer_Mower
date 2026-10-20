// panic_record: the reason, PC and backtrace of a panic, kept in RTC memory over the panic restart and printed once at
// the next boot (USB line PANIC ...). No flash write. A power cycle loses it. The ESP-IDF panic handler is wrapped
// (platformio.ini: -Wl,--wrap=esp_panic_handler): the record is taken first, then the normal handler runs.
// Decode the PCs with the ELF of the same build: xtensa-esp32s2-elf-addr2line -pfiaC -e firmware.elf <pc> ...
#include <Arduino.h>
#include "esp_attr.h"
#include "esp_debug_helpers.h"
#include "esp_private/panic_internal.h"
#include "freertos/xtensa_context.h"
#include "soc/cpu.h"
#include "soc/soc_memory_layout.h"

namespace {
const uint32_t PANIC_MAGIC = 0x50414E43u;  // "PANC"
const int BT_MAX = 8;
struct PanicRecord {
  uint32_t magic;
  uint32_t exception;  // panic_exception_t: 0 debug, 1 interrupt watchdog, 2 task watchdog, 3 abort, 4 fault
  uint32_t pc, exccause, excvaddr;
  const char* reason;  // a string in flash: valid in the next boot of the same image
  uint32_t bt[BT_MAX];
  uint32_t n;
};
RTC_NOINIT_ATTR PanicRecord g_panic;
}  // namespace

extern "C" void __real_esp_panic_handler(panic_info_t* info);

extern "C" void IRAM_ATTR __wrap_esp_panic_handler(panic_info_t* info) {
  g_panic.magic = 0;
  g_panic.exception = (uint32_t)info->exception;
  g_panic.reason = info->reason;
  g_panic.pc = (uint32_t)info->addr;
  g_panic.exccause = g_panic.excvaddr = 0;
  g_panic.n = 0;
  const XtExcFrame* f = (const XtExcFrame*)info->frame;
  if (f) {
    g_panic.pc = f->pc;
    g_panic.exccause = f->exccause;
    g_panic.excvaddr = f->excvaddr;
    esp_backtrace_frame_t fr = {(uint32_t)f->pc, (uint32_t)f->a1, (uint32_t)f->a0, f};
    for (int i = 0; i < BT_MAX; i++) {
      g_panic.bt[g_panic.n++] = esp_cpu_process_stack_pc(fr.pc);
      if (!fr.next_pc || !esp_backtrace_get_next_frame(&fr)) break;
    }
  }
  g_panic.magic = PANIC_MAGIC;
  __real_esp_panic_handler(info);
}

// The record of a panic before this boot, as one line (no newline); false if none. It is cleared.
bool panicRecordTake(char* out, size_t n) {
  if (g_panic.magic != PANIC_MAGIC || g_panic.n > BT_MAX) { g_panic.magic = 0; return false; }
  g_panic.magic = 0;
  const char* why = g_panic.reason && esp_ptr_in_drom(g_panic.reason) ? g_panic.reason : "?";
  int k = snprintf(out, n, "PANIC exception=%lu reason=\"%.60s\" pc=0x%08lx cause=%lu vaddr=0x%08lx bt=", (unsigned long)g_panic.exception,
                   why, (unsigned long)g_panic.pc, (unsigned long)g_panic.exccause, (unsigned long)g_panic.excvaddr);
  for (uint32_t i = 0; i < g_panic.n && k > 0 && (size_t)k < n; i++)
    k += snprintf(out + k, n - k, i ? " 0x%08lx" : "0x%08lx", (unsigned long)g_panic.bt[i]);
  return true;
}
