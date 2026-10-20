// cpu_prof: see cpu_prof.h.
#include "cpu_prof.h"

#include <stdio.h>
#include <string.h>

#include <Arduino.h>  // getCpuFrequencyMhz()

#include "driver/timer.h"
#include "hal/cpu_hal.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace cpuprof {

namespace {

const timer_group_t GROUP = TIMER_GROUP_1;
const timer_idx_t TIMER = TIMER_0;

struct Task {
  TaskHandle_t h;
  uint32_t n;
  char name[configMAX_TASK_NAME_LEN];
};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Task tasks[MAX_TASKS];
int n_tasks = 0;
uint32_t sec_n[MAX_SECTIONS];
uint32_t samples = 0, other = 0;
uint32_t l_samples = 0, l_idle = 0, l_loop = 0, l_sec[MAX_SECTIONS];  // the load() window (0.5.97)
TaskHandle_t idle_task = nullptr;
uint64_t interval_sum_us = 0;
uint64_t isr_cycles = 0;       // the sampler's own cost: CPU cycles inside onTimer  // the sampled time: the sum of the intervals that ended in a sample
uint32_t cur_interval_us = 500;
uint32_t mean_us_ = 500;
uint32_t rng = 0x9E3779B9u;
int64_t window_start_us = 0;
volatile uint8_t cur_section = 0;
TaskHandle_t loop_task = nullptr;
bool running_ = false;

bool IRAM_ATTR onTimer(void*) {
  const uint32_t c0 = cpu_hal_get_cycle_count();
  TaskHandle_t h = xTaskGetCurrentTaskHandle();
  portENTER_CRITICAL_ISR(&mux);
  samples++;
  interval_sum_us += cur_interval_us;
  int i = 0;
  while (i < n_tasks && tasks[i].h != h) i++;
  if (i < n_tasks) {
    tasks[i].n++;
  } else if (n_tasks < MAX_TASKS) {
    tasks[i].h = h;
    tasks[i].n = 1;
    strncpy(tasks[i].name, pcTaskGetTaskName(h), sizeof(tasks[i].name) - 1);
    tasks[i].name[sizeof(tasks[i].name) - 1] = 0;
    n_tasks++;
  } else {
    other++;
  }
  if (h == loop_task) sec_n[cur_section]++;
  l_samples++;
  if (h == idle_task) l_idle++;
  else if (h == loop_task) { l_loop++; l_sec[cur_section]++; }
  rng ^= rng << 13;  // xorshift32
  rng ^= rng >> 17;
  rng ^= rng << 5;
  cur_interval_us = mean_us_ / 2 + rng % (mean_us_ + 1);  // the next interval, uniform in 0.5..1.5 x mean
  portEXIT_CRITICAL_ISR(&mux);
  timer_group_set_alarm_value_in_isr(GROUP, TIMER, cur_interval_us);  // auto reload: counts from 0 since this alarm
  isr_cycles += cpu_hal_get_cycle_count() - c0;  // not exact: the next sample adds it (no lock needed, one writer)
  return false;
}

}  // namespace

bool begin(uint32_t mean_us) {
  if (running_) return true;
  mean_us_ = mean_us < 100 ? 100 : mean_us;
  cur_interval_us = mean_us_;
  loop_task = xTaskGetCurrentTaskHandle();
  timer_config_t cfg = {};
  cfg.alarm_en = TIMER_ALARM_EN;
  cfg.counter_en = TIMER_PAUSE;
  cfg.intr_type = TIMER_INTR_LEVEL;
  cfg.counter_dir = TIMER_COUNT_UP;
  cfg.auto_reload = TIMER_AUTORELOAD_EN;
  cfg.divider = 80;  // APB 80 MHz: 1 us per count
  if (timer_init(GROUP, TIMER, &cfg) != ESP_OK) return false;
  timer_set_counter_value(GROUP, TIMER, 0);
  timer_set_alarm_value(GROUP, TIMER, cur_interval_us);
  timer_enable_intr(GROUP, TIMER);
  if (timer_isr_callback_add(GROUP, TIMER, onTimer, nullptr, 0) != ESP_OK) return false;
  window_start_us = esp_timer_get_time();
  if (timer_start(GROUP, TIMER) != ESP_OK) return false;
  running_ = true;
  return true;
}

void section(uint8_t s) { cur_section = s < MAX_SECTIONS ? s : MAX_SECTIONS - 1; }

bool load(uint32_t drive_mask, uint8_t* busy, uint8_t* loop, uint8_t* drive) {
  if (!idle_task) idle_task = xTaskGetIdleTaskHandle();
  portENTER_CRITICAL(&mux);
  const uint32_t n = l_samples, idle = l_idle, lp = l_loop;
  uint32_t dr = 0;
  for (int i = 0; i < MAX_SECTIONS; i++) if ((drive_mask >> i) & 1u) dr += l_sec[i];
  l_samples = l_idle = l_loop = 0;
  memset(l_sec, 0, sizeof(l_sec));
  portEXIT_CRITICAL(&mux);
  if (!n) return false;
  *busy = (uint8_t)((100u * (n - idle) + n / 2) / n);
  *loop = (uint8_t)((100u * lp + n / 2) / n);
  *drive = (uint8_t)((100u * dr + n / 2) / n);
  return true;
}

bool running() { return running_; }

int report(char* buf, size_t len, const char* const* names, int n) {
  Task t[MAX_TASKS];
  uint32_t sn[MAX_SECTIONS], ns, no;
  uint64_t covered, cyc;
  int nt;
  portENTER_CRITICAL(&mux);
  const int64_t now = esp_timer_get_time();
  nt = n_tasks;
  memcpy(t, tasks, sizeof(Task) * nt);
  memcpy(sn, sec_n, sizeof(sn));
  ns = samples;
  no = other;
  covered = interval_sum_us;
  cyc = isr_cycles;
  isr_cycles = 0;
  for (int i = 0; i < n_tasks; i++) tasks[i].n = 0;  // handles and names stay: a task keeps its slot
  memset(sec_n, 0, sizeof(sec_n));
  samples = 0;
  other = 0;
  interval_sum_us = 0;
  const int64_t start = window_start_us;
  window_start_us = now;
  portEXIT_CRITICAL(&mux);
  const double el = (double)(now - start);
  const double unsampled = el > 0 ? 100.0 * (1.0 - (double)covered / el) : 0;
  // self: the time inside onTimer only; the interrupt entry and exit add about 1-2 us a sample more
  const double self = el > 0 ? 100.0 * (double)cyc / getCpuFrequencyMhz() / el : 0;
  size_t k = (size_t)snprintf(buf, len, "CPU ms=%lu samples=%lu mean_us=%lu unsampled=%.2f%% self=%.2f%% tasks:", (unsigned long)(el / 1000),
                              (unsigned long)ns, (unsigned long)mean_us_, unsampled, self);
  if (!ns) ns = 1;
  for (int i = 0; i < nt && k < len; i++) {  // largest first
    int b = -1;
    for (int j = 0; j < nt; j++)
      if (t[j].h && (b < 0 || t[j].n > t[b].n)) b = j;
    if (b < 0 || !t[b].n) break;
    k += (size_t)snprintf(buf + k, len - k, " %s=%.2f", t[b].name, 100.0 * t[b].n / ns);
    t[b].h = nullptr;
  }
  if (no && k < len) k += (size_t)snprintf(buf + k, len - k, " other=%.2f", 100.0 * no / ns);
  if (k < len) k += (size_t)snprintf(buf + k, len - k, " | loop:");
  for (int s = 0; s < n && s < MAX_SECTIONS && k < len; s++)
    if (sn[s]) k += (size_t)snprintf(buf + k, len - k, " %s=%.2f", names[s], 100.0 * sn[s] / ns);
  if (k >= len) k = len - 1;
  return (int)k;
}

}  // namespace cpuprof
