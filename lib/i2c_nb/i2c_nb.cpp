// i2c_nb: see i2c_nb.h. Register sequence as ESP-IDF 4.4 i2c_param_config() and i2c_master_cmd_begin() for the
// ESP32-S2, without the driver task wait.
#include "i2c_nb.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "hal/cpu_hal.h"
#include "soc/gpio_periph.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2c_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/i2c_periph.h"
#include "soc/i2c_struct.h"
#include "soc/soc.h"

namespace i2cnb {
namespace {

struct Job { uint8_t tag, addr, reg, n, val; bool rd; };
const int QN = 16;  // queue length (a power of 2)
Job q[QN];
volatile uint8_t q_head = 0, q_tail = 0;  // head: next free; tail: next to start
Job cur;                    // the job on the bus
volatile bool on_bus = false, err = false;
volatile int64_t t_start = 0;
Result res[N_TAGS];
volatile uint8_t pend[N_TAGS];  // jobs per tag queued or on the bus
Counts cnt;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
intr_handle_t intr = nullptr;
int pin_sda = -1, pin_scl = -1;
uint32_t bus_hz = 0;
i2c_dev_t* const hw = &I2C0;
// The register fields as hal/i2c_ll.h for the ESP32-S2 (that header does not compile as C++).
union HwCmd {
  struct { uint32_t byte_num : 8, ack_en : 1, ack_exp : 1, ack_val : 1, op_code : 3, reserved14 : 17, done : 1; };
  uint32_t val;
};
enum : uint8_t { OP_RSTART = 0, OP_WRITE = 1, OP_READ = 2, OP_STOP = 3 };
const uint32_t INT_ALL = 0x1ffff;
const uint32_t FIFO_ADDR = 0x6001301c;  // I2C0 FIFO data register (APB address)
inline void fifoRst() {
  hw->fifo_conf.tx_fifo_rst = 1; hw->fifo_conf.tx_fifo_rst = 0;
  hw->fifo_conf.rx_fifo_rst = 1; hw->fifo_conf.rx_fifo_rst = 0;
}

void configure() {  // the controller from reset: master, FIFO mode, MSB first, filter 7, bus timing at bus_hz
  periph_module_enable(PERIPH_I2C0_MODULE);
  hw->int_ena.val = 0;
  hw->int_clr.val = INT_ALL;
  hw->ctr.val = 0;  // i2c_ll_master_init: master, open drain through the GPIO matrix
  hw->ctr.ms_mode = 1;
  hw->ctr.sda_force_out = 1;
  hw->ctr.scl_force_out = 1;
  hw->fifo_conf.nonfifo_en = 0;  // FIFO mode
  hw->ctr.tx_lsb_first = 0;      // MSB first
  hw->ctr.rx_lsb_first = 0;
  fifoRst();
  hw->scl_filter_cfg.thres = 7; hw->sda_filter_cfg.thres = 7;  // filter 7, as the driver
  hw->scl_filter_cfg.en = 1; hw->sda_filter_cfg.en = 1;
  // i2c_ll_cal_bus_clk() and i2c_ll_set_bus_timing() from the APB clock (80 MHz), as the driver
  const uint32_t half = 80 * 1000 * 1000 / bus_hz / 2;
  hw->ctr.ref_always_on = 1;  // APB clock
  hw->scl_low_period.period = half - 1;
  hw->scl_high_period.period = half / 2 + 2;
  hw->scl_high_period.scl_wait_high_period = half - (half / 2 + 2);
  hw->sda_hold.time = half / 2;
  hw->sda_sample.time = half / 2 - 1;
  hw->scl_rstart_setup.time = half;
  hw->scl_stop_setup.time = half;
  hw->scl_start_hold.time = half - 1;
  hw->scl_stop_hold.time = half;
  hw->timeout.tout = half * 20;  // 10 bus cycles, as the driver
  hw->timeout.time_out_en = 1;
  hw->int_ena.val = I2C_NACK_INT_ENA_M | I2C_TIME_OUT_INT_ENA_M | I2C_TRANS_COMPLETE_INT_ENA_M | I2C_ARBITRATION_LOST_INT_ENA_M;
}

void attachPins() {
  const i2c_signal_conn_t& s = i2c_periph_signal[0];
  esp_rom_gpio_connect_out_signal(pin_sda, s.sda_out_sig, false, false);
  esp_rom_gpio_connect_in_signal(pin_sda, s.sda_in_sig, false);
  esp_rom_gpio_connect_out_signal(pin_scl, s.scl_out_sig, false, false);
  esp_rom_gpio_connect_in_signal(pin_scl, s.scl_in_sig, false);
}

// A device that lost a clock mid-byte holds SDA low. The pins as plain open-drain outputs: up to 9 clocks until SDA
// is high, then a stop. About 100 us; only at begin() and after an error with SDA low.
void recoverIfStuck() {
  if (gpio_get_level((gpio_num_t)pin_sda)) return;
  cnt.recover++;
  esp_rom_gpio_connect_out_signal(pin_sda, SIG_GPIO_OUT_IDX, false, false);
  esp_rom_gpio_connect_out_signal(pin_scl, SIG_GPIO_OUT_IDX, false, false);
  gpio_set_level((gpio_num_t)pin_sda, 1);
  for (int i = 0; i < 9 && !gpio_get_level((gpio_num_t)pin_sda); i++) {
    gpio_set_level((gpio_num_t)pin_scl, 0); esp_rom_delay_us(5);
    gpio_set_level((gpio_num_t)pin_scl, 1); esp_rom_delay_us(5);
  }
  gpio_set_level((gpio_num_t)pin_sda, 0); esp_rom_delay_us(5);  // stop: SDA rises while SCL is high
  gpio_set_level((gpio_num_t)pin_scl, 1); esp_rom_delay_us(5);
  gpio_set_level((gpio_num_t)pin_sda, 1); esp_rom_delay_us(5);
  attachPins();
}

__attribute__((always_inline)) inline uint32_t cmd(uint8_t op, uint8_t bytes, bool ack_check, uint8_t ack_val) {
  HwCmd c; c.val = 0;
  c.op_code = op; c.byte_num = bytes; c.ack_en = ack_check; c.ack_exp = 0; c.ack_val = ack_val;
  return c.val;
}

// mux held. Loads the next job's command list and starts it.
void IRAM_ATTR startNext() {
  if (on_bus || err || q_tail == q_head) return;
  cur = q[q_tail & (QN - 1)];
  q_tail = q_tail + 1;
  fifoRst();
  hw->int_clr.val = INT_ALL;
  uint8_t tx[3] = {(uint8_t)(cur.addr << 1), cur.reg, 0};
  int k = 0;
  hw->command[k++].val = cmd(OP_RSTART, 0, false, 0);
  if (!cur.rd) {
    tx[2] = cur.val;
    hw->command[k++].val = cmd(OP_WRITE, 3, true, 0);
  } else {
    tx[2] = (uint8_t)(cur.addr << 1 | 1);
    hw->command[k++].val = cmd(OP_WRITE, 2, true, 0);
    hw->command[k++].val = cmd(OP_RSTART, 0, false, 0);
    hw->command[k++].val = cmd(OP_WRITE, 1, true, 0);
    if (cur.n > 1) hw->command[k++].val = cmd(OP_READ, cur.n - 1, false, 0);  // ACK each byte
    hw->command[k++].val = cmd(OP_READ, 1, false, 1);                         // NACK the last
  }
  hw->command[k++].val = cmd(OP_STOP, 0, false, 0);
  for (int i = 0; i < 3; i++) WRITE_PERI_REG(FIFO_ADDR, tx[i]);
  on_bus = true;
  t_start = esp_timer_get_time();
  hw->ctr.trans_start = 1;
}

// mux held. Ends the job on the bus with status st.
void IRAM_ATTR finish(uint8_t st) {
  const int64_t now = esp_timer_get_time();
  Result& r = res[cur.tag];
  r.status = st;
  r.n = 0;
  r.t_us = now;
  r.xfer_us = (uint32_t)(now - t_start);
  if (st == ST_OK && cur.rd) {
    for (int i = 0; i < cur.n; i++) r.data[i] = (uint8_t)READ_PERI_REG(FIFO_ADDR);
    r.n = cur.n;
  }
  r.seq++;
  if (pend[cur.tag]) pend[cur.tag]--;
  on_bus = false;
  // if-else, not switch: a jump table would sit in flash, which this interrupt cannot read during a flash write
  if (st == ST_OK) { cnt.ok++; cnt.xfer_us_sum += r.xfer_us; if (r.xfer_us > cnt.xfer_us_max) cnt.xfer_us_max = r.xfer_us; }
  else if (st == ST_NACK) cnt.nack++;
  else if (st == ST_TIMEOUT) cnt.timeout++;
  else if (st == ST_ARB) cnt.arb++;
  else cnt.stall++;
  if (st != ST_OK) err = true;  // service() resets the controller, then starts the queue again
}

void IRAM_ATTR isr(void*) {
  const uint32_t c0 = cpu_hal_get_cycle_count();
  const uint32_t s = hw->int_status.val;
  hw->int_clr.val = s;
  portENTER_CRITICAL_ISR(&mux);
  if (on_bus) {
    const uint8_t st = (s & I2C_ARBITRATION_LOST_INT_ST_M) ? ST_ARB : (s & I2C_NACK_INT_ST_M) ? ST_NACK
                       : (s & I2C_TIME_OUT_INT_ST_M) ? ST_TIMEOUT : (s & I2C_TRANS_COMPLETE_INT_ST_M) ? ST_OK : ST_NONE;
    if (st != ST_NONE) {
      finish(st);
      startNext();
    }
  }
  const uint32_t c = cpu_hal_get_cycle_count() - c0;
  cnt.isr_n++; cnt.isr_cyc_sum += c; if (c > cnt.isr_cyc_max) cnt.isr_cyc_max = c;
  portEXIT_CRITICAL_ISR(&mux);
}

bool queue(const Job& j) {
  bool ok;
  portENTER_CRITICAL(&mux);
  ok = (uint8_t)(q_head - q_tail) < QN;
  if (ok) {
    q[q_head & (QN - 1)] = j;
    q_head = q_head + 1;
    pend[j.tag]++;
    startNext();
  } else {
    cnt.full++;
  }
  portEXIT_CRITICAL(&mux);
  return ok;
}

}  // namespace

bool begin(int sda, int scl, uint32_t hz) {
  pin_sda = sda; pin_scl = scl; bus_hz = hz;
  const int pins[2] = {sda, scl};
  for (int p : pins) {
    gpio_set_level((gpio_num_t)p, 1);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p], PIN_FUNC_GPIO);  // routed through the GPIO matrix
    gpio_set_direction((gpio_num_t)p, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode((gpio_num_t)p, GPIO_PULLUP_ONLY);  // as Wire; the board pull-ups also hold the lines
  }
  configure();
  attachPins();
  recoverIfStuck();
  if (!intr && esp_intr_alloc(ETS_I2C_EXT0_INTR_SOURCE, ESP_INTR_FLAG_IRAM, isr, nullptr, &intr) != ESP_OK) return false;
  return true;
}

uint32_t clockHz() { return bus_hz; }

bool read(uint8_t tag, uint8_t addr, uint8_t reg, uint8_t n) {
  if (tag >= N_TAGS || n < 1 || n > MAX_READ) return false;
  return queue(Job{tag, addr, reg, n, 0, true});
}

bool write(uint8_t tag, uint8_t addr, uint8_t reg, uint8_t val) {
  if (tag >= N_TAGS) return false;
  return queue(Job{tag, addr, reg, 0, val, false});
}

bool take(uint8_t tag, uint32_t* seen, Result* out) {
  if (tag >= N_TAGS || res[tag].seq == *seen) return false;  // one word read: no lock for the common case
  portENTER_CRITICAL(&mux);
  *out = res[tag];
  portEXIT_CRITICAL(&mux);
  *seen = out->seq;
  return true;
}

bool pending(uint8_t tag) { return tag < N_TAGS && pend[tag] != 0; }

void service() {
  if (on_bus && !err && esp_timer_get_time() - t_start > (int64_t)STALL_US) {
    portENTER_CRITICAL(&mux);
    if (on_bus && esp_timer_get_time() - t_start > (int64_t)STALL_US) finish(ST_STALL);
    portEXIT_CRITICAL(&mux);
  }
  if (!err) return;
  // After an error: the controller from reset (as the driver's hardware FSM reset), the bus freed if held.
  portENTER_CRITICAL(&mux);
  hw->int_ena.val = 0;
  portEXIT_CRITICAL(&mux);
  periph_module_reset(PERIPH_I2C0_MODULE);
  configure();
  recoverIfStuck();
  cnt.resets++;
  portENTER_CRITICAL(&mux);
  err = false;
  startNext();
  portEXIT_CRITICAL(&mux);
}

void counts(Counts* c, bool reset) {
  portENTER_CRITICAL(&mux);
  *c = cnt;
  if (reset) memset(&cnt, 0, sizeof(cnt));
  portEXIT_CRITICAL(&mux);
}

}  // namespace i2cnb
