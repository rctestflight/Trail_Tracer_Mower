// Diagnostic build diag_qio (2026-10-28, CPU target 60 %; default build from 0.7.5): the flash cache in QIO mode at
// run time, for the measurement of its CPU gain. The bootloader sets DIO at each reset and the app keeps it, so a failed
// switch ends in a reset and a normal DIO boot. Bench "qio": the flash ID, the status registers and the cache read
// mode. Bench "qio on": sets the quad-enable bit if it is clear (one status register write, only when clear), then the
// cache read mode QIO with its dummy cycles. The mode holds until the next reset.
#ifdef DIAG_QIO
#include <Arduino.h>
#include "usb_text.h"
#include <Preferences.h>
#include "esp_attr.h"
#include "esp_spi_flash.h"
#include "esp_rom_efuse.h"
#include "esp32s2/rom/spi_flash.h"
#include "soc/spi_mem_reg.h"
#include "soc/soc.h"

extern UsbOut usbOut;
extern "C" uint32_t bootloader_execute_flash_command(uint8_t command, uint32_t mosi_data, uint8_t mosi_len, uint8_t miso_len);

namespace {
const uint8_t CMD_RDSR = 0x05, CMD_RDSR2 = 0x35, CMD_WREN = 0x06, CMD_WRSR = 0x01, CMD_WRSR2 = 0x31;
struct QioState { uint32_t id, sr1, sr2, ctrl0, ctrl1, user1, sr2After, sr1After; int done, wrote, wpFix; };

// mfg 0xC8 (GD): status register 2 by its own command, QE = bit 1 of it. Others (Winbond, XMC 0x20 and the IDF
// default): one 16-bit write of both registers, QE = bit 9. MXIC (0xC2) and ISSI (0x9D): QE = bit 6 of register 1.
#ifdef QIO_RTC
RTC_IRAM_ATTR  // QIO_RTC: the switch in RTC fast memory (outside the main internal RAM)
#else
IRAM_ATTR
#endif
// write false: no status write; the switch runs only if the quad-enable bit is already set (else done = -4).
// QIO_FORCE_FAIL (test): the bit counts as clear and the write is left out, so the attempt fails (done = -1).
__attribute__((noinline)) void qioRun(QioState& s, bool on, bool write) {   // noinline: all of it stays in internal RAM
  const spi_flash_guard_funcs_t* g = spi_flash_guard_get();
  const uint32_t wp = esp_rom_efuse_get_flash_wp_gpio(), cfg = esp_rom_efuse_get_flash_gpio_info();   // before the cache stops
  g->start();
  esp_rom_spiflash_wait_idle(&g_rom_flashchip);
  s.id = g_rom_flashchip.device_id;
  s.sr1 = bootloader_execute_flash_command(CMD_RDSR, 0, 0, 8);
  s.sr2 = bootloader_execute_flash_command(CMD_RDSR2, 0, 0, 8);
  s.sr2After = s.sr2;
  s.sr1After = s.sr1;
  s.done = 0;
  s.wrote = s.wpFix = 0;
  const uint8_t mfg = (s.id >> 16) & 0xFF;
  const bool sr1qe = (mfg == 0xC2 || mfg == 0x9D);  // MXIC, ISSI: QE = bit 6 of register 1
#ifdef QIO_FORCE_FAIL
  const bool qe = false;
#else
  const bool qe = sr1qe ? (s.sr1 & 0x40) : (s.sr2 & 0x02);
#endif
  if (on) {
    if (qe) {
      s.done = 1;
    } else if (!write) {
      s.done = -4;
    } else {
#ifndef QIO_FORCE_FAIL
      s.wrote = 1;
      bootloader_execute_flash_command(CMD_WREN, 0, 0, 0);
      if (sr1qe) bootloader_execute_flash_command(CMD_WRSR, s.sr1 | 0x40, 8, 0);
      else if (mfg == 0xC8) bootloader_execute_flash_command(CMD_WRSR2, s.sr2 | 0x02, 8, 0);
      else bootloader_execute_flash_command(CMD_WRSR, s.sr1 | ((s.sr2 | 0x02) << 8), 16, 0);
      esp_rom_spiflash_wait_idle(&g_rom_flashchip);
#endif
      s.sr1After = bootloader_execute_flash_command(CMD_RDSR, 0, 0, 8);
      s.sr2After = bootloader_execute_flash_command(CMD_RDSR2, 0, 0, 8);
#ifdef QIO_FORCE_FAIL
      s.done = -1;
#else
      s.done = (sr1qe ? (s.sr1After & 0x40) : (s.sr2After & 0x02)) ? 1 : -1;
#endif
      // write-protect bits (BP0-BP4, SR1 bits 2-6; MXIC/ISSI: bits 2-5) changed by the write: one restore write in the
      // same attempt, with the quad-enable bit as it now reads
      const uint32_t bp = sr1qe ? 0x3C : 0x7C;
      if (s.wrote && (s.sr1After & bp) != (s.sr1 & bp)) {
        s.wpFix = 1;
        bootloader_execute_flash_command(CMD_WREN, 0, 0, 0);
        if (sr1qe || mfg == 0xC8) bootloader_execute_flash_command(CMD_WRSR, (s.sr1 & bp) | (s.sr1After & ~bp & 0xFF), 8, 0);
        else bootloader_execute_flash_command(CMD_WRSR, ((s.sr1 & bp) | (s.sr1After & ~bp & 0xFF)) | (s.sr2After << 8), 16, 0);
        esp_rom_spiflash_wait_idle(&g_rom_flashchip);
        s.sr1After = bootloader_execute_flash_command(CMD_RDSR, 0, 0, 8);
      }
    }
    // the cache dummy cycles as the bootloader sets them (bootloader_flash_dummy_config): mode cycles + the 80 MHz
    // extra. The switch runs only when the DIO value is the one expected, else done = -2 and nothing changes
    const uint32_t dum = REG_GET_FIELD(SPI_MEM_USER1_REG(0), SPI_MEM_USR_DUMMY_CYCLELEN);
    if (s.done == 1 && dum != (uint32_t)(SPI0_R_DIO_DUMMY_CYCLELEN + g_rom_spiflash_dummy_len_plus[0])) s.done = -2;
    if (s.done == 1) {
      esp_rom_spiflash_select_qio_pins(wp, cfg);
      esp_rom_spiflash_config_readmode(ESP_ROM_SPIFLASH_QIO_MODE);
      REG_SET_FIELD(SPI_MEM_USER1_REG(0), SPI_MEM_USR_DUMMY_CYCLELEN, SPI0_R_QIO_DUMMY_CYCLELEN + g_rom_spiflash_dummy_len_plus[0]);
    }
  }
  s.ctrl0 = REG_READ(SPI_MEM_CTRL_REG(0));
  s.ctrl1 = REG_READ(SPI_MEM_CTRL_REG(1));
  s.user1 = REG_READ(SPI_MEM_USER1_REG(0));
  g->end();
}
}  // namespace

static QioState s_boot = {};
static bool s_booted = false;
static uint8_t s_try = 0;  // the NVS record: 0 no attempt, 1 attempt started, 2 attempt passed, 3 attempt failed
// DIAG_QIO_BOOT: QIO from the start of setup(), every boot. The quad-enable write: at most one attempt per board,
// recorded in NVS before it starts (a reset in the attempt counts as the attempt) and after it ends. With a record
// and the bit clear, the board stays DIO with no write: no repeating flash status write (hard limit).
void qioBoot() {
  qioRun(s_boot, true, false);  // no write: QIO if the bit is already set
  s_booted = true;
  if (s_boot.done != -4) return;
  Preferences p;
  if (!p.begin("wfrx_qio", false)) { s_boot.done = -5; return; }  // no NVS: no attempt
  s_try = p.getUChar("try", 0);
  if (s_try == 0) {
    p.putUChar("try", 1);
    qioRun(s_boot, true, true);  // the one attempt: one status write (+ one write-protect restore if needed)
    s_try = s_boot.done == 1 ? 2 : 3;
    p.putUChar("try", s_try);
  } else {
    s_boot.done = -3;  // an earlier attempt: stay DIO, no write
  }
  p.end();
}

// One USB line after start (the USB console is up): the boot result. Release: an event in the log.
void qioReport() {
  if (!s_booted) return;
  static const char* const what[] = {"QIO", "?", "no switch: dummy cycles not as expected", "earlier attempt recorded, DIO, no write",
                                     "bit clear", "no NVS, DIO, no write"};
  const int d = s_boot.done;
  usbOut.printf("QIO boot: done=%d (%s) try=%u wrote=%d wp_fix=%d sr1=%02lx sr2=%02lx sr1_after=%02lx sr2_after=%02lx\n", d,
                d == 1 ? "QIO" : d == -1 ? "attempt failed, DIO" : (d <= -2 && d >= -5) ? what[-d] : "?", (unsigned)s_try,
                s_boot.wrote, s_boot.wpFix, (unsigned long)s_boot.sr1, (unsigned long)s_boot.sr2,
                (unsigned long)s_boot.sr1After, (unsigned long)s_boot.sr2After);
}

// bench "qio reset": clear the NVS record (bench test of the cap only)
void qioResetRecord() {
  Preferences p;
  const bool ok = p.begin("wfrx_qio", false) && p.remove("try");
  p.end();
  usbOut.printf("QIO record %s\n", ok ? "cleared" : "not found");
}

void qioCommand(bool on) {
  qioReport();
  QioState s = {};
  qioRun(s, on, on);  // bench "qio on": a manual write if the bit is clear (bench only, by hand)
  usbOut.printf("QIO id=%06lx sr1=%02lx sr2=%02lx sr_after=%02lx done=%d ctrl0=%08lx (qio=%d dio=%d) ctrl1=%08lx user1_0=%08lx\n",
                (unsigned long)s.id, (unsigned long)s.sr1, (unsigned long)s.sr2, (unsigned long)s.sr2After, s.done,
                (unsigned long)s.ctrl0, (int)!!(s.ctrl0 & SPI_MEM_FREAD_QIO), (int)!!(s.ctrl0 & SPI_MEM_FREAD_DIO),
                (unsigned long)s.ctrl1, (unsigned long)s.user1);
}
#endif
