/*
 * rctf_wfrx_comms_v2.h — Trail Tracer v2.0 STM32 <-> ESP32 link.
 * Release-candidate prototype: frame layout 2 (app 2.0.10).
 *
 * Shared header: the STM32 main app and the ESP32 firmware use the same copy.
 * The STM32 side is in TrailTracer-Rx-STM32; the ESP32 side is in this repository.
 *
 * The STM32 is a generic 2-sensor, 8-bin DSP front end. The ESP32 owns the Trail Tracer
 * setup (bin frequencies, gains, calibration, thresholds, frame period) by command. All
 * ESP32 settings are in RAM; status bit 21 asks for them again after each STM32 reset.
 * Parameters are set or read by command, never streamed.
 *
 * Physical: USART 2 Mbaud 8N1. Little-endian. All structs packed; static asserts fix the
 * offsets. Fields after offset 13 are not aligned: both compilers generate byte access.
 *
 * Framing: the frame tick frames both directions; no sync bytes.
 *   STM32 -> ESP32: one 128 B frame per nDATA_READY falling edge.
 *   ESP32 -> STM32: the bytes received between two frame ticks are one command window.
 * 128 B equals the ESP32-S2 RX FIFO: the ESP32 must set its FIFO-full threshold well below
 * 120 B (for example 64 B).
 *
 * Version detection: proto_ver 2 at offset 0, length 128 B (layout 2), good CRC. Layout 1
 * (98 B, app 2.0.0/2.0.1) was a prototype.
 *
 * CRC: poly 0x8005, init 0xFFFF, no reflection, no final XOR (CRC-16/CMS).
 * Check value: CRC over ASCII "123456789" = 0xAEE7. Covers offset 0 up to the crc field.
 */

#ifndef RCTF_WFRX_COMMS_V2_H
#define RCTF_WFRX_COMMS_V2_H

#include <stdint.h>
#include <stddef.h>

#define WFRX_PROTO_VER              2u
#define WFRX_LAYOUT                 2u

#ifdef __cplusplus
#define WFRX_STATIC_ASSERT(c, m)    static_assert(c, m)
#else
#define WFRX_STATIC_ASSERT(c, m)    _Static_assert(c, m)
#endif

/* ===========================================================================
 * Coil data: 2 results x 8 bins x 2 int24 values.
 *
 * Result r = lane r: lane 1 then lane 2 in each frame. The lane input (sensor 1 or 2) is
 * set by CID 13 and reported in status bits 22-23.
 *
 * Mode per bin (CID 25 bit mask; bit b = bin b; default 0 = all magnitude):
 *   0 signed magnitude: v1 = s1, v2 = s2. LSB 2^-24 magnitude unit.
 *     sN = |GN| x PGA calibration of coil N (factory, NVM). Sign from the tracked
 *     quadrant: q 0 (+,+), q -1 (-,+), q +1 (+,-), q 2 (-,-). No bin gain applied.
 *     Magnitude unit: about 1/mm for a wire bin at unit gain and radius calibration 1.
 *   1 xy: v1 = x, v2 = y. LSB 0.001 mm. Computed with the bin gain (CID 26), radius
 *     calibration (CID 27) and angle calibration (CID 28):
 *       (a, b) = rotate(g*s1, g*s2 by -angle) / radius_cal; x = a/(a^2+b^2), y = b/(a^2+b^2).
 * A value of +/-(2^23 - 1) means saturated. A bin that is off gives zeros.
 * Every value is sent in every frame, in every signal state (no zeroing).
 *
 * ESP32-side conversion, mode 0, wire bin: with gain g, radius R, angle phi, the same
 * formula gives x, y and r = 1/sqrt(a^2 + b^2) in mm.
 * ========================================================================= */

#define WFRX_BINS                   8u
#define WFRX_RESULTS                2u
#define WFRX_INT24_MAX              8388607L
#define WFRX_MAG_LSB_SHIFT          24      /* magnitude = value / 2^24 */
#define WFRX_XY_LSB_MM              0.001f  /* x, y = value * 0.001 mm */

typedef struct __attribute__((packed)) {
    uint8_t  v1[3];             /* int24 LE: s1, or x */
    uint8_t  v2[3];             /* int24 LE: s2, or y */
} WfrxCoilPair;

static inline void wfrx_int24_put(uint8_t *p, int32_t v)
{
    if (v > WFRX_INT24_MAX)  { v = WFRX_INT24_MAX; }
    if (v < -WFRX_INT24_MAX) { v = -WFRX_INT24_MAX; }
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}

static inline int32_t wfrx_int24_get(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 8) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 24)) >> 8;
}

/* ===========================================================================
 * Pages: one per frame, page = seq & 1. Run-time values only, fixed point.
 * ========================================================================= */

typedef struct __attribute__((packed)) {
    uint16_t vbatt_mv;
    uint16_t vcharge_mv;
    int16_t  icharge_ma;
    uint16_t v5v_mv;
    uint16_t vesp_3v3_mv;
    int16_t  temperature_dc;    /* 0.1 degC */
    int16_t  charge_timer_s;    /* > 0 charging time so far, < 0 cooldown left */
    uint8_t  charge_code;       /* WFRX_CHARGE_CODE_* bits */
    uint8_t  slow_divider;      /* slow tasks (slow ADC, charge, supply faults) run every
                                   slow_divider frames: 4, 2 or 1, the largest that keeps
                                   them at 10 Hz or more; MEAS values update at that rate */
} PageMeas;                     /* seq & 1 == 0 */

/* Timing in 10 us units (TIM16 count, t = 0 at the frame tick). Worst values since boot
 * or CID 10. Counters wrap modulo 2^8 or 2^16: the ESP32 counts the increments. */
typedef struct __attribute__((packed)) {
    uint16_t work_end_last;     /* t when the UART TX starts */
    uint16_t work_end_worst;
    uint16_t l2_wait_min;       /* lane 2 wait for DMA segments, minimum */
    uint8_t  isr_worst;         /* frame-tick ISR */
    uint8_t  overrun_count;     /* frames where the next tick came before UART TX */
    uint8_t  hk_skip_count;     /* frames where housekeeping was skipped */
    uint8_t  spare;
    uint16_t cmd_ok;            /* commands executed or answered as duplicate */
    uint8_t  cmd_crc_err;       /* 10 B windows with a bad CRC */
    uint8_t  uart_err;          /* USART RX errors (framing, noise) */
    uint8_t  rx_restart;        /* RX DMA restarts */
    uint8_t  bad_length;        /* non-empty windows that were not 10 B */
} PageHealth;                   /* seq & 1 == 1; cmd_duplicate is read by CID 32 */

WFRX_STATIC_ASSERT(sizeof(PageMeas) == 16, "PageMeas size");
WFRX_STATIC_ASSERT(sizeof(PageHealth) == 16, "PageHealth size");

typedef union {
    uint8_t    raw[16];
    PageMeas   meas;
    PageHealth health;
} StmPage;

/* ===========================================================================
 * STM32 -> ESP32 frame: 128 B, every frame tick.
 * ========================================================================= */

#define STM_FRAME_LEN               128u

typedef enum {
    RSP_NONE         = 0,
    RSP_OK           = 1,
    RSP_OK_CLAMPED   = 2,
    RSP_UNKNOWN_CID  = 3,
    RSP_READ_ONLY    = 4,
    RSP_WRITE_ONLY   = 5,
    RSP_BAD_VALUE    = 6,
} RspStatus;

typedef struct __attribute__((packed)) {
    uint8_t      proto_ver;     /*   0  WFRX_PROTO_VER */
    uint8_t      seq;           /*   1  +1 per frame; page = seq & 1 */
    uint8_t      resp_seq;      /*   2 */
    uint8_t      resp_cid;      /*   3  incl. write bit; 0 = no response */
    uint32_t     status_bits;   /*   4  WFRX_STATUS_* */
    union {
        int32_t  int32;
        float    flt32;
    } resp_value;               /*   8 */
    uint8_t      resp_status;   /*  12  RspStatus */
    uint8_t      resp_index;    /*  13 */
    WfrxCoilPair coil[WFRX_RESULTS][WFRX_BINS];  /* 14 */
    StmPage      page;          /* 110 */
    uint16_t     crc;           /* 126 */
} StmFrameV2;

WFRX_STATIC_ASSERT(sizeof(WfrxCoilPair) == 6, "WfrxCoilPair size");
WFRX_STATIC_ASSERT(sizeof(StmFrameV2) == STM_FRAME_LEN, "StmFrameV2 size");
WFRX_STATIC_ASSERT(offsetof(StmFrameV2, status_bits) == 4,   "offset status_bits");
WFRX_STATIC_ASSERT(offsetof(StmFrameV2, resp_value)  == 8,   "offset resp_value");
WFRX_STATIC_ASSERT(offsetof(StmFrameV2, resp_status) == 12,  "offset resp_status");
WFRX_STATIC_ASSERT(offsetof(StmFrameV2, coil)        == 14,  "offset coil");
WFRX_STATIC_ASSERT(offsetof(StmFrameV2, page)        == 110, "offset page");
WFRX_STATIC_ASSERT(offsetof(StmFrameV2, crc)         == 126, "offset crc");

/* Status bits:
 *   bit 0          retired (always 0)
 *   bits 1..3      bit 1 command in progress (unused); bits 2,3 lane 1 PGA step
 *   bits 4,5       retired (signal weak/lost: the ESP32 derives them)
 *   bit 6          lane 1 overload (ADC above range at the lowest gain, or at any gain
 *                  fixed by CID 29)
 *   bits 7,8       lane 2 PGA step
 *   bits 9,10      retired
 *   bit 11         lane 2 overload
 *   bits 12..16    as v1: VDD fault, CP enable, CP cooldown, ESP VDD fault, 5 V fault
 *   bits 17,18     retired
 *   bit 19         frame overrun (first frame after it)
 *   bit 20         command link error (first frame after a UART error or a discarded window)
 *   bit 21         STM32 reset: set until the first executed write command
 *   bits 22,23     lane inputs: bit 22 lane 1, bit 23 lane 2 (0 = sensor 1, 1 = sensor 2)
 *   bit 24         acquisition running (CID 1); 0 = idle mode
 */
#define WFRX_STATUS_FRAME_OVERRUN       (1u << 19)
#define WFRX_STATUS_CMD_LINK_ERR        (1u << 20)
#define WFRX_STATUS_STM_RESET           (1u << 21)
#define WFRX_STATUS_LANE1_INPUT         (1u << 22)
#define WFRX_STATUS_LANE2_INPUT         (1u << 23)
#define WFRX_STATUS_RUN                 (1u << 24)

/* ===========================================================================
 * ESP32 -> STM32 command frame: 10 B, framed by the frame tick.
 * Window of 0 B: no command. Exactly 10 B and a good CRC: executed, answered in the next
 * frame (resp_seq, resp_cid, resp_index, resp_status, resp_value). Any other window:
 * discarded and counted. Retry after 3 frames with the same seq. The same seq, cid and
 * index within 10 frames of the last executed command: answered again, not executed.
 * ========================================================================= */

#define ESP_FRAME_LEN               10u
#define ESP_CID_WRITE               0x80u
#define WFRX_DUPLICATE_WINDOW       10u

typedef struct __attribute__((packed)) {
    uint8_t  seq;               /* 0 */
    uint8_t  cid;               /* 1  bits 0..6 CID, bit 7 write */
    uint8_t  index;             /* 2  bin, sensor or item: per CID */
    uint8_t  reserved;          /* 3  0 */
    union {
        int32_t  int32;
        float    flt32;
    } value;                    /* 4 */
    uint16_t crc;               /* 8 */
} EspFrameV2;

WFRX_STATIC_ASSERT(sizeof(EspFrameV2) == ESP_FRAME_LEN, "EspFrameV2 size");
WFRX_STATIC_ASSERT(offsetof(EspFrameV2, value) == 4, "offset value");
WFRX_STATIC_ASSERT(offsetof(EspFrameV2, crc)   == 8, "offset crc");

/* CIDs. resp_value returns the value in effect. Defaults apply after each STM32 reset.
 *   1  acquisition              R/W int32 1 = run, 0 = idle. Default 0: the STM32 boots to idle
 *                                   mode. Idle: no sensor acquisition and no DSP, coil data
 *                                   zero; the frame tick, frames, commands, pages, slow ADC,
 *                                   charge-port protection and fault checks keep running. The
 *                                   ESP32 configures, then writes 1. The change takes effect at
 *                                   the next tick; a start resets the per-bin tracking state and
 *                                   the PGA steps. Status bit 24 shows the mode in effect.
 *   3  save NVM                 W   factory calibration incl. CID 27/28 values; all work
 *                                   pauses during the flash write, frames resync after it
 *   6  charge stop voltage      R/W float V, clamped
 *   7  charge overcurrent       R/W float A, clamped
 *   8  charge time limit        R/W int32 s, clamped
 *   9  charge enable request    R/W int32 0/1
 *  10  timing reset             W   worst values and counters
 *  13  lane inputs              R/W int32 bit 0 lane 1, bit 1 lane 2; default 0x02
 *  18  frame period             R/W int32 us, 8000..655350, step 10; default 10000;
 *                                   starts at the next tick
 *  19  primary bin              R/W index = sensor; int32 bin; default 0 (LED, internal)
 *  20  bin frequency            R/W index = bin; float Hz; 0 = off; outside 42.5..120 kHz
 *                                   = off (RSP_BAD_VALUE). Takes effect in the next frame.
 *                                   Default: Tx CH 1..5, 62.5 kHz, 46.875 kHz, off
 *  21  weak threshold           R/W index = bin; float magnitude; default 1/600
 *  22  lost threshold           R/W index = bin; float magnitude; default 1/1200
 *  23  regained threshold       R/W index = bin; float magnitude; default 1/1000
 *  24  regain wait              R/W int32 ms; default 500
 *  25  coil data mode mask      R/W int32 bit b = bin b: 0 magnitude, 1 xy; default 0
 *  26  bin gain (xy mode)       R/W index = sensor * 8 + bin; float; default from NVM
 *  27  radius calibration       R/W index = sensor; float; default from NVM
 *  28  angle calibration        R/W index = sensor; float rad; default from NVM
 *  30  versions                 R   int32 bytes: app major, minor, patch, proto_ver
 *  31  reset cause              R   int32 RCC CSR at app start
 *  32  frame count              R   index 0: frames since boot; index 1: cmd_duplicate
 *  33  timing detail            R   index = item; int32 = last (bits 0..15) | worst
 *                                   (bits 16..31), 10 us units unless stated. Items (app
 *                                   2.0.10, kernel during acquisition): 0 lane 1 done (t
 *                                   from the tick), 1 lane 2 set-up, 2 lane 1 post-
 *                                   processing, 3 lane 2 post-processing + PGA, 4
 *                                   housekeeping, 5 TX preparation, 6 lane 2 done (t),
 *                                   7 NVM pause (0.1 ms), 8 cross-check: TIM15 work end
 *                                   in us | fail count, 9 DMA ch 2/3 interrupt (ADC,
 *                                   USART1 TX) in us, 10 kernel segment call in us
 *  34  bins on at boot          R   int32 bits 8..15 (bits 0..7 = 0: no boot self-test)
 *  29  PGA override             R/W index = sensor; int32 -1 auto (default), 0..2 fixed step
 *                                   (debug and test)
 *  35  fault record             R/W index 0..4, of the last reset (kept over a system reset):
 *                                   0 = 0xFA17 << 16 | type << 8 | main stage (type 1
 *                                   HardFault, 2 Error_Handler, 3 main-loop stall; 0 = none),
 *                                   1 = PC, 2 = LR, 3 = frame count, 4 = faults since
 *                                   power-on. Write (any index): clear.
 *  37  PGA calibration          R/W index = sensor * 6 + coil * 3 + PGA step (coil 0 = coil 1);
 *                                   float; the factory value from NVM; CID 3 saves it.
 *                                   sN = |GN| x this value, so equal magnitudes across the PGA
 *                                   steps need these three values per coil matched.
 *  36  debug fault              W   int32 1 HardFault, 2 main-loop stall, 3 Error_Handler.
 *                                   The unit resets; the ESP32 sends BOOT after 1 s silence.
 * Unknown CID: RSP_UNKNOWN_CID. Index out of range or value not valid: RSP_BAD_VALUE.
 */

#endif /* RCTF_WFRX_COMMS_V2_H */
