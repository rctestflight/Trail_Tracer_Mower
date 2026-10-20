
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "driver/rmt.h"
#include <stdbool.h>

#define PWM_INPUT_OFFSET 0

void pwm_read_rmt_init(uint8_t channelPins[], uint8_t numberOfPins);

int32_t pwm_read_rmt_dur(uint8_t channel);
void pwm_reset_readings();
bool pwm_read_rmt_valid(uint8_t channel, uint32_t max_age_ms);
// 0.5.104 (bench "pwm"): pulses since the last stats reset: count, width (0.1 us ticks) min, max, sum, and the
// interval between pulses (us) min, max, sum over n - 1 intervals. Reset clears them.
typedef struct { uint32_t n, w_min, w_max, w_sum, p_min, p_max, p_sum; } pwm_stats_t;
void pwm_read_rmt_stats(uint8_t channel, pwm_stats_t* out, bool reset);

#ifdef __cplusplus
}
#endif