
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

#ifdef __cplusplus
}
#endif