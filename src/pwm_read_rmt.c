#include "pwm_read_rmt.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"
#include <string.h>

int32_t *pwm_read_dur = NULL;
uint8_t pwm_in_num_channels = 0;
static volatile uint32_t pwm_last_valid_us[SOC_RMT_CHANNELS_PER_GROUP] = {};
static volatile pwm_stats_t pwm_st[SOC_RMT_CHANNELS_PER_GROUP];  // 0.5.104: bench "pwm"
static volatile uint32_t pwm_st_last_us[SOC_RMT_CHANNELS_PER_GROUP];

static void rmt_isr_handler(void* arg)
{
    uint32_t intr_st = RMT.int_st.val;

    for(int i = 0; i < pwm_in_num_channels; i++)
    {
        uint8_t channel = i;
        uint32_t channel_mask = BIT(channel*3+1);

        if (!(intr_st & channel_mask)) continue;

        RMT.conf_ch[channel].conf1.rx_en = 0;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
        volatile rmt_item32_t* item = RMTMEM.chan[channel].data32;
        if (item) {
            // Capture the HIGH pulse even when the first captured phase is LOW.
            uint32_t ticks = item->level0 ? item->duration0 :
                             item->level1 ? item->duration1 : 0;
            pwm_read_dur[i] = ticks;
            if (ticks >= 9000 && ticks <= 21000) {
                const uint32_t now = (uint32_t)esp_timer_get_time();
                pwm_last_valid_us[i] = now;
                volatile pwm_stats_t* st = &pwm_st[i];  // 0.5.104: pulse statistics
                if (st->n) {
                    const uint32_t p = now - pwm_st_last_us[i];
                    if (st->n == 1 || p < st->p_min) st->p_min = p;
                    if (p > st->p_max) st->p_max = p;
                    st->p_sum += p;
                }
                if (!st->n || ticks < st->w_min) st->w_min = ticks;
                if (ticks > st->w_max) st->w_max = ticks;
                st->w_sum += ticks;
                st->n++;
                pwm_st_last_us[i] = now;
            }
        }

        RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[channel].conf1.rx_en = 1;

        //clear RMT interrupt status.
        RMT.int_clr.val = channel_mask;
    }
}

void pwm_read_rmt_init(uint8_t channelPins[], uint8_t numberOfPins)
{
    assert(numberOfPins <= SOC_RMT_CHANNELS_PER_GROUP);
    // Each channel is int32_t; allocating uint16_t storage corrupts the heap.
    pwm_read_dur = calloc(numberOfPins, sizeof(*pwm_read_dur));
    if (pwm_read_dur == NULL) {
        pwm_in_num_channels = 0;
        return;
    }
    pwm_in_num_channels = numberOfPins;
    pwm_reset_readings();  // Initialize before enabling the receive interrupts.

    for(int i = 0; i < pwm_in_num_channels; i++)
    {
        rmt_config_t rmt_configuration = RMT_DEFAULT_CONFIG_RX(channelPins[i], i);
        rmt_config(&rmt_configuration);
        rmt_set_clk_div(i, 8);
        rmt_set_rx_idle_thresh(i, 25000);
        rmt_set_rx_intr_en(i, true);
        rmt_rx_start(i, 1);
    }

    rmt_isr_register(rmt_isr_handler, NULL, 0, NULL);
}

int32_t pwm_read_rmt_dur(uint8_t channel)
{   
    if (pwm_read_dur == NULL || channel >= pwm_in_num_channels) return 0;
    return (pwm_read_dur[channel] - PWM_INPUT_OFFSET) / 10;
}

bool pwm_read_rmt_valid(uint8_t channel, uint32_t max_age_ms) {
    if (channel >= pwm_in_num_channels || pwm_read_dur == NULL) return false;
    uint32_t timestamp = pwm_last_valid_us[channel];
    int32_t pulse = pwm_read_rmt_dur(channel);
    return timestamp != 0 && pulse >= 900 && pulse <= 2100 &&
           (uint32_t)((uint32_t)esp_timer_get_time() - timestamp) < max_age_ms * 1000;
}

void pwm_read_rmt_stats(uint8_t channel, pwm_stats_t* out, bool reset) {
    if (channel >= SOC_RMT_CHANNELS_PER_GROUP) return;
    rmt_set_rx_intr_en(channel, false);
    *out = *(pwm_stats_t*)&pwm_st[channel];
    if (reset) memset((void*)&pwm_st[channel], 0, sizeof(pwm_stats_t));
    rmt_set_rx_intr_en(channel, true);
}

void pwm_reset_readings(){
    for(int i = 0; i < pwm_in_num_channels; i++)
    {
        pwm_read_dur[i] = PWM_INPUT_OFFSET;
        pwm_last_valid_us[i] = 0;
    }
}