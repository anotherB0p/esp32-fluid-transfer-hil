#include "status_led.h"

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"

#define LED_GPIO 21

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;
static uint32_t s_last_rgb = 0xFFFFFFFFu;

void status_led_init(void)
{
    const rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, /* 0.1 us per tick */
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_chan));
    const rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = {.level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9},
        .bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3},
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_enc));
    ESP_ERROR_CHECK(rmt_enable(s_chan));
}

static void set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    const uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    if (rgb == s_last_rgb) {
        return;
    }
    s_last_rgb = rgb;
    static uint8_t grb[3];
    grb[0] = g;
    grb[1] = r;
    grb[2] = b;
    const rmt_transmit_config_t tcfg = {.loop_count = 0};
    rmt_transmit(s_chan, s_enc, grb, sizeof(grb), &tcfg); /* async, ~30 us */
}

void status_led_update(ctl_state_t state, bool link_ok, uint32_t now_ms)
{
    const bool blink_on = (now_ms / 250u) % 2u == 0u;
    switch (state) {
    case CTL_FAULT:
        blink_on ? set_rgb(40, 0, 0) : set_rgb(0, 0, 0);
        return;
    case CTL_CHECK:
        set_rgb(30, 20, 0);
        return;
    case CTL_TRANSFER:
        set_rgb(0, 0, 40);
        return;
    case CTL_COMPLETE:
        set_rgb(0, 35, 0);
        return;
    case CTL_IDLE:
    default:
        if (!link_ok) {
            blink_on ? set_rgb(30, 0, 30) : set_rgb(0, 0, 0);
        } else {
            set_rgb(12, 12, 12);
        }
        return;
    }
}
