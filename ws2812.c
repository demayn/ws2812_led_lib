#include "ws2812.h"
#define TAG "WS2812"
// static const char *WS2812TAG = "WS2812";
void new_ws2812(const ws2812_config *cfg, WS2812 *ws2812)
{
    // handle ws2812 config
    ws2812->data_pin = cfg->data_pin;
    ws2812->num_leds = cfg->num_leds;

    // set color array
    uint8_t tmp[8][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}, {1, 1, 0}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};
    memcpy(ws2812->color_arrays, tmp, 3 * 8);

    // rmt settings
    ws2812->tx_config.loop_count = 0;

    rmt_tx_channel_config_t ws2812_rmt_cfg =
        {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = ws2812->data_pin,
            .mem_block_symbols = 64,
            .resolution_hz = 10000000,
            .trans_queue_depth = 4,
        };

    rmt_bytes_encoder_config_t ws2812_enc_cfg =
        {
            .bit0.level0 = 1,
            .bit0.duration0 = 0.3 * ws2812_rmt_cfg.resolution_hz / 1000000, // T0H=0.3us
            .bit0.level1 = 0,
            .bit0.duration1 = 0.9 * ws2812_rmt_cfg.resolution_hz / 1000000, // T0L=0.9us
            .bit1.level0 = 1,
            .bit1.duration0 = 0.9 * ws2812_rmt_cfg.resolution_hz / 1000000, // T1H=0.9us
            .bit1.level1 = 0,
            .bit1.duration1 = 0.3 * ws2812_rmt_cfg.resolution_hz / 1000000, // T1L=0.3us
            .flags.msb_first = 1,                                           // WS2812 transfer bit order: G7...G0R7...R0B7...B0
        };

    // enable rmt tx hw
    ESP_ERROR_CHECK(rmt_new_tx_channel(&ws2812_rmt_cfg, &ws2812->led_chan));
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&ws2812_enc_cfg, &ws2812->bytes_encoder));
    ESP_ERROR_CHECK(rmt_enable(ws2812->led_chan));

    // malloc led data memory
    ws2812->led_data = (uint8_t(*)[3])malloc(ws2812->num_leds * sizeof(uint8_t[3]));
    memset(ws2812->led_data, 0, 3 * ws2812->num_leds);
    ESP_LOGI(TAG, "WS2812 initialized");
    return;
}

void ws2812_setLEDarr(WS2812 *ws2812, int16_t idx, uint8_t *color_data)
{
    if (idx != -1)
    {
        for (int color = 0; color < 3; color++)
        {
            ws2812->led_data[idx][color] = color_data[color];
        }
    }
    else
    {
        int k = 0;
        for (int idx = 0; idx < ws2812->num_leds; idx++)
        {
            for (int color = 0; color < 3; color++)
            {
                ws2812->led_data[idx][color] = color_data[k];
                k++;
            }
        }
    }
}

/// @brief set LED color data via three values for a specified LED or all (all to the same value)
/// @param idx led index (0...num_leds-1 or -1 for all)
/// @param red red brightness value
/// @param green green brightness value
/// @param blue blue brightness value
void ws2812_setLEDvals(WS2812 *ws2812, int16_t idx, uint8_t red, uint8_t green, uint8_t blue)
{
    if (idx != -1)
    {
        ws2812->led_data[idx][0] = green;
        ws2812->led_data[idx][1] = red;
        ws2812->led_data[idx][2] = blue;
    }

    else
    {
        for (int idx = 0; idx < ws2812->num_leds; idx++)
        {
            ws2812->led_data[idx][0] = green;
            ws2812->led_data[idx][1] = red;
            ws2812->led_data[idx][2] = blue;
        }
    }
}

/// @brief set LED color data via three values for a specified LED or all (all to the same value)
/// @param idx led index (0...num_leds-1 or -1 for all)
/// @param  color prespecified color (see enum color)
/// @param brightness brightness value (0...255)
void ws2812_setLEDcol(WS2812 *ws2812, int16_t idx, ws2812_color color_name, uint8_t brightness)
{
    if (idx != -1)
    {
        for (int color_idx = 0; color_idx < 3; color_idx++)
        {
            ws2812->led_data[idx][color_idx] = ws2812->color_arrays[color_name][color_idx] * brightness;
        }
    }
    else
    {
        for (int idx = 0; idx < ws2812->num_leds; idx++)
        {
            for (int color_idx = 0; color_idx < 3; color_idx++)
            {
                ws2812->led_data[idx][color_idx] = ws2812->color_arrays[color_name][color_idx] * brightness;
            }
        }
    }
}

void ws2812_readLED(WS2812 *ws2812, int16_t idx, uint8_t *color_data)
{
    for (int i = 0; i < 3; i++)
    {
        color_data[i] = ws2812->led_data[idx][i];
    }
}

/// @brief triggers transmission and thus display of the prior set LED data
void ws2812_writeLEDs(WS2812 *ws2812)
{
    ESP_LOGD(TAG, "rmt transmit");
    ESP_ERROR_CHECK(rmt_transmit(ws2812->led_chan, ws2812->bytes_encoder, ws2812->led_data, ws2812->num_leds * sizeof(uint8_t[3]), &(ws2812->tx_config)));
    // ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, 100));
}

/// @brief frees LED data space, disables driver
void ws2812_end(WS2812 *ws2812)
{
    free(ws2812->led_data);
    rmt_disable(ws2812->led_chan);
}

void ws2812_task(void *arg)
{
    ws2812_task_data *task_data = (ws2812_task_data *)arg;
    led_evt_t evt;
}