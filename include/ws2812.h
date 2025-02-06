#ifndef WS2812
#define WS2812

#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include <string.h>

/// @brief human readable Color Shortcuts for WS2812
typedef enum
{
    green,
    red,
    blue,
    black,
    yellow,
    turquoise,
    purple,
    white
} ws2812_color;

typedef struct
{
    gpio_num_t data_pin;
    int16_t num_leds;

} ws2812_config;

typedef struct WS2812 WS2812;

struct WS2812
{
    rmt_channel_handle_t led_chan;
    rmt_encoder_handle_t bytes_encoder;

    rmt_transmit_config_t tx_config;
    // rmt_tx_channel_config_t ws2812_tx_chan_config;
    // rmt_bytes_encoder_config_t bytes_encoder_config;

    gpio_num_t data_pin;
    int16_t num_leds;
    uint8_t (*led_data)[3]; // Green Red Blue

    uint8_t color_arrays[8][3];

    void (*setLEDarr)(WS2812 *ws2812, int16_t idx, uint8_t *color_data);
    void (*setLEDvals)(WS2812 *ws2812, int16_t idx, uint8_t red, uint8_t green, uint8_t blue);
    void (*setLEDcol)(WS2812 *ws2812, int16_t idx, ws2812_color color_name, uint8_t brightness);
    void (*readLED)(WS2812 *ws2812, int16_t idx, uint8_t *color_data);
    void (*writeLEDs)(WS2812 *ws2812);
    void (*end)(WS2812 *ws2812);
};

void new_ws2812(const ws2812_config *cfg, WS2812 *ws2812);
void ws2812_setLEDarr(WS2812 *ws2812, int16_t idx, uint8_t *color_data);
void ws2812_setLEDvals(WS2812 *ws2812, int16_t idx, uint8_t red, uint8_t green, uint8_t blue);
void ws2812_setLEDcol(WS2812 *ws2812, int16_t idx, ws2812_color color_name, uint8_t brightness);
void ws2812_readLED(WS2812 *ws2812, int16_t idx, uint8_t *color_data);
void ws2812_writeLEDs(WS2812 *ws2812);
void ws2812_end(WS2812 *ws2812);

#endif