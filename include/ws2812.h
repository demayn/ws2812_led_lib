#ifndef WS2812_H
#define WS2812_H

#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

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
    QueueHandle_t led_evt_queue; // set NULL iuf not used;
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

    QueueHandle_t led_evt_queue; // set NULL iuf not used;

    uint8_t color_arrays[8][3];
};

typedef enum
{
    WS2812_OFF,
    WS2812_BLINK_ONCE,         // duration = on-time and off-time
    WS2812_BLINK_TWICE,        // duration = on-time and off-time
    WS2812_BLINK_INDEFINITELY, // duration = on-time and off-time
    WS2812_ON,                 // duration = dont care
    WS2812_BREATHE,            // duration = period of one cycle
} ws2812_blink_mode;

typedef struct
{
    int16_t idx;
    ws2812_color color;
    uint8_t brightness;
    uint32_t duration; // in ms
    ws2812_blink_mode mode;
} led_evt_t;

typedef struct
{
    WS2812 *ws2812;
    QueueHandle_t queue;
} ws2812_task_data;

#define BREATH_TASK_INTERVAL 5 // ticks
typedef struct
{
    WS2812 *ws2812;
    SemaphoreHandle_t ws2812_mutex;
    led_evt_t event_data;
} blink_task_data;


void new_ws2812(const ws2812_config *cfg, WS2812 *ws2812);
void ws2812_setLEDarr(WS2812 *ws2812, int16_t idx, uint8_t *color_data);
void ws2812_setLEDvals(WS2812 *ws2812, int16_t idx, uint8_t red, uint8_t green, uint8_t blue);
void ws2812_setLEDcol(WS2812 *ws2812, int16_t idx, ws2812_color color_name, uint8_t brightness);
void ws2812_readLED(WS2812 *ws2812, int16_t idx, uint8_t *color_data);
void ws2812_writeLEDs(WS2812 *ws2812);
void ws2812_end(WS2812 *ws2812);
void breathing_task(void *breathing_data_v);
void blinking_task(void *blinking_data_v);
void ws2812_task(void *arg);

#endif