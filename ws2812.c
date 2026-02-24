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
            .mem_block_symbols = 128,
            .resolution_hz = 10000000,
            .trans_queue_depth = 16,
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
    ws2812->led_data = (uint8_t (*)[3])malloc(ws2812->num_leds * sizeof(uint8_t[3]));
    memset(ws2812->led_data, 0, 3 * ws2812->num_leds);

    if (cfg->led_evt_queue != NULL)
    {
        // create task data
        ws2812_task_data *task_data = malloc(sizeof(ws2812_task_data));
        task_data->ws2812 = ws2812;
        task_data->queue = cfg->led_evt_queue;

        // create ws2812 task
        xTaskCreate(ws2812_task, "WS2812_TASK", 4096, (void *)task_data, tskIDLE_PRIORITY, NULL);
    }
    else
        ws2812->led_evt_queue = NULL;

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
    ESP_LOGI(TAG, "WS2812 task started");
    ws2812_task_data task_data = *(ws2812_task_data *)arg;
    led_evt_t evt;
    TaskHandle_t *led_task_handles = malloc(sizeof(TaskHandle_t) * task_data.ws2812->num_leds);
    void **task_datasets = malloc(sizeof(void *) * task_data.ws2812->num_leds); // array to hold task data pointers
    led_evt_t *old_states = malloc(sizeof(led_evt_t) * task_data.ws2812->num_leds); // to memorize old states for blinking tasks
    if (led_task_handles == NULL || task_datasets == NULL || old_states == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for task handles or datasets");
        vTaskDelete(NULL);
        return;
    }
    memset(led_task_handles, 0, sizeof(TaskHandle_t) * task_data.ws2812->num_leds); // set all taskhandles to NULL
    memset(task_datasets, 0, sizeof(void *) * task_data.ws2812->num_leds);
    memset(old_states, 0, sizeof(led_evt_t) * task_data.ws2812->num_leds);
    SemaphoreHandle_t ws2812_mutex = xSemaphoreCreateMutex();

    // wait for leds to get ready
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (1)
    {
        if (xQueueReceive(task_data.queue, &evt, portMAX_DELAY) == pdTRUE)
        {
            if(evt.mode == old_states[evt.idx].mode && evt.color == old_states[evt.idx].color && evt.brightness == old_states[evt.idx].brightness) // if new event is the same as old event for this led, ignore it
                continue;
            else if(evt.mode != WS2812_BLINK_ONCE && evt.mode != WS2812_BLINK_TWICE) // exclude single events from being memorized as old state, otherwise blinking tasks would not work properly
            {
                old_states[evt.idx] = evt; // memorize new event as old event
            }
                
            switch (evt.mode)
            {
            case WS2812_ON:
            {
                if (led_task_handles[evt.idx] != NULL) // check if task already running
                {
                    vTaskDelete(led_task_handles[evt.idx]); // delete old task
                    led_task_handles[evt.idx] = NULL;
                    vTaskDelay(1); // small delay othzer wise command to led is lost sometimes
                }
                if (task_datasets[evt.idx] != NULL)
                {
                    free(task_datasets[evt.idx]); // free old task data
                    task_datasets[evt.idx] = NULL;
                }
                xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                ws2812_setLEDcol(task_data.ws2812, evt.idx, evt.color, evt.brightness);
                ws2812_writeLEDs(task_data.ws2812);
                xSemaphoreGive(ws2812_mutex);
                break;
            }
            case WS2812_BREATHE:
            {
                if (led_task_handles[evt.idx] != NULL) // check if task already running
                {
                    vTaskDelete(led_task_handles[evt.idx]); // delete old task
                    led_task_handles[evt.idx] = NULL;
                    vTaskDelay(1); // small delay othzer wise command to led is lost sometimes
                }
                if (task_datasets[evt.idx] != NULL)
                {
                    free(task_datasets[evt.idx]); // free old task data
                    task_datasets[evt.idx] = NULL;
                }

                blink_task_data *breath_data = malloc(sizeof(blink_task_data));
                if (breath_data == NULL) // Allocation failed; cannot start breathing task
                    break;

                breath_data->ws2812 = task_data.ws2812;
                breath_data->ws2812_mutex = ws2812_mutex;
                breath_data->event_data = evt;
                task_datasets[evt.idx] = (void *)breath_data; // memorize task data pointer for free later
                xTaskCreate(breathing_task, "BREATH_TASK", 1024, (void *)breath_data, tskIDLE_PRIORITY + 1, &led_task_handles[evt.idx]);

                break;
            }
            case WS2812_BLINK_ONCE:
            {
                if (led_task_handles[evt.idx] != NULL) // check if task already running
                {
                    vTaskSuspend(led_task_handles[evt.idx]); // stop task
                    vTaskDelay(1);                           // small delay otherwise command to led is lost sometimes}
                }
                xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                uint8_t old_brightness_vals[3];
                ws2812_readLED(task_data.ws2812, evt.idx, old_brightness_vals);

                ws2812_setLEDcol(task_data.ws2812, evt.idx, evt.color, evt.brightness);
                ws2812_writeLEDs(task_data.ws2812);
                xSemaphoreGive(ws2812_mutex);
                vTaskDelay(evt.duration / portTICK_PERIOD_MS);

                xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                ws2812_setLEDcol(task_data.ws2812, evt.idx, black, 0);
                ws2812_writeLEDs(task_data.ws2812);
                xSemaphoreGive(ws2812_mutex);
                vTaskDelay(evt.duration / portTICK_PERIOD_MS);

                // restore old color
                xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                ws2812_setLEDarr(task_data.ws2812, evt.idx, old_brightness_vals);
                ws2812_writeLEDs(task_data.ws2812);
                xSemaphoreGive(ws2812_mutex);

                if (led_task_handles[evt.idx] != NULL)
                    vTaskResume(led_task_handles[evt.idx]); // resume task

                break;
            }
            case WS2812_BLINK_TWICE:
            {

                if (led_task_handles[evt.idx] != NULL) // check if task already running
                {
                    vTaskSuspend(led_task_handles[evt.idx]); // stop task
                    vTaskDelay(1);                           // small delay othzer wise command to led is lost sometimes
                }
                xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                uint8_t old_brightness_vals[3];
                ws2812_readLED(task_data.ws2812, evt.idx, old_brightness_vals);
                xSemaphoreGive(ws2812_mutex);
                for (int i = 0; i < 2; i++)
                {
                    xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                    ws2812_setLEDcol(task_data.ws2812, evt.idx, evt.color, evt.brightness);
                    ws2812_writeLEDs(task_data.ws2812);
                    xSemaphoreGive(ws2812_mutex);
                    vTaskDelay(evt.duration / portTICK_PERIOD_MS);

                    xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                    ws2812_setLEDcol(task_data.ws2812, evt.idx, black, 0);
                    ws2812_writeLEDs(task_data.ws2812);
                    xSemaphoreGive(ws2812_mutex);
                    vTaskDelay(evt.duration / portTICK_PERIOD_MS);
                }

                // restore old color
                xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                ws2812_setLEDarr(task_data.ws2812, evt.idx, old_brightness_vals);
                ws2812_writeLEDs(task_data.ws2812);
                xSemaphoreGive(ws2812_mutex);

                if (led_task_handles[evt.idx] != NULL)
                    vTaskResume(led_task_handles[evt.idx]); // resume task
                break;
            }
            case WS2812_BLINK_INDEFINITELY:
            {
                if (led_task_handles[evt.idx] != NULL) // check if task already running
                {
                    vTaskDelete(led_task_handles[evt.idx]); // delete old task
                    led_task_handles[evt.idx] = NULL;
                    vTaskDelay(1); // small delay othzer wise command to led is lost sometimes
                }
                if (task_datasets[evt.idx] != NULL)
                {
                    free(task_datasets[evt.idx]); // free old task data
                    task_datasets[evt.idx] = NULL;
                }

                blink_task_data *blink_data = malloc(sizeof(blink_task_data));
                if (blink_data == NULL) // Allocation failed; cannot start blinking task
                    break;

                blink_data->ws2812 = task_data.ws2812;
                blink_data->ws2812_mutex = ws2812_mutex;
                blink_data->event_data = evt;
                task_datasets[evt.idx] = (void *)blink_data; // memorize task data

                xTaskCreate(blinking_task, "BLINK_TASK", 1024, (void *)blink_data, tskIDLE_PRIORITY + 1, &led_task_handles[evt.idx]);
                break;
            }
            case WS2812_OFF:
            {
                if (led_task_handles[evt.idx] != NULL) // check if task already running
                {
                    vTaskDelete(led_task_handles[evt.idx]); // delete old task
                    led_task_handles[evt.idx] = NULL;
                    vTaskDelay(1); // small delay othzer wise command to led is lost sometimes
                }
                if (task_datasets[evt.idx] != NULL)
                {
                    free(task_datasets[evt.idx]); // free old task data
                    task_datasets[evt.idx] = NULL;
                }

                xSemaphoreTake(ws2812_mutex, portMAX_DELAY);
                ws2812_setLEDcol(task_data.ws2812, evt.idx, black, 0);
                ws2812_writeLEDs(task_data.ws2812);
                xSemaphoreGive(ws2812_mutex);
                break;
            }
            default:
                break;
            }
        }
    }
}

void blinking_task(void *blinking_data_v)
{
    blink_task_data *task_data = (blink_task_data *)blinking_data_v;

    xSemaphoreTake(task_data->ws2812_mutex, portMAX_DELAY);
    ws2812_setLEDcol(task_data->ws2812, task_data->event_data.idx, task_data->event_data.color, task_data->event_data.brightness);
    ws2812_writeLEDs(task_data->ws2812);
    xSemaphoreGive(task_data->ws2812_mutex);

    while (1)
    {
        xSemaphoreTake(task_data->ws2812_mutex, portMAX_DELAY);
        ws2812_setLEDcol(task_data->ws2812, task_data->event_data.idx, task_data->event_data.color, task_data->event_data.brightness);
        ws2812_writeLEDs(task_data->ws2812);
        xSemaphoreGive(task_data->ws2812_mutex);
        vTaskDelay(task_data->event_data.duration / portTICK_PERIOD_MS);

        xSemaphoreTake(task_data->ws2812_mutex, portMAX_DELAY);
        ws2812_setLEDcol(task_data->ws2812, task_data->event_data.idx, black, 0);
        ws2812_writeLEDs(task_data->ws2812);
        xSemaphoreGive(task_data->ws2812_mutex);
        vTaskDelay(task_data->event_data.duration / portTICK_PERIOD_MS);
    }
}

void breathing_task(void *breathing_data_v)
{
    blink_task_data *task_data = (blink_task_data *)breathing_data_v;
    uint8_t current_brightness_vals[3];
    uint8_t breathing_increments[3];
    int8_t up_down_switch = +1;

    // set and read start color (so i dont have to track which leds are involved in breathing)
    xSemaphoreTake(task_data->ws2812_mutex, portMAX_DELAY);
    ws2812_setLEDcol(task_data->ws2812, task_data->event_data.idx, task_data->event_data.color, task_data->event_data.brightness);
    ws2812_writeLEDs(task_data->ws2812);
    ws2812_readLED(task_data->ws2812, task_data->event_data.idx, current_brightness_vals);
    xSemaphoreGive(task_data->ws2812_mutex);

    {                                                                                                       // {} codeblock to limit breathing increment calculation scope
        uint8_t steps = (task_data->event_data.duration / portTICK_PERIOD_MS) / (2 * BREATH_TASK_INTERVAL); // 2*Interval due to up and down cycle
        int16_t incr = task_data->event_data.brightness / steps;

        for (int i = 0; i < 3; i++)
        {
            if (current_brightness_vals[i] > 0) // this means the color is active, thus is breathing increment must be applied
                breathing_increments[i] = incr;
            else
                breathing_increments[i] = 0;
        }
    }

    while (1)
    {
        xSemaphoreTake(task_data->ws2812_mutex, portMAX_DELAY);
        ws2812_readLED(task_data->ws2812, task_data->event_data.idx, current_brightness_vals);
        for (int color_idx = 0; color_idx < 3; color_idx++)
        {
            if (current_brightness_vals[color_idx] > (task_data->event_data.brightness - breathing_increments[color_idx])) // no increasing brightness possible
            {
                up_down_switch = -1;
            }
            else if (current_brightness_vals[color_idx] < breathing_increments[color_idx])
            {
                up_down_switch = +1;
            }
            current_brightness_vals[color_idx] += (up_down_switch * breathing_increments[color_idx]);
        }
        ws2812_setLEDarr(task_data->ws2812, task_data->event_data.idx, current_brightness_vals);
        ws2812_writeLEDs(task_data->ws2812);
        xSemaphoreGive(task_data->ws2812_mutex);
        vTaskDelay(BREATH_TASK_INTERVAL); // no taskdelayuntil since this is just for show
    }
}