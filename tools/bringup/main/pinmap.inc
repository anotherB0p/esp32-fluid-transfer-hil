/*
 * Wiring diagnostic (included from bringup.c when BRINGUP_PINMAP=1).
 *
 * S3 identifies each of its free pads by a burst of N pulses, N = GPIO number
 * (40 ms high / 40 ms low), all pads in parallel, then a 1.5 s pause. Outputs use the
 * weakest drive strength (~5 mA) to limit current if a wire touches a supply rail.
 * C3 samples its pads as inputs only (pull-down) and reports, per pad, the number of
 * pulses in each burst, i.e. the S3 GPIO number wired to that C3 pad.
 * Only S3 drives, so a mis-wire cannot create output-vs-output contention.
 */
#if CONFIG_IDF_TARGET_ESP32S3
static void pinmap_run(void)
{
    /* Not driven: 0/3/45/46 strapping, 19/20 USB, 21 RGB LED, 26-37 flash/PSRAM. */
    static const int pins[] = {1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 43, 44};
    enum { N = sizeof(pins) / sizeof(pins[0]) };
    for (int k = 0; k < N; k++) {
        gpio_reset_pin(pins[k]);
        gpio_set_direction(pins[k], GPIO_MODE_OUTPUT);
        gpio_set_drive_capability(pins[k], GPIO_DRIVE_CAP_0);
        gpio_set_level(pins[k], 0);
    }
    ESP_LOGI(TAG, "PINMAP: each S3 pad sends a burst of N pulses, N = its GPIO number");
    for (int cycle = 0;; cycle++) {
        ESP_LOGI(TAG, "PINMAP: burst cycle %d", cycle);
        for (int p = 0; p < 44; p++) {
            for (int k = 0; k < N; k++) {
                gpio_set_level(pins[k], p < pins[k]);
            }
            vTaskDelay(pdMS_TO_TICKS(40));
            for (int k = 0; k < N; k++) {
                gpio_set_level(pins[k], 0);
            }
            vTaskDelay(pdMS_TO_TICKS(40));
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}
#else
static void pinmap_run(void)
{
    /* All broken-out C3 pads, as inputs. Reading strapping pins after boot is harmless. */
    static const int pins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
    enum { N = sizeof(pins) / sizeof(pins[0]) };
    int prev[N], count[N], high_ms[N];
    TickType_t last_edge[N];
    for (int k = 0; k < N; k++) {
        gpio_reset_pin(pins[k]);
        gpio_set_direction(pins[k], GPIO_MODE_INPUT);
        gpio_pullup_dis(pins[k]);
        gpio_pulldown_en(pins[k]);
        prev[k] = gpio_get_level(pins[k]);
        count[k] = 0;
        high_ms[k] = 0;
        last_edge[k] = 0;
    }
    ESP_LOGI(TAG, "PINMAP: listening on C3 GPIO0-10,20,21 (inputs, pull-down)");
    TickType_t last_summary = xTaskGetTickCount();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        TickType_t now = xTaskGetTickCount();
        for (int k = 0; k < N; k++) {
            int v = gpio_get_level(pins[k]);
            if (v) high_ms[k] += 10;
            if (v && !prev[k]) {
                count[k]++;
                last_edge[k] = now;
            }
            prev[k] = v;
            if (count[k] > 0 && (now - last_edge[k]) > pdMS_TO_TICKS(500)) {
                ESP_LOGI(TAG, "PINMAP: C3 GPIO%d received a burst of %d pulses -> wired to S3 GPIO%d",
                         pins[k], count[k], count[k]);
                count[k] = 0;
            }
        }
        if ((now - last_summary) >= pdMS_TO_TICKS(6000)) {
            char line[200];
            int n = 0;
            for (int k = 0; k < N; k++) {
                n += snprintf(line + n, sizeof(line) - n, " G%d:%d%%", pins[k], high_ms[k] / 60);
                high_ms[k] = 0;
            }
            ESP_LOGI(TAG, "PINMAP: %%time high over 6 s:%s", line);
            last_summary = now;
        }
    }
}
#endif
