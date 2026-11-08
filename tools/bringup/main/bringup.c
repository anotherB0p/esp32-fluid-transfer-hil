/*
 * Board bring-up probe (preparation step, not part of the demo firmware).
 *
 * ESP32-S3 (A13, controller): cycles the on-board WS2812 on GPIO21 red/green/blue.
 * ESP32-C3 (A09, plant):      scans I2C on SDA=GPIO5 / SCL=GPIO6 and, if an SSD1306
 *                              answers at 0x3C/0x3D, turns every OLED pixel on.
 * Both: UART1 link test. Every second each board sends "PING <target> <n>\n" on
 * UART1 TX and logs every line received on UART1 RX, so both directions of the
 * wiring can be checked from the two USB consoles.
 *   S3: TX=GPIO1 RX=GPIO2     C3: TX=GPIO3 RX=GPIO4     115200 8N1
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_mac.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "bringup";

#define LINK_UART UART_NUM_1
#define LINK_BAUD 115200

#if CONFIG_IDF_TARGET_ESP32S3
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#define LINK_TX 1
#define LINK_RX 2
#define LED_GPIO 21

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;

static void board_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, /* 0.1 us per tick */
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_chan));
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = {.level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9},
        .bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3},
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_enc));
    ESP_ERROR_CHECK(rmt_enable(s_chan));
}

static void board_tick(int i)
{
    /* WS2812 byte order is G, R, B. Low brightness. */
    static const struct { const char *name; uint8_t grb[3]; } colors[] = {
        {"RED", {0, 32, 0}}, {"GREEN", {32, 0, 0}}, {"BLUE", {0, 0, 32}},
    };
    int c = i % 3;
    rmt_transmit_config_t tcfg = {.loop_count = 0};
    ESP_ERROR_CHECK(rmt_transmit(s_chan, s_enc, colors[c].grb, 3, &tcfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_chan, 100));
    ESP_LOGI(TAG, "heartbeat %d: WS2812 on GPIO%d set to %s", i, LED_GPIO, colors[c].name);
}

#elif CONFIG_IDF_TARGET_ESP32C3
#include "driver/i2c_master.h"

#define LINK_TX 3
#define LINK_RX 4
#define SDA_GPIO 5
#define SCL_GPIO 6

static void board_init(void)
{
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    int oled_addr = -1;
    for (int addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "I2C device ACK at 0x%02X (SDA=GPIO%d SCL=GPIO%d)", addr, SDA_GPIO, SCL_GPIO);
            if (addr == 0x3C || addr == 0x3D) {
                oled_addr = addr;
            }
        }
    }
    if (oled_addr < 0) {
        ESP_LOGW(TAG, "no SSD1306 at 0x3C/0x3D");
        return;
    }
    i2c_master_dev_handle_t dev = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = oled_addr,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));
    /* control byte 0x00 = command stream: display off, charge pump on,
     * display on, entire display ON (ignores RAM). */
    static const uint8_t cmds[] = {0x00, 0xAE, 0x8D, 0x14, 0xAF, 0xA5};
    esp_err_t err = i2c_master_transmit(dev, cmds, sizeof(cmds), 100);
    ESP_LOGI(TAG, "SSD1306 at 0x%02X: all-pixels-on command -> %s", oled_addr, esp_err_to_name(err));
}

static void board_tick(int i)
{
    ESP_LOGI(TAG, "heartbeat %d", i);
}
#else
#error "bring-up probe supports only ESP32-S3 and ESP32-C3"
#endif

static void link_init(void)
{
    uart_config_t cfg = {
        .baud_rate = LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, LINK_TX, LINK_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_pullup_en(LINK_RX);
    ESP_LOGI(TAG, "link UART1 TX=GPIO%d RX=GPIO%d %d 8N1", LINK_TX, LINK_RX, LINK_BAUD);

    /* Self-test of the firmware RX path without wires: internal TX->RX loopback. */
    ESP_ERROR_CHECK(uart_set_loop_back(LINK_UART, true));
    static const char probe_msg[] = "LOOPBACK";
    uart_write_bytes(LINK_UART, probe_msg, sizeof(probe_msg) - 1);
    uart_wait_tx_done(LINK_UART, pdMS_TO_TICKS(50));
    char got[16] = {0};
    int n = uart_read_bytes(LINK_UART, got, sizeof(probe_msg) - 1, pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(uart_set_loop_back(LINK_UART, false));
    uart_flush_input(LINK_UART);
    ESP_LOGI(TAG, "link internal loopback self-test: %s (%d bytes)",
             (n == (int)sizeof(probe_msg) - 1 && memcmp(got, probe_msg, n) == 0) ? "PASS" : "FAIL", n);
}

/* Reads whatever arrived on the link and logs complete lines (non-printable bytes as hex). */
static void link_poll(void)
{
    static char line[96];
    static size_t len;
    uint8_t buf[64];
    int n = uart_read_bytes(LINK_UART, buf, sizeof(buf), 0);
    for (int k = 0; k < n; k++) {
        uint8_t b = buf[k];
        if (b == '\n' || len >= sizeof(line) - 5) {
            line[len] = 0;
            ESP_LOGI(TAG, "link RX: \"%s\"", line);
            len = 0;
        } else if (b >= 0x20 && b < 0x7F) {
            line[len++] = (char)b;
        } else if (b != '\r') {
            len += snprintf(line + len, sizeof(line) - len, "\\x%02X", b);
        }
    }
}

#if BRINGUP_PINMAP
#include "pinmap.inc"
#endif /* BRINGUP_PINMAP */

void app_main(void)
{
    esp_chip_info_t info;
    esp_chip_info(&info);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BASE);
    ESP_LOGI(TAG, "target=%s rev=%d cores=%d mac=%02x:%02x:%02x:%02x:%02x:%02x",
             CONFIG_IDF_TARGET, info.revision, info.cores,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#if BRINGUP_PINMAP
    pinmap_run();
#endif
    board_init();
    link_init();

    for (int i = 0;; i++) {
        board_tick(i);
        char msg[40];
        int m = snprintf(msg, sizeof(msg), "PING %s %d\n", CONFIG_IDF_TARGET, i);
        uart_write_bytes(LINK_UART, msg, m);
        for (int t = 0; t < 20; t++) { /* 1 s, polling the link every 50 ms */
            link_poll();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
