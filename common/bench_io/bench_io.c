#include "bench_io.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#define LINK_UART UART_NUM_1

uint32_t bench_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void link_init(int tx_gpio, int rx_gpio, int baud)
{
    const uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* An unplugged wire must read as idle (high), not as noise. */
    gpio_pullup_en(rx_gpio);
}

int link_read(uint8_t *buf, size_t max)
{
    int n = uart_read_bytes(LINK_UART, buf, (uint32_t)max, 0);
    return n < 0 ? 0 : n;
}

static uint32_t s_noise_ppm;
static uint32_t s_corrupted;

void link_set_noise_ppm(uint32_t ppm)
{
    s_noise_ppm = ppm > 1000000u ? 1000000u : ppm;
}

uint32_t link_noise_ppm(void)
{
    return s_noise_ppm;
}

uint32_t link_corrupted_bytes(void)
{
    return s_corrupted;
}

void link_write(const uint8_t *buf, size_t len)
{
    /* TX ring buffer is 512 bytes; a frame is <= 30 bytes every 100 ms, so this never blocks. */
    if (s_noise_ppm == 0u || len > 64u) {
        uart_write_bytes(LINK_UART, buf, len);
        return;
    }
    uint8_t tmp[64];
    memcpy(tmp, buf, len);
    for (size_t i = 0; i < len; i++) {
        if (esp_random() % 1000000u < s_noise_ppm) {
            tmp[i] ^= (uint8_t)(1u << (esp_random() % 8u));
            s_corrupted++;
        }
    }
    uart_write_bytes(LINK_UART, tmp, len);
}

const char *bench_fw_version(void)
{
    return esp_app_get_description()->version;
}

const char *bench_build_info(void)
{
    /* project_name[32] + version[32] + date[16] + time[16] + idf_ver[32] + text: fits in 160 */
    static char buf[160];
    if (buf[0] == '\0') {
        const esp_app_desc_t *d = esp_app_get_description();
        snprintf(buf, sizeof(buf), "%s %s built %s %s, IDF %s", d->project_name, d->version,
                 d->date, d->time, d->idf_ver);
    }
    return buf;
}

uint16_t bench_boot_id(void)
{
    static uint16_t id;
    while (id == 0u) {
        id = (uint16_t)(esp_random() & 0xFFFFu);
    }
    return id;
}

const char *bench_reset_reason_str(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXTERNAL";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "OTHER_WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    default: return "UNKNOWN";
    }
}

void bench_wdt_subscribe(void)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
}

void bench_wdt_feed(void)
{
    esp_task_wdt_reset();
}

void console_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 2048; /* room for long lines (OTA chunks are ~520 chars) */
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    usb_serial_jtag_vfs_use_driver(); /* printf goes through the same driver */
    setvbuf(stdout, NULL, _IOLBF, 0);
}

bool console_poll_line(char *line, size_t max)
{
    static char acc[640];
    static size_t len;
    static bool overflow;
    uint8_t c;
    while (usb_serial_jtag_read_bytes(&c, 1, 0) == 1) {
        if (c == '\r' || c == '\n') {
            if (overflow) {
                overflow = false;
                len = 0;
                continue;
            }
            if (len == 0) {
                continue;
            }
            acc[len] = '\0';
            len = 0;
            /* trim */
            char *s = acc;
            while (*s && isspace((unsigned char)*s)) s++;
            size_t n = strlen(s);
            while (n > 0 && isspace((unsigned char)s[n - 1])) s[--n] = '\0';
            if (n == 0 || n >= max) {
                continue;
            }
            memcpy(line, s, n + 1);
            return true;
        }
        if (len < sizeof(acc) - 1) {
            acc[len++] = (char)c;
        } else {
            overflow = true;
        }
    }
    return false;
}
