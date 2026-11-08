#include "bench_io.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_err.h"
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

void link_write(const uint8_t *buf, size_t len)
{
    /* TX ring buffer is 512 bytes; a frame is <= 30 bytes every 100 ms, so this never blocks. */
    uart_write_bytes(LINK_UART, buf, len);
}

void console_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    usb_serial_jtag_vfs_use_driver(); /* printf goes through the same driver */
    setvbuf(stdout, NULL, _IOLBF, 0);
}

bool console_poll_line(char *line, size_t max)
{
    static char acc[96];
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
