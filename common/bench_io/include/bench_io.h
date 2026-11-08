/*
 * Small ESP-IDF helpers shared by both firmwares:
 *  - monotonic millisecond clock (esp_timer, never goes backwards);
 *  - inter-board link on UART1 (non-blocking reads);
 *  - line-based operator console on the USB-Serial/JTAG port.
 */
#ifndef BENCH_IO_H
#define BENCH_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

uint32_t bench_now_ms(void);

void link_init(int tx_gpio, int rx_gpio, int baud);
int link_read(uint8_t *buf, size_t max);            /* non-blocking, returns bytes read */
void link_write(const uint8_t *buf, size_t len);

void console_init(void);
/* Non-blocking. Returns true and fills `line` (NUL-terminated, trimmed) when a full line
 * has been typed. Lines longer than `max - 1` are discarded. */
bool console_poll_line(char *line, size_t max);

#endif /* BENCH_IO_H */
