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

/* Test hook: flip one random bit in each transmitted byte with probability ppm/1e6.
 * Used to demonstrate CRC rejection and link supervision on the real wire. */
void link_set_noise_ppm(uint32_t ppm);
uint32_t link_noise_ppm(void);
uint32_t link_corrupted_bytes(void);

/* Firmware identity: app version (git describe of the project, set by ESP-IDF at
 * configure time), build date/time, IDF version; a random 16-bit id per boot. */
const char *bench_fw_version(void);
const char *bench_build_info(void);
uint16_t bench_boot_id(void);

/* Human-readable esp_reset_reason(). */
const char *bench_reset_reason_str(void);

/* Task watchdog: subscribe the calling task (timeout set in sdkconfig, panics -> reset). */
void bench_wdt_subscribe(void);
void bench_wdt_feed(void);

void console_init(void);
/* Non-blocking. Returns true and fills `line` (NUL-terminated, trimmed) when a full line
 * has been typed. Lines longer than `max - 1` are discarded. */
bool console_poll_line(char *line, size_t max);

#endif /* BENCH_IO_H */
