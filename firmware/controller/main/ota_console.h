/*
 * Firmware update over the existing wired console (no Wi-Fi), A/B partitions with
 * bootloader rollback.
 *
 *   ota begin <size> <sha256-hex>   prepare the inactive slot (only when not transferring)
 *   ota w <offset> <base64>         write the next chunk (strictly sequential), reply OTA OK
 *   ota end                         check size + SHA-256 of what was written, validate the
 *                                   image, switch the boot slot, reboot
 *   ota abort | ota status
 *
 * After booting a new image it is PENDING_VERIFY. ota_selftest_poll() marks it valid only
 * when the controller sees a compatible plant over a working link within
 * OTA_SELFTEST_TIMEOUT_MS. Otherwise, or if the new image crashes / is reset by the
 * watchdog before that, the bootloader returns to the previous image.
 */
#ifndef OTA_CONSOLE_H
#define OTA_CONSOLE_H

#include <stdbool.h>
#include <stdint.h>

#define OTA_SELFTEST_TIMEOUT_MS 10000u

/* Prints boot slot/state; call once at boot. */
void ota_boot_report(void);

/* Handles a console line starting with "ota". `transfer_active` blocks 'begin'. */
void ota_handle(char *args, bool transfer_active, uint32_t now_ms);

/* Call every loop. `healthy` = compatible peer + fresh link + commands acknowledged. */
void ota_selftest_poll(bool healthy, uint32_t now_ms);

bool ota_in_progress(void);

#endif
