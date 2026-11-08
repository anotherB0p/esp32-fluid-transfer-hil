#include "ota_console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "psa/crypto.h"

static esp_ota_handle_t s_handle;
static const esp_partition_t *s_target;
static bool s_active;
static uint32_t s_size, s_written;
static uint8_t s_expected[32];
static psa_hash_operation_t s_hash;

static bool s_pending_verify;
static bool s_selftest_done;

static const char *state_name(esp_ota_img_states_t st)
{
    switch (st) {
    case ESP_OTA_IMG_NEW: return "NEW";
    case ESP_OTA_IMG_PENDING_VERIFY: return "PENDING_VERIFY";
    case ESP_OTA_IMG_VALID: return "VALID";
    case ESP_OTA_IMG_INVALID: return "INVALID";
    case ESP_OTA_IMG_ABORTED: return "ABORTED";
    case ESP_OTA_IMG_UNDEFINED: return "UNDEFINED";
    default: return "?";
    }
}

static void report(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(run, &st);
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    printf("OTA status: running=%s @0x%06lx state=%s version=%s next_slot=%s%s\n",
           run ? run->label : "?", run ? (unsigned long)run->address : 0ul, state_name(st),
           esp_app_get_description()->version, next ? next->label : "none",
           s_active ? " (update in progress)" : "");
}

void ota_boot_report(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        s_pending_verify = true;
        printf("OTA: booted a NEW image in %s, state PENDING_VERIFY: self-test must pass within "
               "%u ms or the bootloader rolls back\n",
               run->label, (unsigned)OTA_SELFTEST_TIMEOUT_MS);
    }
    const esp_partition_t *last_bad = esp_ota_get_last_invalid_partition();
    if (last_bad) {
        printf("OTA: note: slot %s holds an image marked INVALID (rolled back earlier)\n",
               last_bad->label);
    }
    report();
}

bool ota_in_progress(void)
{
    return s_active;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void abort_update(const char *why)
{
    if (s_active) {
        esp_ota_abort(s_handle);
        psa_hash_abort(&s_hash);
    }
    s_active = false;
    printf("OTA ERR %s (update aborted, running image unchanged)\n", why);
}

static void cmd_begin(char *a_size, char *a_sha, bool transfer_active)
{
    if (transfer_active) {
        printf("OTA ERR refused: a transfer is active (allowed in IDLE, COMPLETE or FAULT)\n");
        return;
    }
    if (s_active) {
        abort_update("begin while an update was in progress");
    }
    if (!a_size || !a_sha || strlen(a_sha) != 64) {
        printf("OTA ERR usage: ota begin <size> <sha256 hex>\n");
        return;
    }
    for (int i = 0; i < 32; i++) {
        const int hi = hexval(a_sha[2 * i]), lo = hexval(a_sha[2 * i + 1]);
        if (hi < 0 || lo < 0) {
            printf("OTA ERR bad sha256 hex\n");
            return;
        }
        s_expected[i] = (uint8_t)(hi << 4 | lo);
    }
    s_size = (uint32_t)strtoul(a_size, NULL, 10);
    s_target = esp_ota_get_next_update_partition(NULL);
    if (!s_target || s_size == 0 || s_size > s_target->size) {
        printf("OTA ERR no slot or bad size %lu (slot size %lu)\n", (unsigned long)s_size,
               s_target ? (unsigned long)s_target->size : 0ul);
        return;
    }
    /* Sequential writes: erase sector by sector as data arrives, so no multi-second erase
     * blocks the 10 ms control loop (the watchdog would reset us). */
    esp_err_t err = esp_ota_begin(s_target, OTA_WITH_SEQUENTIAL_WRITES, &s_handle);
    if (err != ESP_OK) {
        printf("OTA ERR esp_ota_begin: %s\n", esp_err_to_name(err));
        return;
    }
    s_hash = psa_hash_operation_init();
    if (psa_crypto_init() != PSA_SUCCESS || psa_hash_setup(&s_hash, PSA_ALG_SHA_256) != PSA_SUCCESS) {
        esp_ota_abort(s_handle);
        printf("OTA ERR sha256 init failed\n");
        return;
    }
    s_written = 0;
    s_active = true;
    printf("OTA READY slot=%s @0x%06lx size=%lu\n", s_target->label, (unsigned long)s_target->address,
           (unsigned long)s_size);
}

static void cmd_write(char *a_off, char *a_b64)
{
    if (!s_active) {
        printf("OTA ERR no update in progress\n");
        return;
    }
    if (!a_off || !a_b64) {
        abort_update("usage: ota w <offset> <base64>");
        return;
    }
    const uint32_t off = (uint32_t)strtoul(a_off, NULL, 10);
    if (off != s_written) {
        /* The host may resend the last chunk if our OK got lost: acknowledge, don't rewrite. */
        printf("OTA OK %lu (offset %lu ignored, expected %lu)\n", (unsigned long)s_written,
               (unsigned long)off, (unsigned long)s_written);
        return;
    }
    uint8_t buf[480];
    size_t n = 0;
    if (mbedtls_base64_decode(buf, sizeof(buf), &n, (const unsigned char *)a_b64, strlen(a_b64)) != 0 ||
        n == 0 || s_written + n > s_size) {
        abort_update("bad chunk (base64 or size)");
        return;
    }
    esp_err_t err = esp_ota_write(s_handle, buf, n);
    if (err != ESP_OK) {
        abort_update(esp_err_to_name(err));
        return;
    }
    psa_hash_update(&s_hash, buf, n);
    s_written += (uint32_t)n;
    printf("OTA OK %lu\n", (unsigned long)s_written);
}

static void cmd_end(void)
{
    if (!s_active) {
        printf("OTA ERR no update in progress\n");
        return;
    }
    if (s_written != s_size) {
        abort_update("size mismatch at end");
        return;
    }
    uint8_t got[32];
    size_t len = 0;
    psa_hash_finish(&s_hash, got, sizeof(got), &len);
    if (len != 32 || memcmp(got, s_expected, 32) != 0) {
        esp_ota_abort(s_handle);
        s_active = false;
        printf("OTA ERR sha256 mismatch: image corrupted in transfer (running image unchanged)\n");
        return;
    }
    esp_err_t err = esp_ota_end(s_handle); /* validates the app image structure/checksum */
    s_active = false;
    if (err != ESP_OK) {
        printf("OTA ERR image validation failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = esp_ota_set_boot_partition(s_target);
    if (err != ESP_OK) {
        printf("OTA ERR set boot partition: %s\n", esp_err_to_name(err));
        return;
    }
    printf("OTA DONE sha256 ok, %lu bytes, next boot from %s (PENDING_VERIFY). Rebooting.\n",
           (unsigned long)s_written, s_target->label);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

void ota_handle(char *args, bool transfer_active, uint32_t now_ms)
{
    (void)now_ms;
    char *sub = strtok(args, " ");
    char *a1 = strtok(NULL, " ");
    char *a2 = strtok(NULL, " ");
    if (!sub || strcmp(sub, "status") == 0) {
        report();
    } else if (strcmp(sub, "begin") == 0) {
        cmd_begin(a1, a2, transfer_active);
    } else if (strcmp(sub, "w") == 0) {
        cmd_write(a1, a2);
    } else if (strcmp(sub, "end") == 0) {
        cmd_end();
    } else if (strcmp(sub, "abort") == 0) {
        abort_update("aborted by operator");
    } else {
        printf("OTA ERR usage: ota status|begin|w|end|abort\n");
    }
}

void ota_selftest_poll(bool healthy, uint32_t now_ms)
{
    if (!s_pending_verify || s_selftest_done) {
        return;
    }
#if OTA_TEST_BAD_IMAGE
    /* Deliberately broken build (bench rollback test): never passes the self-test. */
    healthy = false;
#endif
    if (healthy) {
        s_selftest_done = true;
        const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        printf("OTA: self-test PASSED after %lu ms (compatible plant, link OK, commands "
               "acknowledged): image marked VALID (%s)\n",
               (unsigned long)now_ms, esp_err_to_name(err));
    } else if (now_ms > OTA_SELFTEST_TIMEOUT_MS) {
        s_selftest_done = true;
        printf("OTA: self-test FAILED within %u ms: rolling back to the previous image\n",
               (unsigned)OTA_SELFTEST_TIMEOUT_MS);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
}
