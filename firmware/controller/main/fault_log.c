#include "fault_log.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define NS "faultlog"

typedef struct {
    uint32_t boot;
    uint32_t uptime_ms;
    uint8_t cause;
    char cause_name[20];
    char detail[56];
} fault_rec_t;

typedef struct {
    uint32_t next; /* total records ever written; slot = next % LEN */
    fault_rec_t rec[FAULT_LOG_LEN];
} fault_store_t;

static uint32_t s_boot;
static QueueHandle_t s_q;
static uint32_t s_dropped;
static fault_store_t s_store; /* RAM mirror, only touched by the writer task and print */
static bool s_nvs_ok;

static void load(void)
{
    nvs_handle_t h;
    memset(&s_store, 0, sizeof(s_store));
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(s_store);
    if (nvs_get_blob(h, "log", &s_store, &sz) != ESP_OK || sz != sizeof(s_store)) {
        memset(&s_store, 0, sizeof(s_store));
    }
    nvs_close(h);
}

static esp_err_t save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, "log", &s_store, sizeof(s_store));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static void writer_task(void *arg)
{
    (void)arg;
    fault_rec_t r;
    for (;;) {
        if (xQueueReceive(s_q, &r, portMAX_DELAY) == pdTRUE) {
            s_store.rec[s_store.next % FAULT_LOG_LEN] = r;
            s_store.next++;
            const esp_err_t err = s_nvs_ok ? save() : ESP_ERR_INVALID_STATE;
            if (err != ESP_OK) {
                printf("WARN fault log: NVS write failed (%s)\n", esp_err_to_name(err));
            }
        }
    }
}

uint32_t fault_log_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition layout changed: only this firmware's data lives there. */
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    s_nvs_ok = (err == ESP_OK);
    if (s_nvs_ok) {
        nvs_handle_t h;
        if (nvs_open(NS, NVS_READWRITE, &h) == ESP_OK) {
            nvs_get_u32(h, "boots", &s_boot);
            s_boot++;
            nvs_set_u32(h, "boots", s_boot);
            nvs_commit(h);
            nvs_close(h);
        }
        load();
    } else {
        printf("WARN fault log: NVS unavailable (%s), log is RAM-only\n", esp_err_to_name(err));
    }
    s_q = xQueueCreate(4, sizeof(fault_rec_t));
    xTaskCreate(writer_task, "faultlog", 3072, NULL, 1, NULL);
    return s_boot;
}

uint32_t fault_log_boot_count(void)
{
    return s_boot;
}

void fault_log_record(uint8_t cause, const char *cause_name, const char *detail, uint32_t uptime_ms)
{
    fault_rec_t r = {.boot = s_boot, .uptime_ms = uptime_ms, .cause = cause};
    snprintf(r.cause_name, sizeof(r.cause_name), "%s", cause_name);
    snprintf(r.detail, sizeof(r.detail), "%s", detail);
    if (xQueueSend(s_q, &r, 0) != pdTRUE) {
        s_dropped++;
    }
}

void fault_log_print(void)
{
    const uint32_t n = s_store.next < FAULT_LOG_LEN ? s_store.next : FAULT_LOG_LEN;
    printf("FAULTLOG boot=%lu total_recorded=%lu shown=%lu dropped=%lu nvs=%s\n",
           (unsigned long)s_boot, (unsigned long)s_store.next, (unsigned long)n,
           (unsigned long)s_dropped, s_nvs_ok ? "ok" : "unavailable");
    for (uint32_t i = 0; i < n; i++) {
        const fault_rec_t *r = &s_store.rec[(s_store.next - n + i) % FAULT_LOG_LEN];
        printf("FAULTLOG #%lu boot=%lu t=%lu %s : %s\n", (unsigned long)(s_store.next - n + i + 1),
               (unsigned long)r->boot, (unsigned long)r->uptime_ms, r->cause_name, r->detail);
    }
}

void fault_log_clear(void)
{
    memset(&s_store, 0, sizeof(s_store));
    if (s_nvs_ok) save();
    printf("FAULTLOG cleared\n");
}
