/*
 * Transfer controller firmware - ESP32-S3 (board A13).
 *
 * One periodic task (10 ms period, vTaskDelayUntil):
 *   - drain UART1, feed the protocol decoder, hand valid TELEM to the FSM;
 *   - handle operator console lines (USB-Serial/JTAG);
 *   - ctl_tick(); send CMD every 100 ms; status line every 1 s;
 *   - mirror the state on the on-board WS2812 (GPIO21).
 * Nothing in the loop blocks: UART reads/writes and console reads are non-blocking.
 *
 * Link: UART1 TX=GPIO1 RX=GPIO2 115200 8N1 (see docs/hardware.md).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "bench_io.h"
#include "controller_fsm.h"
#include "proto.h"
#include "status_led.h"

#define LINK_TX_GPIO 1
#define LINK_RX_GPIO 2
#define LINK_BAUD 115200
#define LOOP_MS 10u
#define CMD_PERIOD_MS 100u
#define STAT_PERIOD_MS 1000u
#define DEFAULT_TARGET_ML 500u

static ctl_t s_ctl;
static proto_decoder_t s_dec;
static uint8_t s_cmd_seq;
static uint32_t s_mute_until_ms; /* test hook: suppress CMD transmission */
static bool s_muted;
static uint32_t s_telem_count, s_telem_seq_gaps;
static bool s_have_seq;
static uint8_t s_last_seq;

static void on_event(void *ctx, ctl_state_t from, ctl_state_t to, ctl_fault_t cause, const char *detail)
{
    (void)ctx;
    const uint32_t now = bench_now_ms();
    if (from == to) {
        printf("EVT t=%lu %s phase=%s : %s\n", (unsigned long)now, ctl_state_name(to),
               ctl_phase_name(s_ctl.phase), detail);
    } else if (to == CTL_FAULT) {
        printf("EVT t=%lu %s -> FAULT cause=%s : %s\n", (unsigned long)now, ctl_state_name(from),
               ctl_fault_name(cause), detail);
    } else {
        printf("EVT t=%lu %s -> %s%s%s : %s\n", (unsigned long)now, ctl_state_name(from),
               ctl_state_name(to), s_ctl.phase != PH_NONE ? " phase=" : "",
               s_ctl.phase != PH_NONE ? ctl_phase_name(s_ctl.phase) : "", detail);
    }
}

static void print_status(uint32_t now)
{
    const proto_telem_t *t = &s_ctl.telem;
    const uint32_t age = ctl_link_age_ms(&s_ctl, now);
    printf("STAT t=%lu state=%s phase=%s fault=%s target=%u | src=%u dst=%u flow=%u "
           "valve cmd=%u act=%u pump cmd=%u act=%u plant_failsafe=%u | link=%s age=%ld "
           "rx_ok=%lu crc_err=%lu len_err=%lu range_err=%lu junk=%lu seq_gaps=%lu%s\n",
           (unsigned long)now, ctl_state_name(s_ctl.state), ctl_phase_name(s_ctl.phase),
           ctl_fault_name(s_ctl.fault), s_ctl.target_dst_ml, t->src_ml, t->dst_ml, t->flow_ml_s,
           s_ctl.out_valve, t->valve_open, s_ctl.out_pump, t->pump_running,
           (t->flags & PLANT_FLAG_FAILSAFE) ? 1 : 0, ctl_link_ok(&s_ctl, now) ? "OK" : "DOWN",
           age == UINT32_MAX ? -1L : (long)age, (unsigned long)s_dec.stats.frames_ok,
           (unsigned long)s_dec.stats.crc_errors, (unsigned long)s_dec.stats.len_errors,
           (unsigned long)s_dec.stats.range_errors, (unsigned long)s_dec.stats.junk_bytes,
           (unsigned long)s_telem_seq_gaps, s_muted ? " CMD_TX_MUTED" : "");
    if (s_ctl.state == CTL_FAULT) {
        printf("STAT fault_detail: %s\n", s_ctl.fault_detail);
    }
}

static void handle_line(char *line, uint32_t now)
{
    printf("> %s\n", line);
    char *cmd = strtok(line, " ");
    char *arg = strtok(NULL, " ");
    const char *why = NULL;

    if (strcmp(cmd, "start") == 0) {
        long target = arg ? strtol(arg, NULL, 10) : (long)DEFAULT_TARGET_ML;
        if (target <= 0 || target > 65535) {
            printf("ERR start: bad target\n");
        } else if (!ctl_start(&s_ctl, (uint16_t)target, now, &why)) {
            printf("ERR start refused: %s\n", why);
        }
    } else if (strcmp(cmd, "stop") == 0) {
        ctl_stop(&s_ctl, now);
    } else if (strcmp(cmd, "reset") == 0) {
        if (!ctl_reset(&s_ctl, now, &why)) {
            printf("ERR reset refused: %s\n", why);
        }
    } else if (strcmp(cmd, "status") == 0) {
        print_status(now);
    } else if (strcmp(cmd, "mute") == 0) {
        long ms = arg ? strtol(arg, NULL, 10) : 2000;
        s_mute_until_ms = now + (uint32_t)(ms > 0 ? ms : 0);
        s_muted = ms > 0;
        printf("EVT t=%lu TEST: CMD transmission muted for %ld ms\n", (unsigned long)now, ms);
    } else if (strcmp(cmd, "reboot") == 0) {
        printf("EVT t=%lu rebooting on operator request\n", (unsigned long)now);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
    } else if (strcmp(cmd, "help") == 0) {
        printf("commands: start [target_mL] | stop | reset | status | mute <ms> | reboot\n");
    } else {
        printf("ERR unknown command '%s' (try 'help')\n", cmd);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    uint8_t rx[64];
    char line[80];
    proto_msg_t msg;
    uint32_t last_cmd = 0, last_stat = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        const uint32_t now = bench_now_ms();

        int n;
        while ((n = link_read(rx, sizeof(rx))) > 0) {
            for (int i = 0; i < n; i++) {
                if (proto_decoder_feed(&s_dec, rx[i], &msg) && msg.type == MSG_TELEM) {
                    if (s_have_seq && msg.seq != (uint8_t)(s_last_seq + 1u)) {
                        s_telem_seq_gaps++;
                    }
                    s_have_seq = true;
                    s_last_seq = msg.seq;
                    s_telem_count++;
                    ctl_on_telem(&s_ctl, &msg.u.telem, now);
                }
            }
        }

        while (console_poll_line(line, sizeof(line))) {
            handle_line(line, now);
        }

        ctl_tick(&s_ctl, now);

        if (s_muted && (int32_t)(now - s_mute_until_ms) >= 0) {
            s_muted = false;
            printf("EVT t=%lu TEST: CMD transmission resumed\n", (unsigned long)now);
        }
        if ((uint32_t)(now - last_cmd) >= CMD_PERIOD_MS) {
            last_cmd = now;
            if (!s_muted) {
                uint8_t f[PROTO_MAX_FRAME];
                const proto_cmd_t c = {.pump_on = s_ctl.out_pump, .valve_open = s_ctl.out_valve};
                link_write(f, proto_encode_cmd(s_cmd_seq++, &c, f));
            }
        }
        if ((uint32_t)(now - last_stat) >= STAT_PERIOD_MS) {
            last_stat = now;
            print_status(now);
        }

        status_led_update(s_ctl.state, ctl_link_ok(&s_ctl, now), now);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
    }
}

void app_main(void)
{
    console_init();
    status_led_init();
    link_init(LINK_TX_GPIO, LINK_RX_GPIO, LINK_BAUD);
    proto_decoder_init(&s_dec);

    ctl_config_t cfg;
    ctl_default_config(&cfg);
    ctl_init(&s_ctl, &cfg, bench_now_ms(), on_event, NULL);

    printf("EVT t=%lu BOOT controller (ESP32-S3) reset_reason=%d state=IDLE. "
           "Nothing resumes after reboot: send 'start' to begin.\n",
           (unsigned long)bench_now_ms(), (int)esp_reset_reason());
    printf("config: min_src=%u dst_high=%u link_timeout=%lu valve_timeout=%lu flow_grace=%lu "
           "flow_min=%u\n",
           cfg.min_src_ml, cfg.dst_high_ml, (unsigned long)cfg.link_timeout_ms,
           (unsigned long)cfg.valve_timeout_ms, (unsigned long)cfg.flow_grace_ms, cfg.flow_min_ml_s);

    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);
}
