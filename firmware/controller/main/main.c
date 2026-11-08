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
#include "fault_log.h"
#include "proto.h"
#include "status_led.h"
#include "blackbox.h"
#include "ota_console.h"

#define LINK_TX_GPIO 1
#define LINK_RX_GPIO 2
#define LINK_BAUD 115200
#define LOOP_MS 10u
#define CMD_PERIOD_MS 100u
#define STAT_PERIOD_MS 1000u
#define HELLO_PERIOD_MS 500u
#define DEFAULT_TARGET_ML 500u

static ctl_t s_ctl;
static bool s_stream; /* 'stream on': compact TLM line every 100 ms for tools/dashboard.py */
static blackbox_t s_bb;
static proto_hello_t s_my_hello;
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
        fault_log_record((uint8_t)cause, ctl_fault_name(cause), detail, now);
        bb_trigger(&s_bb, (uint8_t)cause, now); /* keeps recording 1 s of the reaction */
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
    printf("STAT peer %s proto=v%u boot_id=%04x fw_id=%08lx reboots_seen=%lu | self fw=%s "
           "boot_id=%04x proto=v%u\n",
           s_ctl.have_peer ? (ctl_peer_ok(&s_ctl) ? "OK" : "INCOMPATIBLE") : "NONE",
           s_ctl.peer.proto_ver, s_ctl.peer.boot_id, (unsigned long)s_ctl.peer.fw_id,
           (unsigned long)s_ctl.peer_reboots, bench_fw_version(), s_my_hello.boot_id,
           s_my_hello.proto_ver);
    const ctl_rtt_t *r = &s_ctl.rtt;
    printf("STAT diag boot=%lu cmd_ack=%s rtt_ms last=%lu min=%lu avg=%lu max=%lu n=%lu "
           "tx_noise_ppm=%lu tx_corrupted=%lu\n",
           (unsigned long)fault_log_boot_count(), ctl_ack_ok(&s_ctl, now) ? "OK" : "STALE",
           (unsigned long)r->last_ms, (unsigned long)r->min_ms,
           (unsigned long)(r->count ? r->sum_ms / r->count : 0), (unsigned long)r->max_ms,
           (unsigned long)r->count, (unsigned long)link_noise_ppm(),
           (unsigned long)link_corrupted_bytes());
    if (s_ctl.state == CTL_FAULT) {
        printf("STAT fault_detail: %s\n", s_ctl.fault_detail);
    }
}

static void handle_line(char *line, uint32_t now)
{
    if (strncmp(line, "ota", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        if (strncmp(line, "ota w ", 6) != 0) { /* do not echo every data chunk */
            printf("> %s\n", line);
        }
        ota_handle(line + 3, s_ctl.state == CTL_CHECK || s_ctl.state == CTL_TRANSFER, now);
        return;
    }
    printf("> %s\n", line);
    char *cmd = strtok(line, " ");
    char *arg = strtok(NULL, " ");
    const char *why = NULL;

    if (strcmp(cmd, "start") == 0) {
        long target = arg ? strtol(arg, NULL, 10) : (long)DEFAULT_TARGET_ML;
        if (ota_in_progress()) {
            printf("ERR start refused: firmware update in progress\n");
        } else if (target <= 0 || target > 65535) {
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
    } else if (strcmp(cmd, "stream") == 0) {
        s_stream = arg && strcmp(arg, "on") == 0;
        printf("EVT t=%lu telemetry stream %s\n", (unsigned long)now, s_stream ? "ON" : "OFF");
    } else if (strcmp(cmd, "blackbox") == 0) {
        fault_log_print_blackbox();
    } else if (strcmp(cmd, "version") == 0) {
        printf("VERSION %s | protocol v%u | boot_id %04x\n", bench_build_info(), PROTO_VERSION,
               s_my_hello.boot_id);
    } else if (strcmp(cmd, "faults") == 0) {
        if (arg && strcmp(arg, "clear") == 0) {
            fault_log_clear();
        } else {
            fault_log_print();
        }
    } else if (strcmp(cmd, "noise") == 0) {
        long ppm = arg ? strtol(arg, NULL, 10) : 0;
        link_set_noise_ppm((uint32_t)(ppm > 0 ? ppm : 0));
        printf("EVT t=%lu TEST: CMD TX bit-flip noise %lu ppm per byte\n", (unsigned long)now,
               (unsigned long)link_noise_ppm());
    } else if (strcmp(cmd, "hang") == 0) {
        printf("EVT t=%lu TEST: control task hangs now (watchdog must reset the board)\n",
               (unsigned long)now);
        fflush(stdout);
        for (;;) {
            /* deliberately stuck: no WDT feed, no yield */
        }
    } else if (strcmp(cmd, "reboot") == 0) {
        printf("EVT t=%lu rebooting on operator request\n", (unsigned long)now);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
    } else if (strcmp(cmd, "help") == 0) {
        printf("commands: start [target_mL] | stop | reset | status | version | faults [clear] | "
               "blackbox | stream on|off | ota status|begin|w|end|abort | mute <ms> | noise <ppm> | "
               "hang | reboot\n");
    } else {
        printf("ERR unknown command '%s' (try 'help')\n", cmd);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    uint8_t rx[64];
    static char line[640]; /* OTA data lines are ~520 chars */
    proto_msg_t msg;
    uint32_t last_cmd = 0, last_stat = 0;
    uint32_t last_hello = bench_now_ms() - HELLO_PERIOD_MS; /* send one right away */
    uint8_t hello_seq = 0;
    TickType_t wake = xTaskGetTickCount();
    bench_wdt_subscribe();

    for (;;) {
        bench_wdt_feed();
        const uint32_t now = bench_now_ms();

        int n;
        while ((n = link_read(rx, sizeof(rx))) > 0) {
            for (int i = 0; i < n; i++) {
                if (!proto_decoder_feed(&s_dec, rx[i], &msg)) {
                    continue;
                }
                if (msg.type == MSG_TELEM) {
                    if (s_have_seq && msg.seq != (uint8_t)(s_last_seq + 1u)) {
                        s_telem_seq_gaps++;
                    }
                    s_have_seq = true;
                    s_last_seq = msg.seq;
                    s_telem_count++;
                    ctl_on_telem(&s_ctl, &msg.u.telem, now);
                } else if (msg.type == MSG_HELLO) {
                    const proto_hello_t *h = &msg.u.hello;
                    switch (ctl_on_hello(&s_ctl, h, now)) {
                    case CTL_HELLO_FIRST:
                        printf("EVT t=%lu HELLO from plant: protocol v%u boot_id=%04x fw_id=%08lx "
                               "-> compatible\n",
                               (unsigned long)now, h->proto_ver, h->boot_id, (unsigned long)h->fw_id);
                        break;
                    case CTL_HELLO_PEER_REBOOT:
                        printf("EVT t=%lu HELLO: plant boot_id changed to %04x (plant rebooted)\n",
                               (unsigned long)now, h->boot_id);
                        break;
                    case CTL_HELLO_INCOMPATIBLE: {
                        static uint32_t last_warn;
                        if ((uint32_t)(now - last_warn) > 5000u || last_warn == 0u) {
                            last_warn = now;
                            printf("EVT t=%lu HELLO: INCOMPATIBLE peer role=%u protocol v%u "
                                   "(expected plant v%u) - start refused\n",
                                   (unsigned long)now, h->role, h->proto_ver, PROTO_VERSION);
                        }
                        break;
                    }
                    case CTL_HELLO_SAME:
                    default:
                        break;
                    }
                }
            }
        }

        while (console_poll_line(line, sizeof(line))) {
            handle_line(line, now);
        }

        ctl_tick(&s_ctl, now);
        ota_selftest_poll(ctl_peer_ok(&s_ctl) && ctl_link_ok(&s_ctl, now) && ctl_ack_ok(&s_ctl, now) &&
                              !(s_ctl.telem.flags & PLANT_FLAG_PEER_BAD),
                          now);

        if (s_muted && (int32_t)(now - s_mute_until_ms) >= 0) {
            s_muted = false;
            printf("EVT t=%lu TEST: CMD transmission resumed\n", (unsigned long)now);
        }
        if ((uint32_t)(now - last_cmd) >= CMD_PERIOD_MS) {
            last_cmd = now;
            if (!s_muted) {
                uint8_t f[PROTO_MAX_FRAME];
                const proto_cmd_t c = {.pump_on = s_ctl.out_pump, .valve_open = s_ctl.out_valve};
                ctl_on_cmd_sent(&s_ctl, s_cmd_seq, now);
                link_write(f, proto_encode_cmd(s_cmd_seq++, &c, f));
            }
            /* black box sample, one per CMD period */
            const proto_telem_t *t = &s_ctl.telem;
            const uint32_t age = ctl_link_age_ms(&s_ctl, now);
            const bb_sample_t smp = {
                .t_ms = now,
                .state = (uint8_t)s_ctl.state,
                .phase = (uint8_t)s_ctl.phase,
                .io = (uint8_t)((s_ctl.out_pump ? BB_IO_PUMP_CMD : 0u) |
                                (s_ctl.out_valve ? BB_IO_VALVE_CMD : 0u) |
                                (t->pump_running ? BB_IO_PUMP_ACT : 0u) |
                                (t->valve_open ? BB_IO_VALVE_ACT : 0u)),
                .flags = t->flags,
                .src_ml = t->src_ml,
                .dst_ml = t->dst_ml,
                .flow_ml_s = t->flow_ml_s,
                .link_age_ms = (uint16_t)(age > 65535u ? 65535u : age),
            };
            if (bb_add(&s_bb, &smp)) {
                fault_log_save_blackbox(&s_bb, ctl_fault_name((ctl_fault_t)s_bb.cause));
                bb_rearm(&s_bb);
            }
            if (s_stream) {
                printf("TLM t=%lu st=%s ph=%s fault=%s tgt=%u src=%u dst=%u flow=%u pc=%u pa=%u "
                       "vc=%u va=%u fl=%u age=%lu ack=%u rtt=%lu\n",
                       (unsigned long)now, ctl_state_name(s_ctl.state), ctl_phase_name(s_ctl.phase),
                       ctl_fault_name(s_ctl.fault), s_ctl.target_dst_ml, t->src_ml, t->dst_ml,
                       t->flow_ml_s, s_ctl.out_pump, t->pump_running, s_ctl.out_valve, t->valve_open,
                       t->flags, (unsigned long)(age > 99999u ? 99999u : age),
                       ctl_ack_ok(&s_ctl, now) ? 1u : 0u, (unsigned long)s_ctl.rtt.last_ms);
            }
        }
        if ((uint32_t)(now - last_hello) >= HELLO_PERIOD_MS && !s_muted) {
            last_hello = now;
            uint8_t f[PROTO_MAX_FRAME];
            link_write(f, proto_encode_hello(hello_seq++, &s_my_hello, f));
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
    const uint32_t boot = fault_log_init();
    bb_init(&s_bb);
    s_my_hello = (proto_hello_t){.proto_ver = PROTO_VERSION,
                                 .role = PROTO_ROLE_CONTROLLER,
                                 .boot_id = bench_boot_id(),
                                 .fw_id = proto_fnv1a(bench_fw_version())};

    ctl_config_t cfg;
    ctl_default_config(&cfg);
    ctl_init(&s_ctl, &cfg, bench_now_ms(), on_event, NULL);

    printf("EVT t=%lu BOOT controller (ESP32-S3) boot#%lu reset_reason=%s state=IDLE. "
           "Nothing resumes after reboot: send 'start' to begin.\n",
           (unsigned long)bench_now_ms(), (unsigned long)boot, bench_reset_reason_str());
    printf("VERSION %s | protocol v%u | boot_id %04x fw_id %08lx\n", bench_build_info(),
           PROTO_VERSION, s_my_hello.boot_id, (unsigned long)s_my_hello.fw_id);
    printf("config: min_src=%u dst_high=%u link_timeout=%lu valve_timeout=%lu flow_grace=%lu "
           "flow_min=%u stop_lead=%lu cmd_ack_timeout=%lu task_wdt=%ds\n",
           cfg.min_src_ml, cfg.dst_high_ml, (unsigned long)cfg.link_timeout_ms,
           (unsigned long)cfg.valve_timeout_ms, (unsigned long)cfg.flow_grace_ms, cfg.flow_min_ml_s,
           (unsigned long)cfg.stop_lead_ms, (unsigned long)cfg.cmd_ack_timeout_ms,
           CONFIG_ESP_TASK_WDT_TIMEOUT_S);
    fault_log_print();
    ota_boot_report();

    xTaskCreate(control_task, "control", 6144, NULL, 5, NULL);
}
