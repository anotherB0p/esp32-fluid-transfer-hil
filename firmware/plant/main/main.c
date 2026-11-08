/*
 * Plant simulator firmware - ESP32-C3 (board A09).
 *
 * Simulates two tanks, a pump and a valve (common/transfer_core/plant_model.c), receives
 * CMD frames from the controller and answers with TELEM every 100 ms. Faults are
 * injected here, on the plant's own console, because they belong to the simulated
 * physical world; the controller learns about them only through telemetry.
 *
 * If no valid CMD arrives for 300 ms the model itself goes to the safe state (pump off,
 * valve commanded closed). It does not wait for the controller to notice.
 *
 * Link: UART1 TX=GPIO3 RX=GPIO4 115200 8N1 (see docs/hardware.md).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "bench_io.h"
#include "oled.h"
#include "plant_model.h"
#include "proto.h"

#define LINK_TX_GPIO 3
#define LINK_RX_GPIO 4
#define LINK_BAUD 115200
#define LOOP_MS 10u
#define STAT_PERIOD_MS 1000u
#define INIT_SRC_ML 800.0f
#define INIT_DST_ML 100.0f

static plant_t s_plant;
static proto_decoder_t s_dec;
static uint8_t s_tel_seq;
static uint32_t s_mute_until_ms;
static bool s_muted; /* test hook: stop sending TELEM and ignore CMD */
static uint32_t s_cmd_count, s_cmd_seq_gaps;
static bool s_have_seq;
static uint8_t s_last_seq;
static bool s_reject_logged;
static proto_hello_t s_my_hello;
#define HELLO_PERIOD_MS 500u

static void print_status(uint32_t now)
{
    const plant_t *p = &s_plant;
    printf("STAT peer ctl_hello=%s ctl_proto=v%u | self fw=%s boot_id=%04x proto=v%u%s | "
           "sim: dst_frozen=%u dst_inflow=%.0f\n",
           p->have_ctl_hello ? (p->ctl_compatible ? "OK" : "INCOMPATIBLE") : "NONE",
           p->ctl_proto_ver, bench_fw_version(), s_my_hello.boot_id, s_my_hello.proto_ver,
           s_my_hello.proto_ver != PROTO_VERSION ? " (FAKED by 'proto' test hook)" : "",
           p->fault_dst_frozen, (double)p->dst_inflow_ml_s);
    printf("STAT t=%lu src=%.1f dst=%.1f flow=%.1f | pump cmd=%u act=%u valve cmd=%u act=%u | "
           "failsafe=%u armed=%u cmd_age=%ld | faults: valve_stuck=%u pump_dead=%u leak=%.0f | "
           "rx_ok=%lu crc_err=%lu len_err=%lu range_err=%lu junk=%lu seq_gaps=%lu "
           "tx_noise_ppm=%lu tx_corrupted=%lu%s\n",
           (unsigned long)now, (double)p->src_ml, (double)p->dst_ml, (double)p->flow_ml_s,
           p->pump_cmd, p->pump_running, p->valve_cmd, p->valve_open, p->failsafe, p->armed,
           p->have_cmd ? (long)(now - p->last_cmd_ms) : -1L, p->fault_valve_stuck,
           p->fault_pump_dead, (double)p->leak_ml_s, (unsigned long)s_dec.stats.frames_ok,
           (unsigned long)s_dec.stats.crc_errors, (unsigned long)s_dec.stats.len_errors,
           (unsigned long)s_dec.stats.range_errors, (unsigned long)s_dec.stats.junk_bytes,
           (unsigned long)s_cmd_seq_gaps, (unsigned long)link_noise_ppm(),
           (unsigned long)link_corrupted_bytes(), s_muted ? " LINK_MUTED" : "");
}

static bool parse_flag(const char *s, bool *out)
{
    if (!s) return false;
    if (strcmp(s, "1") == 0 || strcmp(s, "on") == 0) { *out = true; return true; }
    if (strcmp(s, "0") == 0 || strcmp(s, "off") == 0) { *out = false; return true; }
    return false;
}

static void handle_line(char *line, uint32_t now)
{
    printf("> %s\n", line);
    char *cmd = strtok(line, " ");
    char *a1 = strtok(NULL, " ");
    char *a2 = strtok(NULL, " ");

    if (strcmp(cmd, "set") == 0 && a1 && a2) {
        long v = strtol(a2, NULL, 10);
        if (v < 0 || v > (long)PROTO_TANK_CAPACITY_ML) {
            printf("ERR level must be 0..%u mL\n", PROTO_TANK_CAPACITY_ML);
        } else if (strcmp(a1, "src") == 0) {
            s_plant.src_ml = (float)v;
            printf("EVT t=%lu SIM: source level set to %ld mL\n", (unsigned long)now, v);
        } else if (strcmp(a1, "dst") == 0) {
            s_plant.dst_ml = (float)v;
            printf("EVT t=%lu SIM: destination level set to %ld mL\n", (unsigned long)now, v);
        } else {
            printf("ERR set src|dst <mL>\n");
        }
    } else if (strcmp(cmd, "fault") == 0 && a1) {
        bool on = false;
        if (strcmp(a1, "clear") == 0) {
            s_plant.fault_valve_stuck = false;
            s_plant.fault_pump_dead = false;
            s_plant.leak_ml_s = 0.0f;
            s_plant.dst_inflow_ml_s = 0.0f;
            s_plant.fault_dst_frozen = false;
            printf("EVT t=%lu SIM: all injected faults cleared\n", (unsigned long)now);
        } else if (strcmp(a1, "dst_frozen") == 0 && parse_flag(a2, &on)) {
            s_plant.fault_dst_frozen = on;
            s_plant.dst_frozen_value = s_plant.dst_ml;
            printf("EVT t=%lu SIM: dst_frozen=%u (destination sensor stuck at %.0f mL)\n",
                   (unsigned long)now, on, (double)s_plant.dst_frozen_value);
        } else if (strcmp(a1, "dst_inflow") == 0 && a2) {
            long v = strtol(a2, NULL, 10);
            s_plant.dst_inflow_ml_s = (float)(v < 0 ? 0 : (v > 500 ? 500 : v));
            printf("EVT t=%lu SIM: external inflow into destination %.0f mL/s\n", (unsigned long)now,
                   (double)s_plant.dst_inflow_ml_s);
        } else if (strcmp(a1, "valve_stuck") == 0 && parse_flag(a2, &on)) {
            s_plant.fault_valve_stuck = on;
            printf("EVT t=%lu SIM: valve_stuck=%u (valve frozen %s)\n", (unsigned long)now, on,
                   s_plant.valve_open ? "OPEN" : "CLOSED");
        } else if (strcmp(a1, "pump_dead") == 0 && parse_flag(a2, &on)) {
            s_plant.fault_pump_dead = on;
            printf("EVT t=%lu SIM: pump_dead=%u\n", (unsigned long)now, on);
        } else if (strcmp(a1, "leak") == 0 && a2) {
            long v = strtol(a2, NULL, 10);
            s_plant.leak_ml_s = (float)(v < 0 ? 0 : (v > 500 ? 500 : v));
            printf("EVT t=%lu SIM: source leak %.0f mL/s\n", (unsigned long)now, (double)s_plant.leak_ml_s);
        } else {
            printf("ERR fault valve_stuck|pump_dead|dst_frozen <0|1> | fault leak|dst_inflow <mL/s> "
                   "| fault clear\n");
        }
    } else if (strcmp(cmd, "proto") == 0 && a1) {
        long v = strtol(a1, NULL, 10);
        s_my_hello.proto_ver = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
        printf("EVT t=%lu TEST: advertising protocol v%u in HELLO (real v%u)\n", (unsigned long)now,
               s_my_hello.proto_ver, PROTO_VERSION);
    } else if (strcmp(cmd, "version") == 0) {
        printf("VERSION %s | protocol v%u | boot_id %04x\n", bench_build_info(), PROTO_VERSION,
               s_my_hello.boot_id);
    } else if (strcmp(cmd, "mute") == 0) {
        long ms = a1 ? strtol(a1, NULL, 10) : 2000;
        s_mute_until_ms = now + (uint32_t)(ms > 0 ? ms : 0);
        s_muted = ms > 0;
        printf("EVT t=%lu TEST: link muted for %ld ms (no TELEM sent, CMD ignored)\n",
               (unsigned long)now, ms);
    } else if (strcmp(cmd, "status") == 0) {
        print_status(now);
    } else if (strcmp(cmd, "noise") == 0) {
        long ppm = a1 ? strtol(a1, NULL, 10) : 0;
        link_set_noise_ppm((uint32_t)(ppm > 0 ? ppm : 0));
        printf("EVT t=%lu TEST: TELEM TX bit-flip noise %lu ppm per byte\n", (unsigned long)now,
               (unsigned long)link_noise_ppm());
    } else if (strcmp(cmd, "hang") == 0) {
        printf("EVT t=%lu TEST: plant task hangs now (watchdog must reset the board)\n",
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
        printf("commands: set src|dst <mL> | fault valve_stuck|pump_dead|dst_frozen <0|1> | "
               "fault leak|dst_inflow <mL/s> | fault clear | mute <ms> | noise <ppm> | "
               "proto <n> | hang | status | version | reboot\n");
    } else {
        printf("ERR unknown command '%s' (try 'help')\n", cmd);
    }
}

static void plant_task(void *arg)
{
    (void)arg;
    uint8_t rx[64];
    char line[80];
    proto_msg_t msg;
    uint32_t last_step = bench_now_ms(), last_stat = 0;
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
                if (!proto_decoder_feed(&s_dec, rx[i], &msg) || s_muted) {
                    continue;
                }
                if (msg.type == MSG_HELLO) {
                    const bool had = s_plant.have_ctl_hello, was_ok = s_plant.ctl_compatible;
                    const bool ok = plant_on_hello(&s_plant, &msg.u.hello);
                    if (msg.u.hello.role == PROTO_ROLE_CONTROLLER && (!had || ok != was_ok)) {
                        printf("EVT t=%lu HELLO from controller: protocol v%u boot_id=%04x -> %s\n",
                               (unsigned long)now, msg.u.hello.proto_ver, msg.u.hello.boot_id,
                               ok ? "compatible" : "INCOMPATIBLE, will not arm");
                    }
                    continue;
                }
                if (msg.type == MSG_CMD) {
                    if (s_have_seq && msg.seq != (uint8_t)(s_last_seq + 1u)) {
                        s_cmd_seq_gaps++;
                    }
                    s_have_seq = true;
                    s_last_seq = msg.seq;
                    s_cmd_count++;
                    const bool was_failsafe = s_plant.failsafe;
                    const plant_cmd_result_t r = plant_on_cmd(&s_plant, &msg.u.cmd, msg.seq, now);
                    if (was_failsafe) {
                        printf("EVT t=%lu link: valid CMD received, failsafe cleared\n",
                               (unsigned long)now);
                    }
                    if (r == PLANT_CMD_ARMED) {
                        printf("EVT t=%lu ARMED: safe command received, run commands accepted\n",
                               (unsigned long)now);
                        s_reject_logged = false;
                    } else if (r == PLANT_CMD_NO_PEER && !s_reject_logged) {
                        s_reject_logged = true;
                        printf("EVT t=%lu NOT_ARMED: no compatible controller HELLO yet, staying "
                               "safe\n",
                               (unsigned long)now);
                    } else if (r == PLANT_CMD_REJECTED && !s_reject_logged) {
                        s_reject_logged = true;
                        printf("EVT t=%lu NOT_ARMED: run command (pump=%u valve=%u) IGNORED until a "
                               "safe command arrives\n",
                               (unsigned long)now, msg.u.cmd.pump_on, msg.u.cmd.valve_open);
                    }
                }
            }
        }

        while (console_poll_line(line, sizeof(line))) {
            handle_line(line, now);
        }

        if (s_muted && (int32_t)(now - s_mute_until_ms) >= 0) {
            s_muted = false;
            printf("EVT t=%lu TEST: link unmuted\n", (unsigned long)now);
        }

        if ((uint32_t)(now - last_step) >= PLANT_DT_MS) {
            last_step += PLANT_DT_MS;
            const uint32_t cmd_age = now - s_plant.last_cmd_ms;
            if (plant_step(&s_plant, now)) {
                printf("EVT t=%lu FAILSAFE: no valid CMD for %lu ms -> pump OFF, valve commanded "
                       "CLOSED (plant-side protection)\n",
                       (unsigned long)now, (unsigned long)cmd_age);
            }
            if (!s_muted) {
                proto_telem_t t;
                plant_fill_telem(&s_plant, &t);
                uint8_t f[PROTO_MAX_FRAME];
                link_write(f, proto_encode_telem(s_tel_seq++, &t, f));
                if ((uint32_t)(now - last_hello) >= HELLO_PERIOD_MS) {
                    last_hello = now;
                    link_write(f, proto_encode_hello(hello_seq++, &s_my_hello, f));
                }
            }
            const oled_view_t v = {
                .src_ml = (uint16_t)(s_plant.src_ml + 0.5f),
                .dst_ml = (uint16_t)(s_plant.dst_ml + 0.5f),
                .flow_ml_s = (uint16_t)(s_plant.flow_ml_s + 0.5f),
                .valve_open = s_plant.valve_open,
                .pump_running = s_plant.pump_running,
                .failsafe = s_plant.failsafe,
                .fault_injected = s_plant.fault_valve_stuck || s_plant.fault_pump_dead ||
                                  s_plant.leak_ml_s > 0.0f || s_plant.dst_inflow_ml_s > 0.0f ||
                                  s_plant.fault_dst_frozen,
            };
            oled_set_view(&v);
        }

        if ((uint32_t)(now - last_stat) >= STAT_PERIOD_MS) {
            last_stat = now;
            print_status(now);
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
    }
}

void app_main(void)
{
    console_init();
    link_init(LINK_TX_GPIO, LINK_RX_GPIO, LINK_BAUD);
    proto_decoder_init(&s_dec);
    plant_init(&s_plant, INIT_SRC_ML, INIT_DST_ML, bench_now_ms());
    s_my_hello = (proto_hello_t){.proto_ver = PROTO_VERSION,
                                 .role = PROTO_ROLE_PLANT,
                                 .boot_id = bench_boot_id(),
                                 .fw_id = proto_fnv1a(bench_fw_version())};
    printf("VERSION %s | protocol v%u | boot_id %04x fw_id %08lx\n", bench_build_info(),
           PROTO_VERSION, s_my_hello.boot_id, (unsigned long)s_my_hello.fw_id);
    const bool oled = oled_init();
    printf("EVT t=%lu BOOT plant simulator (ESP32-C3) reset_reason=%s: src=%.0f dst=%.0f mL, "
           "pump OFF, valve CLOSED, NOT_ARMED until a safe CMD. OLED %s.\n",
           (unsigned long)bench_now_ms(), bench_reset_reason_str(), (double)INIT_SRC_ML,
           (double)INIT_DST_ML, oled ? "ok" : "NOT FOUND (running without display)");
    xTaskCreate(plant_task, "plant", 4096, NULL, 5, NULL);
}
