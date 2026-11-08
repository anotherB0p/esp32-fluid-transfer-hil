/*
 * Host unit tests for common/transfer_core (protocol, plant model, controller FSM).
 * Build and run: tests/run_tests.ps1   (gcc, -std=c99 -Wall -Wextra -Werror)
 *
 * The closed-loop tests connect the real controller FSM and the real plant model
 * through the real encoder/decoder, byte by byte, with a simulated 1 ms clock.
 * They test the logic only; they say nothing about UART drivers, timing jitter
 * or the boards themselves.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "controller_fsm.h"
#include "plant_model.h"
#include "proto.h"

static int g_checks, g_failed;
static const char *g_test;

#define CHECK(cond)                                                                        \
    do {                                                                                   \
        g_checks++;                                                                        \
        if (!(cond)) {                                                                     \
            g_failed++;                                                                    \
            printf("  FAIL %s:%d [%s]: %s\n", __FILE__, __LINE__, g_test, #cond);          \
        }                                                                                  \
    } while (0)

#define CHECK_EQ(a, b)                                                                     \
    do {                                                                                   \
        long _a = (long)(a), _b = (long)(b);                                               \
        g_checks++;                                                                        \
        if (_a != _b) {                                                                    \
            g_failed++;                                                                    \
            printf("  FAIL %s:%d [%s]: %s == %ld, expected %ld\n", __FILE__, __LINE__,     \
                   g_test, #a, _a, _b);                                                    \
        }                                                                                  \
    } while (0)

#define RUN(fn)                                                                            \
    do {                                                                                   \
        g_test = #fn;                                                                      \
        int _f = g_failed;                                                                 \
        fn();                                                                              \
        printf("%s %s\n", g_failed == _f ? "ok  " : "FAIL", #fn);                          \
    } while (0)

/* ------------------------------------------------------------------ protocol */

static int feed_all(proto_decoder_t *d, const uint8_t *b, size_t n, proto_msg_t *last)
{
    int got = 0;
    proto_msg_t m;
    for (size_t i = 0; i < n; i++) {
        if (proto_decoder_feed(d, b[i], &m)) {
            got++;
            if (last) *last = m;
        }
    }
    return got;
}

static void test_crc_known_vector(void)
{
    const uint8_t v[] = "123456789";
    CHECK_EQ(proto_crc16(v, 9), 0x29B1); /* CRC-16/CCITT-FALSE check value */
}

static void test_cmd_roundtrip(void)
{
    uint8_t f[PROTO_MAX_FRAME];
    proto_cmd_t c = {.pump_on = 1, .valve_open = 1};
    size_t n = proto_encode_cmd(42, &c, f);
    CHECK_EQ(n, 8);
    CHECK_EQ(f[0], PROTO_SOF);
    proto_decoder_t d;
    proto_decoder_init(&d);
    proto_msg_t m;
    CHECK_EQ(feed_all(&d, f, n, &m), 1);
    CHECK_EQ(m.type, MSG_CMD);
    CHECK_EQ(m.seq, 42);
    CHECK_EQ(m.u.cmd.pump_on, 1);
    CHECK_EQ(m.u.cmd.valve_open, 1);
}

static void test_telem_roundtrip(void)
{
    uint8_t f[PROTO_MAX_FRAME];
    proto_telem_t t = {.src_ml = 999, .dst_ml = 1000, .flow_ml_s = 50, .valve_open = 1,
                       .pump_running = 0, .cmd_seq_ack = 7, .flags = PLANT_FLAG_FAILSAFE};
    size_t n = proto_encode_telem(200, &t, f);
    CHECK_EQ(n, 16);
    proto_decoder_t d;
    proto_decoder_init(&d);
    proto_msg_t m;
    CHECK_EQ(feed_all(&d, f, n, &m), 1);
    CHECK_EQ(m.type, MSG_TELEM);
    CHECK_EQ(m.seq, 200);
    CHECK(memcmp(&m.u.telem, &t, sizeof(t)) == 0);
}

static void test_junk_and_resync(void)
{
    uint8_t buf[128];
    size_t n = 0;
    /* boot-log style junk, including stray SOF bytes */
    const char junk[] = "ESP-ROM:esp32c3\r\n\xA5\xFF\xA5\x01 rst:0x1";
    memcpy(buf, junk, sizeof(junk) - 1);
    n += sizeof(junk) - 1;
    proto_cmd_t c = {.pump_on = 0, .valve_open = 1};
    n += proto_encode_cmd(1, &c, &buf[n]);
    n += proto_encode_cmd(2, &c, &buf[n]);
    proto_decoder_t d;
    proto_decoder_init(&d);
    proto_msg_t m;
    CHECK_EQ(feed_all(&d, buf, n, &m), 2);
    CHECK_EQ(m.seq, 2);
    CHECK(d.stats.junk_bytes > 0);
}

static void test_corrupted_frame_rejected_next_ok(void)
{
    uint8_t buf[64];
    proto_cmd_t c = {.pump_on = 1, .valve_open = 1};
    size_t n1 = proto_encode_cmd(1, &c, buf);
    buf[5] ^= 0x01; /* flip a payload bit */
    size_t n2 = proto_encode_cmd(2, &c, &buf[n1]);
    proto_decoder_t d;
    proto_decoder_init(&d);
    proto_msg_t m;
    CHECK_EQ(feed_all(&d, buf, n1 + n2, &m), 1);
    CHECK_EQ(m.seq, 2);
    CHECK_EQ(d.stats.crc_errors, 1);
}

static void test_bad_length_rejected(void)
{
    const uint8_t bad[] = {PROTO_SOF, 0x01, PROTO_SOF, 0xFF};
    proto_decoder_t d;
    proto_decoder_init(&d);
    CHECK_EQ(feed_all(&d, bad, sizeof(bad), NULL), 0);
    CHECK_EQ(d.stats.len_errors, 2);
}

static void test_range_check_after_valid_crc(void)
{
    uint8_t f[PROTO_MAX_FRAME];
    proto_telem_t t = {.src_ml = 100, .dst_ml = 100};
    size_t n = proto_encode_telem(1, &t, f);
    f[4] = 0xFF; /* src = 0x00FF? set high byte to make it > capacity */
    f[5] = 0x7F;
    uint16_t crc = proto_crc16(&f[1], n - 3);
    f[n - 2] = (uint8_t)(crc & 0xFF);
    f[n - 1] = (uint8_t)(crc >> 8);
    proto_decoder_t d;
    proto_decoder_init(&d);
    CHECK_EQ(feed_all(&d, f, n, NULL), 0);
    CHECK_EQ(d.stats.range_errors, 1);

    proto_cmd_t c = {.pump_on = 1};
    n = proto_encode_cmd(1, &c, f);
    f[4] = 2; /* pump_on must be 0/1 */
    crc = proto_crc16(&f[1], n - 3);
    f[n - 2] = (uint8_t)(crc & 0xFF);
    f[n - 1] = (uint8_t)(crc >> 8);
    CHECK_EQ(feed_all(&d, f, n, NULL), 0);
    CHECK_EQ(d.stats.range_errors, 2);
}

static void test_wrong_length_for_type(void)
{
    /* MSG_CMD with a 3-byte payload: CRC valid, length wrong for the type */
    uint8_t f[] = {PROTO_SOF, 5, MSG_CMD, 9, 1, 1, 0, 0, 0};
    uint16_t crc = proto_crc16(&f[1], 6);
    f[7] = (uint8_t)(crc & 0xFF);
    f[8] = (uint8_t)(crc >> 8);
    proto_decoder_t d;
    proto_decoder_init(&d);
    CHECK_EQ(feed_all(&d, f, sizeof(f), NULL), 0);
    CHECK_EQ(d.stats.len_errors, 1);
}

/* ------------------------------------------------------------------ plant model */

static void step_ms(plant_t *p, uint32_t *now, uint32_t ms, const proto_cmd_t *cmd)
{
    for (uint32_t t = 0; t < ms; t += PLANT_DT_MS) {
        *now += PLANT_DT_MS;
        if (cmd) plant_on_cmd(p, cmd, 0, *now);
        plant_step(p, *now);
    }
}

/* A safe command arms the plant (required after boot and after failsafe). */
static void arm(plant_t *p, uint32_t now)
{
    const proto_cmd_t safe = {0, 0};
    plant_on_cmd(p, &safe, 0, now);
}

static void test_plant_ignores_run_cmd_until_armed(void)
{
    plant_t p;
    plant_init(&p, 800, 100, 0);
    uint32_t now = 0;
    proto_cmd_t on = {.pump_on = 1, .valve_open = 1};
    CHECK_EQ(plant_on_cmd(&p, &on, 1, 0), PLANT_CMD_REJECTED);
    step_ms(&p, &now, 1000, &on);
    CHECK(!p.pump_running);
    CHECK(!p.valve_open);
    proto_telem_t t;
    plant_fill_telem(&p, &t);
    CHECK(t.flags & PLANT_FLAG_NOT_ARMED);
    const proto_cmd_t safe = {0, 0};
    CHECK_EQ(plant_on_cmd(&p, &safe, 2, now), PLANT_CMD_ARMED);
    step_ms(&p, &now, 500, &on);
    CHECK(p.pump_running);
    CHECK(p.valve_open);
}

static void test_plant_disarms_after_failsafe(void)
{
    plant_t p;
    plant_init(&p, 800, 100, 0);
    uint32_t now = 0;
    arm(&p, now);
    proto_cmd_t on = {.pump_on = 1, .valve_open = 1};
    step_ms(&p, &now, 1000, &on);
    CHECK(p.pump_running);
    step_ms(&p, &now, 500, NULL); /* commands stop -> failsafe */
    CHECK(p.failsafe);
    CHECK(!p.armed);
    /* link comes back with a RUN command: must not resume on its own */
    CHECK_EQ(plant_on_cmd(&p, &on, 9, now), PLANT_CMD_REJECTED);
    step_ms(&p, &now, 500, &on);
    CHECK(!p.pump_running);
}

static void test_plant_boots_in_failsafe(void)
{
    plant_t p;
    plant_init(&p, 800, 100, 0);
    uint32_t now = 0;
    step_ms(&p, &now, 500, NULL);
    CHECK(p.failsafe);
    CHECK(!p.pump_running);
    CHECK(!p.valve_open);
    CHECK(p.flow_ml_s == 0.0f);
}

static void test_plant_transfers_with_valve_travel(void)
{
    plant_t p;
    plant_init(&p, 800, 100, 0);
    uint32_t now = 0;
    arm(&p, now);
    proto_cmd_t on = {.pump_on = 1, .valve_open = 1};
    step_ms(&p, &now, 200, &on);
    CHECK(!p.valve_open); /* 300 ms travel not finished */
    step_ms(&p, &now, 100, &on);
    CHECK(p.valve_open);
    step_ms(&p, &now, 2000, &on);
    CHECK(p.flow_ml_s > 45.0f);
    CHECK(fabsf((p.src_ml + p.dst_ml) - 900.0f) < 0.01f); /* mass conserved without leak */
}

static void test_plant_cmd_timeout_failsafe(void)
{
    plant_t p;
    plant_init(&p, 800, 100, 0);
    uint32_t now = 0;
    arm(&p, now);
    proto_cmd_t on = {.pump_on = 1, .valve_open = 1};
    step_ms(&p, &now, 1000, &on);
    CHECK(p.pump_running);
    CHECK(!p.failsafe);
    /* commands stop: failsafe must engage once age > 300 ms, i.e. at the 4th step */
    bool entered = false;
    uint32_t entered_after = 0;
    /* failsafe at +400 ms, valve travel 300 ms -> closed at +700 ms */
    for (uint32_t t = 100; t <= 800; t += 100) {
        now += 100;
        if (plant_step(&p, now) && !entered) {
            entered = true;
            entered_after = t;
        }
    }
    CHECK(entered);
    CHECK_EQ(entered_after, 400);
    CHECK(!p.pump_running);
    CHECK(!p.valve_open); /* commanded closed, travel done */
    CHECK(p.flow_ml_s == 0.0f);
}

static void test_plant_stuck_valve_and_dead_pump(void)
{
    plant_t p;
    plant_init(&p, 800, 100, 0);
    uint32_t now = 0;
    p.fault_valve_stuck = true;
    p.fault_pump_dead = true;
    arm(&p, now);
    proto_cmd_t on = {.pump_on = 1, .valve_open = 1};
    step_ms(&p, &now, 2000, &on);
    CHECK(!p.valve_open);
    CHECK(!p.pump_running);
    CHECK(p.valve_cmd); /* commanded differs from actual */
    CHECK(p.flow_ml_s == 0.0f);
}

static void test_plant_runs_dry(void)
{
    plant_t p;
    plant_init(&p, 30, 100, 0);
    uint32_t now = 0;
    arm(&p, now);
    proto_cmd_t on = {.pump_on = 1, .valve_open = 1};
    step_ms(&p, &now, 5000, &on);
    CHECK(p.src_ml == 0.0f);
    CHECK(p.flow_ml_s == 0.0f);
    CHECK(fabsf(p.dst_ml - 130.0f) < 0.01f);
}

/* ------------------------------------------------------------------ closed loop */

typedef struct {
    ctl_t ctl;
    plant_t plant;
    proto_decoder_t to_plant, to_ctl;
    uint32_t now;
    uint8_t cmd_seq, tel_seq;
    bool drop_cmd;   /* controller -> plant wire cut */
    bool drop_telem; /* plant -> controller wire cut */
    int events;
    ctl_fault_t last_cause;
} loop_t;

static void on_event(void *ctx, ctl_state_t from, ctl_state_t to, ctl_fault_t cause,
                     const char *detail)
{
    loop_t *l = ctx;
    l->events++;
    l->last_cause = cause;
    (void)from;
    (void)to;
    (void)detail;
}

static void loop_init(loop_t *l, float src, float dst)
{
    memset(l, 0, sizeof(*l));
    ctl_config_t cfg;
    ctl_default_config(&cfg);
    ctl_init(&l->ctl, &cfg, 0, on_event, l);
    plant_init(&l->plant, src, dst, 0);
    proto_decoder_init(&l->to_plant);
    proto_decoder_init(&l->to_ctl);
}

/* Advance 1 ms at a time: FSM tick every 10 ms, CMD/TELEM/model step every 100 ms. */
static void loop_run(loop_t *l, uint32_t ms)
{
    uint8_t f[PROTO_MAX_FRAME];
    proto_msg_t m;
    for (uint32_t i = 0; i < ms; i++) {
        l->now++;
        if (l->now % 100 == 0) {
            proto_cmd_t c = {.pump_on = l->ctl.out_pump, .valve_open = l->ctl.out_valve};
            size_t n = proto_encode_cmd(l->cmd_seq++, &c, f);
            for (size_t k = 0; k < n && !l->drop_cmd; k++) {
                if (proto_decoder_feed(&l->to_plant, f[k], &m) && m.type == MSG_CMD) {
                    plant_on_cmd(&l->plant, &m.u.cmd, m.seq, l->now);
                }
            }
        }
        if (l->now % 100 == 50) {
            plant_step(&l->plant, l->now);
            proto_telem_t t;
            plant_fill_telem(&l->plant, &t);
            size_t n = proto_encode_telem(l->tel_seq++, &t, f);
            for (size_t k = 0; k < n && !l->drop_telem; k++) {
                if (proto_decoder_feed(&l->to_ctl, f[k], &m) && m.type == MSG_TELEM) {
                    ctl_on_telem(&l->ctl, &m.u.telem, l->now);
                }
            }
        }
        if (l->now % 10 == 0) {
            ctl_tick(&l->ctl, l->now);
        }
    }
}

/* Run until the controller reaches `state` or `limit_ms` elapses; returns elapsed ms. */
static uint32_t loop_until(loop_t *l, ctl_state_t state, uint32_t limit_ms)
{
    uint32_t start = l->now;
    while (l->ctl.state != state && l->now - start < limit_ms) {
        loop_run(l, 1);
    }
    return l->now - start;
}

static void test_loop_happy_path(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000); /* link up, plant leaves failsafe */
    CHECK(ctl_link_ok(&l.ctl, l.now));
    CHECK(!l.plant.failsafe);
    const char *why = NULL;
    CHECK(ctl_start(&l.ctl, 500, l.now, &why));
    uint32_t t = loop_until(&l, CTL_COMPLETE, 20000);
    CHECK_EQ(l.ctl.state, CTL_COMPLETE);
    /* 400 mL at 50 mL/s ~ 8 s plus valve travel and ramp */
    CHECK(t > 8000 && t < 11000);
    CHECK(l.plant.dst_ml >= 500.0f && l.plant.dst_ml < 515.0f);
    CHECK(!l.plant.pump_running);
    CHECK(!l.plant.valve_open);
    CHECK(fabsf(l.plant.src_ml + l.plant.dst_ml - 900.0f) < 0.01f);
    /* a second transfer may start from COMPLETE */
    CHECK(ctl_start(&l.ctl, 600, l.now, &why));
    loop_until(&l, CTL_COMPLETE, 20000);
    CHECK_EQ(l.ctl.state, CTL_COMPLETE);
}

static void test_loop_insufficient_source(void)
{
    loop_t l;
    loop_init(&l, 300, 100); /* needs 400 + 100 reserve */
    loop_run(&l, 1000);
    const char *why;
    CHECK(ctl_start(&l.ctl, 500, l.now, &why));
    loop_until(&l, CTL_FAULT, 2000);
    CHECK_EQ(l.ctl.fault, F_INSUFFICIENT_SOURCE);
    CHECK(!l.plant.pump_running);
}

static void test_loop_stuck_valve_closed(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    l.plant.fault_valve_stuck = true;
    const char *why;
    ctl_start(&l.ctl, 500, l.now, &why);
    uint32_t t = loop_until(&l, CTL_FAULT, 5000);
    CHECK_EQ(l.ctl.fault, F_VALVE_NOT_OPEN);
    CHECK(t >= 1000 && t <= 1300);
    CHECK(!l.plant.pump_running); /* pump was never started */
    loop_run(&l, 300);
    CHECK(ctl_reset(&l.ctl, l.now, &why)); /* valve is closed: reset allowed */
}

static void test_loop_dead_pump_no_flow(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    l.plant.fault_pump_dead = true;
    const char *why;
    ctl_start(&l.ctl, 500, l.now, &why);
    loop_until(&l, CTL_FAULT, 5000);
    CHECK_EQ(l.ctl.fault, F_NO_FLOW);
    CHECK(!l.ctl.out_pump);
    CHECK(!l.ctl.out_valve);
}

static void test_loop_source_low_by_leak(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 3000);
    CHECK_EQ(l.ctl.state, CTL_TRANSFER);
    l.plant.leak_ml_s = 150.0f;
    loop_until(&l, CTL_FAULT, 10000);
    CHECK_EQ(l.ctl.fault, F_SOURCE_LOW);
    loop_run(&l, 1000);
    CHECK(!l.plant.pump_running);
}

static void test_loop_telemetry_lost(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 3000);
    CHECK_EQ(l.ctl.state, CTL_TRANSFER);
    l.drop_telem = true;
    uint32_t t = loop_until(&l, CTL_FAULT, 2000);
    CHECK_EQ(l.ctl.fault, F_LINK_LOST);
    /* last telemetry is up to 100 ms old when the cut happens; timeout 300 ms; tick 10 ms */
    CHECK(t >= 200 && t <= 420);
    /* controller now commands safe state; plant obeys because commands still arrive */
    loop_run(&l, 500);
    CHECK(!l.plant.pump_running);
    /* link restored: no auto-resume, fault stays latched until reset */
    l.drop_telem = false;
    loop_run(&l, 2000);
    CHECK_EQ(l.ctl.state, CTL_FAULT);
    CHECK(ctl_reset(&l.ctl, l.now, &why));
    CHECK_EQ(l.ctl.state, CTL_IDLE);
}

static void test_loop_commands_lost_plant_failsafe(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 3000);
    CHECK(l.plant.pump_running);
    l.drop_cmd = true;
    uint32_t start = l.now;
    while (!l.plant.failsafe && l.now - start < 2000) loop_run(&l, 1);
    CHECK(l.plant.failsafe);
    CHECK(!l.plant.pump_running); /* plant stopped itself */
    CHECK(l.now - start <= 450);
    loop_until(&l, CTL_FAULT, 1000);
    CHECK_EQ(l.ctl.fault, F_PLANT_NO_CMD);
    /* reset refused while plant still cannot hear us */
    loop_run(&l, 500);
    CHECK(!ctl_reset(&l.ctl, l.now, &why));
    l.drop_cmd = false;
    loop_run(&l, 1000);
    CHECK(ctl_reset(&l.ctl, l.now, &why));
}

static void test_loop_valve_stuck_open_on_stop(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 300, l.now, &why);
    loop_run(&l, 2000);
    CHECK_EQ(l.ctl.phase, PH_PUMPING);
    l.plant.fault_valve_stuck = true; /* stuck in OPEN position */
    loop_until(&l, CTL_FAULT, 10000);
    CHECK_EQ(l.ctl.fault, F_VALVE_NOT_CLOSED);
    loop_run(&l, 500);
    CHECK(!ctl_reset(&l.ctl, l.now, &why)); /* valve still open: refused */
    CHECK(why != NULL && strstr(why, "valve") != NULL);
    l.plant.fault_valve_stuck = false;       /* "repaired" */
    loop_run(&l, 1000);
    CHECK(ctl_reset(&l.ctl, l.now, &why));
}

static void test_loop_operator_stop_and_start_refused_in_fault(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 2000);
    ctl_stop(&l.ctl, l.now);
    CHECK_EQ(l.ctl.state, CTL_FAULT);
    CHECK_EQ(l.ctl.fault, F_OPERATOR_STOP);
    CHECK(!ctl_start(&l.ctl, 600, l.now, &why));
    loop_run(&l, 1000);
    CHECK(ctl_reset(&l.ctl, l.now, &why));
    CHECK(ctl_start(&l.ctl, 600, l.now, &why));
}

static void test_loop_start_without_link(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    l.drop_telem = true;
    loop_run(&l, 500);
    const char *why;
    CHECK(ctl_start(&l.ctl, 500, l.now, &why));
    uint32_t t = loop_until(&l, CTL_FAULT, 3000);
    CHECK_EQ(l.ctl.fault, F_LINK_LOST);
    CHECK(t > 1000 && t < 1100);
    CHECK(!l.plant.pump_running);
}

static void test_controller_reboot_does_not_resume(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 3000);
    CHECK(l.plant.pump_running);
    /* controller reboot: fresh FSM state, link keeps running */
    ctl_config_t cfg;
    ctl_default_config(&cfg);
    ctl_init(&l.ctl, &cfg, l.now, on_event, &l);
    loop_run(&l, 1000);
    CHECK_EQ(l.ctl.state, CTL_IDLE);
    CHECK(!l.plant.pump_running); /* IDLE commands pump off */
    CHECK(!l.plant.valve_open);
}

/* Regression for the bench finding: the C3 reboots in ~120 ms, faster than the 300 ms
 * link timeout, so the controller never saw a link gap and the fresh plant obeyed the
 * still-running RUN command. */
static void test_loop_fast_plant_reboot_mid_transfer(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 3000);
    CHECK(l.plant.pump_running);
    /* reboot: state lost, sources back to defaults, link gap shorter than the timeout */
    plant_init(&l.plant, 800, 100, l.now);
    bool pump_ran_after_reboot = false;
    uint32_t start = l.now;
    while (l.ctl.state != CTL_FAULT && l.now - start < 2000) {
        loop_run(&l, 1);
        pump_ran_after_reboot |= l.plant.pump_running;
    }
    CHECK_EQ(l.ctl.fault, F_PLANT_RESTARTED);
    CHECK(!pump_ran_after_reboot);
    loop_run(&l, 1000);
    CHECK(!l.plant.pump_running);
    CHECK(ctl_reset(&l.ctl, l.now, &why)); /* FAULT sends safe commands -> plant re-armed */
}

int main(void)
{
    RUN(test_crc_known_vector);
    RUN(test_cmd_roundtrip);
    RUN(test_telem_roundtrip);
    RUN(test_junk_and_resync);
    RUN(test_corrupted_frame_rejected_next_ok);
    RUN(test_bad_length_rejected);
    RUN(test_range_check_after_valid_crc);
    RUN(test_wrong_length_for_type);
    RUN(test_plant_ignores_run_cmd_until_armed);
    RUN(test_plant_disarms_after_failsafe);
    RUN(test_plant_boots_in_failsafe);
    RUN(test_plant_transfers_with_valve_travel);
    RUN(test_plant_cmd_timeout_failsafe);
    RUN(test_plant_stuck_valve_and_dead_pump);
    RUN(test_plant_runs_dry);
    RUN(test_loop_happy_path);
    RUN(test_loop_insufficient_source);
    RUN(test_loop_stuck_valve_closed);
    RUN(test_loop_dead_pump_no_flow);
    RUN(test_loop_source_low_by_leak);
    RUN(test_loop_telemetry_lost);
    RUN(test_loop_commands_lost_plant_failsafe);
    RUN(test_loop_valve_stuck_open_on_stop);
    RUN(test_loop_operator_stop_and_start_refused_in_fault);
    RUN(test_loop_start_without_link);
    RUN(test_controller_reboot_does_not_resume);
    RUN(test_loop_fast_plant_reboot_mid_transfer);
    printf("\n%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
