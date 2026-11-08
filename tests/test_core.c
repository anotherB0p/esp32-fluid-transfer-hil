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
#include "blackbox.h"

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

/* Every 1-bit and every 2-bit error in a TELEM frame must be rejected.
 * CRC-16/CCITT detects all such errors for frames this short; the test proves the
 * decoder does not accept them through some other path (e.g. a corrupted LEN). */
static void test_all_1_and_2_bit_errors_rejected(void)
{
    uint8_t good[PROTO_MAX_FRAME];
    proto_telem_t t = {.src_ml = 512, .dst_ml = 300, .flow_ml_s = 49, .valve_open = 1,
                       .pump_running = 1, .cmd_seq_ack = 77, .flags = 0};
    const size_t n = proto_encode_telem(5, &t, good);
    const size_t bits = n * 8u;
    long accepted = 0, tried = 0;
    for (size_t a = 0; a < bits; a++) {
        for (size_t b = a; b < bits; b++) {
            uint8_t f[PROTO_MAX_FRAME + 1];
            memcpy(f, good, n);
            f[a / 8] ^= (uint8_t)(1u << (a % 8));
            if (b != a) f[b / 8] ^= (uint8_t)(1u << (b % 8));
            proto_decoder_t d;
            proto_decoder_init(&d);
            accepted += feed_all(&d, f, n, NULL);
            tried++;
        }
    }
    printf("    %ld corrupted frames (all 1- and 2-bit errors), %ld accepted\n", tried, accepted);
    CHECK_EQ(accepted, 0);
}

/* Regression for the bench finding (noisy_link): one flipped bit anywhere in a frame
 * must cost exactly that frame. Before the fix a corrupted LEN (e.g. 12 -> 28) made the
 * decoder wait for a long body and swallow the next, intact frame too: three frames lost
 * from a single bit = LINK_LOST on the real wire at 0.5 % byte noise. */
static void test_single_bit_error_loses_only_that_frame(void)
{
    proto_telem_t t = {.src_ml = 700, .dst_ml = 200, .flow_ml_s = 50, .valve_open = 1,
                       .pump_running = 1, .cmd_seq_ack = 3};
    uint8_t one[PROTO_MAX_FRAME];
    const size_t n = proto_encode_telem(0, &t, one);
    int worst_lost = 0;
    for (size_t bit = 0; bit < n * 8u; bit++) {
        uint8_t stream[4 * PROTO_MAX_FRAME];
        size_t len = 0;
        for (uint8_t s = 0; s < 4; s++) {
            len += proto_encode_telem(s, &t, &stream[len]);
        }
        stream[n + bit / 8] ^= (uint8_t)(1u << (bit % 8)); /* corrupt frame #1 only */
        proto_decoder_t d;
        proto_decoder_init(&d);
        int got = feed_all(&d, stream, len, NULL);
        int lost = 4 - got;
        if (lost > worst_lost) worst_lost = lost;
    }
    printf("    worst case: %d frame(s) lost per single-bit error\n", worst_lost);
    CHECK_EQ(worst_lost, 1);
}

/* Simple xorshift PRNG so the fuzz run is reproducible. */
static uint32_t s_rng = 0x12345678u;
static uint32_t rnd(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

/* 2 MB of random bytes with valid frames mixed in: the decoder must never crash, must
 * recover every valid frame that is preceded by a clean SOF boundary, and false accepts
 * from pure noise must be rare (CRC-16 + length + range checks). */
static void test_decoder_fuzz(void)
{
    proto_decoder_t d;
    proto_decoder_init(&d);
    long good_sent = 0, good_got = 0, false_accepts = 0;
    proto_msg_t m;
    uint8_t f[PROTO_MAX_FRAME];
    for (int block = 0; block < 20000; block++) {
        /* ~100 noise bytes */
        int noise = (int)(rnd() % 200u);
        for (int i = 0; i < noise; i++) {
            if (proto_decoder_feed(&d, (uint8_t)rnd(), &m)) false_accepts++;
        }
        /* flush any partial frame the noise may have started: a long run of non-SOF bytes
         * is not enough (a started frame swallows them), so use the known max frame size */
        for (int i = 0; i < (int)PROTO_MAX_FRAME; i++) {
            if (proto_decoder_feed(&d, 0x00, &m)) false_accepts++;
        }
        proto_cmd_t c = {.pump_on = (uint8_t)(rnd() & 1u), .valve_open = (uint8_t)(rnd() & 1u)};
        size_t n = proto_encode_cmd((uint8_t)block, &c, f);
        good_sent++;
        for (size_t k = 0; k < n; k++) {
            if (proto_decoder_feed(&d, f[k], &m)) {
                if (m.type == MSG_CMD && m.seq == (uint8_t)block) good_got++;
                else false_accepts++;
            }
        }
    }
    printf("    fuzz: %ld valid frames sent, %ld recovered, %ld false accepts from noise "
           "(crc_err=%lu len_err=%lu range_err=%lu)\n",
           good_sent, good_got, false_accepts, (unsigned long)d.stats.crc_errors,
           (unsigned long)d.stats.len_errors, (unsigned long)d.stats.range_errors);
    CHECK_EQ(good_got, good_sent);
    CHECK(false_accepts <= 2);
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

static const proto_hello_t CTL_HELLO = {.proto_ver = PROTO_VERSION,
                                         .role = PROTO_ROLE_CONTROLLER,
                                         .boot_id = 0x1111,
                                         .fw_id = 0xC0FFEEu};

/* A compatible controller HELLO plus a safe command arms the plant (required after boot
 * and after failsafe). */
static void arm(plant_t *p, uint32_t now)
{
    const proto_cmd_t safe = {0, 0};
    plant_on_hello(p, &CTL_HELLO);
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
    CHECK(t.flags & PLANT_FLAG_PEER_BAD);
    const proto_cmd_t safe = {0, 0};
    /* safe command but no controller HELLO yet: stays unarmed */
    CHECK_EQ(plant_on_cmd(&p, &safe, 2, now), PLANT_CMD_NO_PEER);
    /* incompatible controller: still unarmed */
    proto_hello_t bad = CTL_HELLO;
    bad.proto_ver = (uint8_t)(PROTO_VERSION + 7u);
    CHECK(!plant_on_hello(&p, &bad));
    CHECK_EQ(plant_on_cmd(&p, &safe, 3, now), PLANT_CMD_NO_PEER);
    CHECK(plant_on_hello(&p, &CTL_HELLO));
    CHECK_EQ(plant_on_cmd(&p, &safe, 4, now), PLANT_CMD_ARMED);
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
    bool ack_frozen; /* plant bug: telemetry keeps reporting an old cmd_seq_ack */
    uint8_t frozen_ack;
    proto_hello_t ctl_hello, plant_hello;
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
    l->ctl_hello = CTL_HELLO;
    l->plant_hello = (proto_hello_t){.proto_ver = PROTO_VERSION, .role = PROTO_ROLE_PLANT,
                                     .boot_id = 0x2222, .fw_id = 0xBEEFu};
}

/* Advance 1 ms at a time: FSM tick every 10 ms, CMD/TELEM/model step every 100 ms. */
static void loop_run(loop_t *l, uint32_t ms)
{
    uint8_t f[PROTO_MAX_FRAME];
    proto_msg_t m;
    for (uint32_t i = 0; i < ms; i++) {
        l->now++;
        if (l->now % 500 == 20) { /* HELLO both ways every 500 ms */
            size_t n = proto_encode_hello(0, &l->ctl_hello, f);
            for (size_t k = 0; k < n && !l->drop_cmd; k++) {
                if (proto_decoder_feed(&l->to_plant, f[k], &m) && m.type == MSG_HELLO) {
                    plant_on_hello(&l->plant, &m.u.hello);
                }
            }
            n = proto_encode_hello(0, &l->plant_hello, f);
            for (size_t k = 0; k < n && !l->drop_telem; k++) {
                if (proto_decoder_feed(&l->to_ctl, f[k], &m) && m.type == MSG_HELLO) {
                    ctl_on_hello(&l->ctl, &m.u.hello, l->now);
                }
            }
        }
        if (l->now % 100 == 0) {
            proto_cmd_t c = {.pump_on = l->ctl.out_pump, .valve_open = l->ctl.out_valve};
            ctl_on_cmd_sent(&l->ctl, l->cmd_seq, l->now);
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
            if (l->ack_frozen) {
                t.cmd_seq_ack = l->frozen_ack;
            }
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
    /* with stop anticipation the final level lands within +-3 mL of the target */
    printf("    happy path: final dst %.1f mL for target 500\n", (double)l.plant.dst_ml);
    CHECK(l.plant.dst_ml >= 497.0f && l.plant.dst_ml <= 503.0f);
    /* command round trip: CMD at t%100==0, TELEM at t%100==50 -> 50 ms in this simulation */
    CHECK(l.ctl.rtt.count > 50);
    CHECK_EQ(l.ctl.rtt.min_ms, 50);
    CHECK_EQ(l.ctl.rtt.max_ms, 50);
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

/* Independent detector: telemetry is fresh and the plant claims to be fine, but it no
 * longer acknowledges our commands (e.g. its command handling is stuck). */
static void test_loop_ack_frozen(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 2000);
    CHECK_EQ(l.ctl.phase, PH_PUMPING);
    l.ack_frozen = true;
    l.frozen_ack = l.plant.cmd_seq_ack;
    uint32_t t = loop_until(&l, CTL_FAULT, 2000);
    CHECK_EQ(l.ctl.fault, F_CMD_NOT_ACKED);
    CHECK(t >= 300 && t <= 520);
    loop_run(&l, 500);
    CHECK(!ctl_reset(&l.ctl, l.now, &why)); /* still not acknowledged */
    l.ack_frozen = false;
    loop_run(&l, 500);
    CHECK(ctl_reset(&l.ctl, l.now, &why));
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

/* ------------------------------------------------------------------ v2.1 additions */

static void test_hello_roundtrip_and_role_check(void)
{
    uint8_t f[PROTO_MAX_FRAME];
    proto_hello_t h = {.proto_ver = 99, .role = PROTO_ROLE_PLANT, .boot_id = 0xBEEF,
                       .fw_id = 0x12345678u};
    size_t n = proto_encode_hello(7, &h, f);
    CHECK_EQ(n, 14);
    proto_decoder_t d;
    proto_decoder_init(&d);
    proto_msg_t m;
    /* a foreign protocol version is still decoded: the application must see it */
    CHECK_EQ(feed_all(&d, f, n, &m), 1);
    CHECK_EQ(m.type, MSG_HELLO);
    CHECK(memcmp(&m.u.hello, &h, sizeof(h)) == 0);
    h.role = 3; /* unknown role is a range error */
    n = proto_encode_hello(8, &h, f);
    CHECK_EQ(feed_all(&d, f, n, NULL), 0);
    CHECK_EQ(d.stats.range_errors, 1);
    CHECK(proto_fnv1a("abc") != proto_fnv1a("abd"));
}

static void test_loop_plant_speaks_other_protocol(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    l.plant_hello.proto_ver = (uint8_t)(PROTO_VERSION + 1u);
    loop_run(&l, 1000);
    CHECK(!ctl_peer_ok(&l.ctl));
    const char *why = NULL;
    CHECK(!ctl_start(&l.ctl, 500, l.now, &why)); /* refused up front */
    CHECK(why != NULL && strstr(why, "incompatible") != NULL);
    CHECK_EQ(l.ctl.state, CTL_IDLE);
}

static void test_loop_controller_speaks_other_protocol(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    l.ctl_hello.proto_ver = (uint8_t)(PROTO_VERSION + 1u);
    loop_run(&l, 1000);
    CHECK(!l.plant.armed); /* plant refuses to arm for an incompatible controller */
    const char *why;
    CHECK(ctl_start(&l.ctl, 500, l.now, &why));
    loop_until(&l, CTL_FAULT, 2000);
    CHECK_EQ(l.ctl.fault, F_PEER_INCOMPATIBLE);
    CHECK(!l.plant.pump_running);
}

static void test_loop_reboot_seen_via_hello_boot_id(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 2000);
    CHECK_EQ(l.ctl.state, CTL_TRANSFER);
    /* same plant state, but it announces a new boot_id (as after a reboot) */
    l.plant_hello.boot_id = 0x3333;
    loop_until(&l, CTL_FAULT, 1000);
    CHECK_EQ(l.ctl.fault, F_PLANT_RESTARTED);
    CHECK_EQ(l.ctl.peer_reboots, 1);
}

static void test_loop_overfill_in_idle(void)
{
    loop_t l;
    loop_init(&l, 100, 900);
    loop_run(&l, 1000);
    CHECK_EQ(l.ctl.state, CTL_IDLE);
    l.plant.dst_inflow_ml_s = 100.0f; /* liquid arrives from outside the transfer */
    uint32_t t = loop_until(&l, CTL_FAULT, 3000);
    CHECK_EQ(l.ctl.fault, F_OVERFILL);
    CHECK(t > 400 && t < 700); /* 900 -> 951 mL at 100 mL/s */
    const char *why = NULL;
    l.plant.dst_inflow_ml_s = 0.0f;
    loop_run(&l, 300);
    CHECK(!ctl_reset(&l.ctl, l.now, &why));
    CHECK(why != NULL && strstr(why, "high limit") != NULL);
    l.plant.dst_ml = 800.0f; /* drained */
    loop_run(&l, 300);
    CHECK(ctl_reset(&l.ctl, l.now, &why));
}

static void test_loop_dst_sensor_frozen(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 3000);
    CHECK_EQ(l.ctl.phase, PH_PUMPING);
    l.plant.fault_dst_frozen = true;
    l.plant.dst_frozen_value = l.plant.dst_ml;
    uint32_t t = loop_until(&l, CTL_FAULT, 3000);
    CHECK_EQ(l.ctl.fault, F_SENSOR_IMPLAUSIBLE);
    /* measured from the last CHANGE of the reported value, which can precede the freeze
     * by up to one telemetry period */
    printf("    frozen sensor detected %lu ms after the freeze\n", (unsigned long)t);
    CHECK(t >= 850 && t <= 1200);
    loop_run(&l, 500);
    CHECK(!l.plant.pump_running);
}

static void test_loop_dst_sensor_jump(void)
{
    loop_t l;
    loop_init(&l, 800, 100);
    loop_run(&l, 1000);
    const char *why;
    ctl_start(&l.ctl, 600, l.now, &why);
    loop_run(&l, 3000);
    l.plant.dst_ml += 200.0f; /* sensor step (physically impossible in 100 ms) */
    uint32_t t = loop_until(&l, CTL_FAULT, 500);
    CHECK_EQ(l.ctl.fault, F_SENSOR_IMPLAUSIBLE);
    CHECK(t <= 110);
}

static void test_blackbox_pre_and_post_trigger(void)
{
    blackbox_t bb;
    bb_init(&bb);
    bb_sample_t s = {0};
    for (uint32_t i = 0; i < 100; i++) { /* wrap the ring several times */
        s.t_ms = i * 100u;
        CHECK(!bb_add(&bb, &s));
    }
    CHECK_EQ(bb_count(&bb), BB_LEN);
    CHECK_EQ(bb_at(&bb, 0)->t_ms, (100u - BB_LEN) * 100u); /* oldest */
    CHECK_EQ(bb_at(&bb, BB_LEN - 1)->t_ms, 9900u);       /* newest */
    bb_trigger(&bb, 7, 9950);
    bb_trigger(&bb, 9, 9960); /* second trigger ignored */
    bool done = false;
    uint32_t added = 0;
    while (!done) {
        s.t_ms += 100u;
        done = bb_add(&bb, &s);
        added++;
    }
    CHECK_EQ(added, BB_POST);
    CHECK_EQ(bb.cause, 7);
    /* frozen: more samples are ignored, the snapshot stays intact */
    const uint32_t newest = bb_at(&bb, BB_LEN - 1)->t_ms;
    s.t_ms = 123456u;
    CHECK(!bb_add(&bb, &s));
    CHECK_EQ(bb_at(&bb, BB_LEN - 1)->t_ms, newest);
    /* snapshot = (BB_LEN - BB_POST) samples before the trigger + BB_POST after */
    uint32_t before = 0;
    for (uint32_t i = 0; i < BB_LEN; i++) {
        if (bb_at(&bb, i)->t_ms < 9950u) before++;
    }
    CHECK_EQ(before, BB_LEN - BB_POST);
    bb_rearm(&bb);
    CHECK(!bb_add(&bb, &s));
    CHECK_EQ(bb_at(&bb, BB_LEN - 1)->t_ms, 123456u);
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
    RUN(test_all_1_and_2_bit_errors_rejected);
    RUN(test_single_bit_error_loses_only_that_frame);
    RUN(test_decoder_fuzz);
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
    RUN(test_loop_ack_frozen);
    RUN(test_hello_roundtrip_and_role_check);
    RUN(test_loop_plant_speaks_other_protocol);
    RUN(test_loop_controller_speaks_other_protocol);
    RUN(test_loop_reboot_seen_via_hello_boot_id);
    RUN(test_loop_overfill_in_idle);
    RUN(test_loop_dst_sensor_frozen);
    RUN(test_loop_dst_sensor_jump);
    RUN(test_blackbox_pre_and_post_trigger);
    printf("\n%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
