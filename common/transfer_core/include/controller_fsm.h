/*
 * Transfer controller state machine (non-blocking, tick driven).
 *
 *   IDLE --start--> CHECK --ok--> TRANSFER --target reached--> COMPLETE
 *                     |              |  (OPEN_VALVE -> PUMPING -> STOPPING)
 *                     +---- any failed check / supervision -------> FAULT (latched)
 *
 * - Outputs (pump/valve commands) are a pure function of state + phase: everything
 *   except TRANSFER commands pump OFF and valve CLOSED.
 * - FAULT stores the first cause and a detail string; it is left only by an explicit
 *   reset, and only when the plant is observed in a safe state over a working link
 *   (fresh telemetry, plant not in failsafe, pump not running, valve closed).
 * - Nothing resumes automatically: after a reboot the controller starts in IDLE, and
 *   link recovery while in FAULT does not clear the fault.
 * - All timing uses the caller's monotonic millisecond clock (wrap-safe arithmetic).
 *
 * Pure C99, no platform dependencies.
 */
#ifndef CONTROLLER_FSM_H
#define CONTROLLER_FSM_H

#include <stdbool.h>
#include <stdint.h>

#include "proto.h"

typedef enum {
    CTL_IDLE = 0,
    CTL_CHECK,
    CTL_TRANSFER,
    CTL_COMPLETE,
    CTL_FAULT,
} ctl_state_t;

typedef enum {
    PH_NONE = 0,
    PH_OPEN_VALVE, /* valve commanded open, pump off, waiting for valve feedback */
    PH_PUMPING,    /* pump on, supervising flow and levels */
    PH_STOPPING,   /* pump off, valve commanded closed, waiting for feedback */
} ctl_phase_t;

typedef enum {
    F_NONE = 0,
    F_LINK_LOST,            /* no valid telemetry within link_timeout_ms */
    F_PLANT_NO_CMD,         /* plant reports failsafe: it is not getting our commands */
    F_PLANT_NOT_SAFE,       /* CHECK: pump running or valve open before start */
    F_TARGET_REACHED,       /* CHECK: destination already at/above target */
    F_INSUFFICIENT_SOURCE,  /* CHECK: source cannot supply target + reserve */
    F_VALVE_NOT_OPEN,       /* valve did not report open in time (or closed while pumping) */
    F_NO_FLOW,              /* pump commanded, flow below minimum after grace period */
    F_SOURCE_LOW,           /* source dropped to the reserve level during transfer */
    F_OVERFILL,             /* destination above high limit */
    F_VALVE_NOT_CLOSED,     /* valve did not report closed when stopping */
    F_OPERATOR_STOP,        /* operator 'stop' during CHECK/TRANSFER */
    F_PLANT_NOT_ARMED,      /* plant recovered from failsafe during transfer (needs re-arming) */
    F_PLANT_RESTARTED,      /* plant reports it rebooted during transfer */
    F_CMD_NOT_ACKED,        /* telemetry flows but does not acknowledge recent commands */
} ctl_fault_t;

typedef struct {
    uint16_t min_src_ml;        /* reserve left in source, pump must not run dry */
    uint16_t dst_high_ml;       /* destination high limit (overfill interlock) */
    uint32_t link_timeout_ms;   /* telemetry older than this = link lost */
    uint32_t check_timeout_ms;  /* CHECK waits this long for a working link */
    uint32_t valve_timeout_ms;  /* valve feedback deadline */
    uint32_t flow_grace_ms;     /* time allowed for flow to build up */
    uint16_t flow_min_ml_s;     /* minimum acceptable flow while pumping */
    uint32_t stop_lead_ms;      /* stop early: dst + flow * lead >= target (latency + valve travel) */
    uint32_t cmd_ack_timeout_ms;/* newest acknowledged CMD must have been sent within this */
} ctl_config_t;

#define CTL_SENT_RING 16u

typedef struct {
    uint32_t last_ms, min_ms, max_ms, sum_ms, count;
} ctl_rtt_t;

void ctl_default_config(ctl_config_t *cfg);

typedef void (*ctl_event_cb_t)(void *ctx, ctl_state_t from, ctl_state_t to, ctl_fault_t cause,
                               const char *detail);

typedef struct {
    ctl_config_t cfg;
    ctl_state_t state;
    ctl_phase_t phase;
    uint32_t state_since_ms;
    uint32_t phase_since_ms;

    uint16_t target_dst_ml;

    bool have_telem;
    uint32_t last_telem_ms;
    proto_telem_t telem;

    ctl_fault_t fault;
    char fault_detail[72];

    bool out_pump;
    bool out_valve;

    /* command acknowledgement supervision */
    struct {
        uint8_t seq;
        bool valid;
        uint32_t sent_ms;
    } sent[CTL_SENT_RING];
    uint8_t sent_next;
    bool have_ack;
    uint32_t last_acked_sent_ms; /* send time of the newest CMD the plant acknowledged */
    ctl_rtt_t rtt;

    ctl_event_cb_t on_event;
    void *cb_ctx;
} ctl_t;

void ctl_init(ctl_t *c, const ctl_config_t *cfg, uint32_t now_ms, ctl_event_cb_t cb, void *ctx);

/* Feed a validated telemetry frame. */
void ctl_on_telem(ctl_t *c, const proto_telem_t *t, uint32_t now_ms);

/* Record that a CMD with this SEQ was handed to the link at now_ms. */
void ctl_on_cmd_sent(ctl_t *c, uint8_t seq, uint32_t now_ms);

/* True if a recently sent command has been acknowledged (see cmd_ack_timeout_ms). */
bool ctl_ack_ok(const ctl_t *c, uint32_t now_ms);

/* Operator commands. Return false and set *why when refused. */
bool ctl_start(ctl_t *c, uint16_t target_dst_ml, uint32_t now_ms, const char **why);
void ctl_stop(ctl_t *c, uint32_t now_ms);
bool ctl_reset(ctl_t *c, uint32_t now_ms, const char **why);

/* Periodic evaluation (call every 10..100 ms). Updates out_pump / out_valve. */
void ctl_tick(ctl_t *c, uint32_t now_ms);

bool ctl_link_ok(const ctl_t *c, uint32_t now_ms);
uint32_t ctl_link_age_ms(const ctl_t *c, uint32_t now_ms);

const char *ctl_state_name(ctl_state_t s);
const char *ctl_phase_name(ctl_phase_t p);
const char *ctl_fault_name(ctl_fault_t f);

#endif /* CONTROLLER_FSM_H */
