#include "controller_fsm.h"

#include <stdio.h>
#include <string.h>

void ctl_default_config(ctl_config_t *cfg)
{
    cfg->min_src_ml = 100;
    cfg->dst_high_ml = 950;
    cfg->link_timeout_ms = 300;   /* 3 telemetry periods */
    cfg->check_timeout_ms = 1000;
    cfg->valve_timeout_ms = 1000; /* model travel time is 300 ms */
    cfg->flow_grace_ms = 1500;
    cfg->flow_min_ml_s = 20;      /* nominal is 50 mL/s */
    /* Measured on the bench without anticipation: 9 mL overshoot at 50 mL/s, i.e. ~180 ms
     * of telemetry age + CMD period + model step + valve travel with decaying flow. */
    cfg->stop_lead_ms = 180;
    /* Backup detector for one-way loss; the plant's own failsafe (300 ms) normally fires
     * first. 400 ms = 4 CMD periods. */
    cfg->cmd_ack_timeout_ms = 400;
    /* Max legitimate change per 100 ms frame is 5 mL at nominal flow; 20 mL leaves margin
     * for jitter (two frames merged) without accepting a sensor step. */
    cfg->dst_jump_ml = 20;
    cfg->dst_frozen_ms = 1000;
}

const char *ctl_state_name(ctl_state_t s)
{
    switch (s) {
    case CTL_IDLE: return "IDLE";
    case CTL_CHECK: return "CHECK";
    case CTL_TRANSFER: return "TRANSFER";
    case CTL_COMPLETE: return "COMPLETE";
    case CTL_FAULT: return "FAULT";
    }
    return "?";
}

const char *ctl_phase_name(ctl_phase_t p)
{
    switch (p) {
    case PH_NONE: return "-";
    case PH_OPEN_VALVE: return "OPEN_VALVE";
    case PH_PUMPING: return "PUMPING";
    case PH_STOPPING: return "STOPPING";
    }
    return "?";
}

const char *ctl_fault_name(ctl_fault_t f)
{
    switch (f) {
    case F_NONE: return "NONE";
    case F_LINK_LOST: return "LINK_LOST";
    case F_PLANT_NO_CMD: return "PLANT_NO_CMD";
    case F_PLANT_NOT_SAFE: return "PLANT_NOT_SAFE";
    case F_TARGET_REACHED: return "TARGET_REACHED";
    case F_INSUFFICIENT_SOURCE: return "INSUFFICIENT_SOURCE";
    case F_VALVE_NOT_OPEN: return "VALVE_NOT_OPEN";
    case F_NO_FLOW: return "NO_FLOW";
    case F_SOURCE_LOW: return "SOURCE_LOW";
    case F_OVERFILL: return "OVERFILL";
    case F_VALVE_NOT_CLOSED: return "VALVE_NOT_CLOSED";
    case F_OPERATOR_STOP: return "OPERATOR_STOP";
    case F_PLANT_NOT_ARMED: return "PLANT_NOT_ARMED";
    case F_PLANT_RESTARTED: return "PLANT_RESTARTED";
    case F_CMD_NOT_ACKED: return "CMD_NOT_ACKED";
    case F_PEER_INCOMPATIBLE: return "PEER_INCOMPATIBLE";
    case F_SENSOR_IMPLAUSIBLE: return "SENSOR_IMPLAUSIBLE";
    }
    return "?";
}

static void apply_outputs(ctl_t *c)
{
    c->out_pump = false;
    c->out_valve = false;
    if (c->state == CTL_TRANSFER) {
        c->out_valve = (c->phase == PH_OPEN_VALVE || c->phase == PH_PUMPING);
        c->out_pump = (c->phase == PH_PUMPING);
    }
}

static void enter(ctl_t *c, ctl_state_t to, ctl_phase_t ph, uint32_t now, const char *detail)
{
    const ctl_state_t from = c->state;
    c->state = to;
    c->phase = ph;
    c->state_since_ms = now;
    c->phase_since_ms = now;
    apply_outputs(c);
    if (c->on_event) {
        c->on_event(c->cb_ctx, from, to, c->fault, detail);
    }
}

static void set_phase(ctl_t *c, ctl_phase_t ph, uint32_t now, const char *detail)
{
    c->phase = ph;
    c->phase_since_ms = now;
    apply_outputs(c);
    if (c->on_event) {
        c->on_event(c->cb_ctx, c->state, c->state, F_NONE, detail);
    }
}

static void fault(ctl_t *c, ctl_fault_t cause, uint32_t now, const char *detail)
{
    if (c->state == CTL_FAULT) {
        return; /* keep the first cause */
    }
    c->fault = cause;
    snprintf(c->fault_detail, sizeof(c->fault_detail), "%s", detail);
    enter(c, CTL_FAULT, PH_NONE, now, detail);
}

void ctl_init(ctl_t *c, const ctl_config_t *cfg, uint32_t now_ms, ctl_event_cb_t cb, void *ctx)
{
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    c->state = CTL_IDLE;
    c->phase = PH_NONE;
    c->state_since_ms = now_ms;
    c->phase_since_ms = now_ms;
    c->on_event = cb;
    c->cb_ctx = ctx;
    apply_outputs(c);
}

void ctl_on_telem(ctl_t *c, const proto_telem_t *t, uint32_t now_ms)
{
    /* Destination sensor plausibility: a step larger than physically possible between two
     * frames is a sensor fault (only judged while transferring; levels are set by hand
     * on this bench while idle). */
    if (c->have_prev_dst && t->dst_ml != c->prev_dst_ml) {
        const uint16_t jump = (t->dst_ml > c->prev_dst_ml) ? (uint16_t)(t->dst_ml - c->prev_dst_ml)
                                                           : (uint16_t)(c->prev_dst_ml - t->dst_ml);
        /* A rebooted / re-arming plant restarts its simulation from initial levels: that
         * is reported by the flags and handled as PLANT_RESTARTED, not as a sensor fault. */
        const bool plant_reset =
            (t->flags & (PLANT_FLAG_RESTARTED | PLANT_FLAG_NOT_ARMED | PLANT_FLAG_FAILSAFE)) != 0u;
        if (c->state == CTL_TRANSFER && !plant_reset && jump > c->cfg.dst_jump_ml) {
            char d[72];
            snprintf(d, sizeof(d), "dst jumped %u -> %u mL in one frame (max %u)", c->prev_dst_ml,
                     t->dst_ml, c->cfg.dst_jump_ml);
            fault(c, F_SENSOR_IMPLAUSIBLE, now_ms, d);
        }
        c->dst_changed_ms = now_ms;
    }
    if (!c->have_prev_dst) {
        c->dst_changed_ms = now_ms;
    }
    c->have_prev_dst = true;
    c->prev_dst_ml = t->dst_ml;

    c->telem = *t;
    c->have_telem = true;
    c->last_telem_ms = now_ms;

    /* Match the acknowledged SEQ against the most recent sends (8-bit SEQ wraps). */
    for (uint32_t i = 1; i <= CTL_SENT_RING; i++) {
        const uint32_t idx = (c->sent_next + CTL_SENT_RING - i) % CTL_SENT_RING;
        if (c->sent[idx].valid && c->sent[idx].seq == t->cmd_seq_ack) {
            const uint32_t sent = c->sent[idx].sent_ms;
            if (!c->have_ack || (int32_t)(sent - c->last_acked_sent_ms) > 0) {
                const uint32_t rtt = now_ms - sent;
                c->rtt.last_ms = rtt;
                if (c->rtt.count == 0 || rtt < c->rtt.min_ms) c->rtt.min_ms = rtt;
                if (rtt > c->rtt.max_ms) c->rtt.max_ms = rtt;
                c->rtt.sum_ms += rtt;
                c->rtt.count++;
                c->last_acked_sent_ms = sent;
                c->have_ack = true;
            }
            break;
        }
    }
}

bool ctl_peer_ok(const ctl_t *c)
{
    return c->have_peer && c->peer.role == PROTO_ROLE_PLANT && c->peer.proto_ver == PROTO_VERSION;
}

ctl_hello_result_t ctl_on_hello(ctl_t *c, const proto_hello_t *h, uint32_t now_ms)
{
    const bool active = (c->state == CTL_CHECK || c->state == CTL_TRANSFER);
    const bool first = !c->have_peer;
    const bool rebooted = c->have_peer && h->boot_id != c->peer.boot_id;
    c->peer = *h;
    c->have_peer = true;

    if (!ctl_peer_ok(c)) {
        if (active) {
            char d[72];
            snprintf(d, sizeof(d), "HELLO: peer role %u protocol v%u, expected plant v%u", h->role,
                     h->proto_ver, PROTO_VERSION);
            fault(c, F_PEER_INCOMPATIBLE, now_ms, d);
        }
        return CTL_HELLO_INCOMPATIBLE;
    }
    if (rebooted) {
        c->peer_reboots++;
        if (active) {
            fault(c, F_PLANT_RESTARTED, now_ms, "HELLO: plant boot_id changed (rebooted)");
        }
        return CTL_HELLO_PEER_REBOOT;
    }
    return first ? CTL_HELLO_FIRST : CTL_HELLO_SAME;
}

void ctl_on_cmd_sent(ctl_t *c, uint8_t seq, uint32_t now_ms)
{
    c->sent[c->sent_next].seq = seq;
    c->sent[c->sent_next].sent_ms = now_ms;
    c->sent[c->sent_next].valid = true;
    c->sent_next = (uint8_t)((c->sent_next + 1u) % CTL_SENT_RING);
}

bool ctl_ack_ok(const ctl_t *c, uint32_t now_ms)
{
    return c->have_ack && (uint32_t)(now_ms - c->last_acked_sent_ms) <= c->cfg.cmd_ack_timeout_ms;
}

uint32_t ctl_link_age_ms(const ctl_t *c, uint32_t now_ms)
{
    return c->have_telem ? (uint32_t)(now_ms - c->last_telem_ms) : UINT32_MAX;
}

bool ctl_link_ok(const ctl_t *c, uint32_t now_ms)
{
    return c->have_telem && (uint32_t)(now_ms - c->last_telem_ms) <= c->cfg.link_timeout_ms;
}

bool ctl_start(ctl_t *c, uint16_t target_dst_ml, uint32_t now_ms, const char **why)
{
    if (c->state != CTL_IDLE && c->state != CTL_COMPLETE) {
        *why = (c->state == CTL_FAULT) ? "in FAULT: 'reset' first" : "already running";
        return false;
    }
    if (target_dst_ml == 0u || target_dst_ml > c->cfg.dst_high_ml) {
        *why = "target out of range (1..dst_high)";
        return false;
    }
    if (c->have_peer && !ctl_peer_ok(c)) {
        *why = "plant is incompatible (protocol version / role from HELLO)";
        return false;
    }
    c->target_dst_ml = target_dst_ml;
    c->fault = F_NONE;
    c->fault_detail[0] = '\0';
    enter(c, CTL_CHECK, PH_NONE, now_ms, "start requested");
    return true;
}

void ctl_stop(ctl_t *c, uint32_t now_ms)
{
    if (c->state == CTL_CHECK || c->state == CTL_TRANSFER) {
        fault(c, F_OPERATOR_STOP, now_ms, "operator stop");
    }
}

bool ctl_reset(ctl_t *c, uint32_t now_ms, const char **why)
{
    if (c->state != CTL_FAULT && c->state != CTL_COMPLETE) {
        *why = "nothing to reset";
        return false;
    }
    if (!ctl_link_ok(c, now_ms)) {
        *why = "link down: no fresh telemetry";
        return false;
    }
    if (c->telem.flags & (PLANT_FLAG_FAILSAFE | PLANT_FLAG_NOT_ARMED)) {
        *why = "plant still in failsafe / not armed (not receiving our commands)";
        return false;
    }
    if (!ctl_ack_ok(c, now_ms)) {
        *why = "our commands are not acknowledged by the plant";
        return false;
    }
    if (!ctl_peer_ok(c) || (c->telem.flags & PLANT_FLAG_PEER_BAD)) {
        *why = "no compatible HELLO exchange with the plant";
        return false;
    }
    if (c->telem.dst_ml > c->cfg.dst_high_ml) {
        *why = "destination still above the high limit";
        return false;
    }
    if (c->telem.pump_running) {
        *why = "pump still running";
        return false;
    }
    if (c->telem.valve_open) {
        *why = "valve still open";
        return false;
    }
    c->fault = F_NONE;
    c->fault_detail[0] = '\0';
    enter(c, CTL_IDLE, PH_NONE, now_ms, "reset accepted");
    return true;
}

static void tick_check(ctl_t *c, uint32_t now)
{
    char d[72];
    const proto_telem_t *t = &c->telem;
    const uint32_t in_state = now - c->state_since_ms;

    if (!ctl_link_ok(c, now) || !ctl_ack_ok(c, now) || !ctl_peer_ok(c) ||
        (t->flags & (PLANT_FLAG_FAILSAFE | PLANT_FLAG_NOT_ARMED | PLANT_FLAG_PEER_BAD))) {
        if (in_state > c->cfg.check_timeout_ms) {
            if (!ctl_link_ok(c, now)) {
                fault(c, F_LINK_LOST, now, "CHECK: no fresh telemetry");
            } else if (!ctl_ack_ok(c, now)) {
                fault(c, F_CMD_NOT_ACKED, now, "CHECK: commands not acknowledged");
            } else if (!c->have_peer) {
                fault(c, F_PEER_INCOMPATIBLE, now, "CHECK: no HELLO from the plant");
            } else if (!ctl_peer_ok(c)) {
                snprintf(d, sizeof(d), "CHECK: plant speaks protocol v%u, expected v%u",
                         c->peer.proto_ver, PROTO_VERSION);
                fault(c, F_PEER_INCOMPATIBLE, now, d);
            } else if (t->flags & PLANT_FLAG_PEER_BAD) {
                fault(c, F_PEER_INCOMPATIBLE, now, "CHECK: plant has no compatible HELLO from us");
            } else {
                fault(c, F_PLANT_NO_CMD, now, "CHECK: plant in failsafe / not armed");
            }
        }
        return; /* wait for a working link */
    }
    if (t->pump_running || t->valve_open) {
        snprintf(d, sizeof(d), "CHECK: pump=%u valve=%u before start", t->pump_running, t->valve_open);
        fault(c, F_PLANT_NOT_SAFE, now, d);
        return;
    }
    if (t->dst_ml >= c->target_dst_ml) {
        snprintf(d, sizeof(d), "CHECK: dst %u >= target %u mL", t->dst_ml, c->target_dst_ml);
        fault(c, F_TARGET_REACHED, now, d);
        return;
    }
    const uint32_t need = (uint32_t)(c->target_dst_ml - t->dst_ml) + c->cfg.min_src_ml;
    if (t->src_ml < need) {
        snprintf(d, sizeof(d), "CHECK: src %u < needed %lu mL (incl. reserve %u)", t->src_ml,
                 (unsigned long)need, c->cfg.min_src_ml);
        fault(c, F_INSUFFICIENT_SOURCE, now, d);
        return;
    }
    snprintf(d, sizeof(d), "CHECK ok: src %u dst %u target %u mL", t->src_ml, t->dst_ml,
             c->target_dst_ml);
    enter(c, CTL_TRANSFER, PH_OPEN_VALVE, now, d);
}

static void tick_transfer(ctl_t *c, uint32_t now)
{
    char d[72];
    const proto_telem_t *t = &c->telem;
    const uint32_t in_phase = now - c->phase_since_ms;

    if (t->flags & PLANT_FLAG_RESTARTED) {
        fault(c, F_PLANT_RESTARTED, now, "plant reports RESTARTED (rebooted during transfer)");
        return;
    }
    if (t->flags & PLANT_FLAG_FAILSAFE) {
        fault(c, F_PLANT_NO_CMD, now, "plant entered failsafe (commands not received)");
        return;
    }
    if (t->flags & PLANT_FLAG_PEER_BAD) {
        fault(c, F_PEER_INCOMPATIBLE, now, "plant reports no compatible controller HELLO");
        return;
    }
    if (t->flags & PLANT_FLAG_NOT_ARMED) {
        fault(c, F_PLANT_NOT_ARMED, now, "plant reports NOT_ARMED (recovered from failsafe)");
        return;
    }
    if (!ctl_ack_ok(c, now)) {
        snprintf(d, sizeof(d), "newest acknowledged CMD was sent %lu ms ago",
                 c->have_ack ? (unsigned long)(now - c->last_acked_sent_ms) : 0ul);
        fault(c, F_CMD_NOT_ACKED, now, d);
        return;
    }
    if (t->dst_ml > c->cfg.dst_high_ml) {
        snprintf(d, sizeof(d), "dst %u > high limit %u mL", t->dst_ml, c->cfg.dst_high_ml);
        fault(c, F_OVERFILL, now, d);
        return;
    }

    switch (c->phase) {
    case PH_OPEN_VALVE:
        if (t->valve_open) {
            snprintf(d, sizeof(d), "valve open confirmed after %lu ms", (unsigned long)in_phase);
            set_phase(c, PH_PUMPING, now, d);
        } else if (in_phase > c->cfg.valve_timeout_ms) {
            snprintf(d, sizeof(d), "valve commanded OPEN, actual CLOSED after %lu ms",
                     (unsigned long)in_phase);
            fault(c, F_VALVE_NOT_OPEN, now, d);
        }
        break;

    case PH_PUMPING:
        if (!t->valve_open) {
            fault(c, F_VALVE_NOT_OPEN, now, "valve reported CLOSED while pumping");
        } else if (t->dst_ml + (uint32_t)t->flow_ml_s * c->cfg.stop_lead_ms / 1000u >=
                   c->target_dst_ml) {
            snprintf(d, sizeof(d), "target reached: dst %u + flow %u x %lu ms >= %u mL", t->dst_ml,
                     t->flow_ml_s, (unsigned long)c->cfg.stop_lead_ms, c->target_dst_ml);
            set_phase(c, PH_STOPPING, now, d);
        } else if (t->src_ml <= c->cfg.min_src_ml) {
            snprintf(d, sizeof(d), "src %u <= reserve %u mL", t->src_ml, c->cfg.min_src_ml);
            fault(c, F_SOURCE_LOW, now, d);
        } else if (in_phase > c->cfg.flow_grace_ms && t->flow_ml_s < c->cfg.flow_min_ml_s) {
            snprintf(d, sizeof(d), "flow %u < %u mL/s, pump actual=%u", t->flow_ml_s,
                     c->cfg.flow_min_ml_s, t->pump_running);
            fault(c, F_NO_FLOW, now, d);
        } else if (t->flow_ml_s >= c->cfg.flow_min_ml_s &&
                   (uint32_t)(now - c->dst_changed_ms) > c->cfg.dst_frozen_ms &&
                   in_phase > c->cfg.dst_frozen_ms) {
            snprintf(d, sizeof(d), "dst frozen at %u mL for %lu ms while flow %u mL/s", t->dst_ml,
                     (unsigned long)(now - c->dst_changed_ms), t->flow_ml_s);
            fault(c, F_SENSOR_IMPLAUSIBLE, now, d);
        }
        break;

    case PH_STOPPING:
        if (!t->valve_open && !t->pump_running) {
            snprintf(d, sizeof(d), "stopped: dst %u mL, src %u mL", t->dst_ml, t->src_ml);
            enter(c, CTL_COMPLETE, PH_NONE, now, d);
        } else if (in_phase > c->cfg.valve_timeout_ms) {
            snprintf(d, sizeof(d), "valve commanded CLOSED, actual OPEN after %lu ms",
                     (unsigned long)in_phase);
            fault(c, F_VALVE_NOT_CLOSED, now, d);
        }
        break;

    case PH_NONE:
    default:
        break;
    }
}

void ctl_tick(ctl_t *c, uint32_t now_ms)
{
    /* Overfill is an alarm in every state: liquid can also arrive from outside the
     * transfer (e.g. another line), and it must be latched even when the pump is off. */
    if (c->state != CTL_FAULT && ctl_link_ok(c, now_ms) && c->telem.dst_ml > c->cfg.dst_high_ml) {
        char d[72];
        snprintf(d, sizeof(d), "dst %u > high limit %u mL (state %s)", c->telem.dst_ml,
                 c->cfg.dst_high_ml, ctl_state_name(c->state));
        fault(c, F_OVERFILL, now_ms, d);
    }
    if (c->state == CTL_CHECK) {
        tick_check(c, now_ms);
    } else if (c->state == CTL_TRANSFER) {
        if (!ctl_link_ok(c, now_ms)) {
            char d[72];
            snprintf(d, sizeof(d), "no valid telemetry for %lu ms",
                     (unsigned long)ctl_link_age_ms(c, now_ms));
            fault(c, F_LINK_LOST, now_ms, d);
        } else {
            tick_transfer(c, now_ms);
        }
    }
    apply_outputs(c);
}
