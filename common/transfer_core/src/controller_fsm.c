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
    c->telem = *t;
    c->have_telem = true;
    c->last_telem_ms = now_ms;
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

    if (!ctl_link_ok(c, now) || (t->flags & (PLANT_FLAG_FAILSAFE | PLANT_FLAG_NOT_ARMED))) {
        if (in_state > c->cfg.check_timeout_ms) {
            if (!ctl_link_ok(c, now)) {
                fault(c, F_LINK_LOST, now, "CHECK: no fresh telemetry");
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
    if (t->flags & PLANT_FLAG_NOT_ARMED) {
        fault(c, F_PLANT_NOT_ARMED, now, "plant reports NOT_ARMED (recovered from failsafe)");
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
        } else if (t->dst_ml >= c->target_dst_ml) {
            snprintf(d, sizeof(d), "target reached: dst %u >= %u mL", t->dst_ml, c->target_dst_ml);
            set_phase(c, PH_STOPPING, now, d);
        } else if (t->src_ml <= c->cfg.min_src_ml) {
            snprintf(d, sizeof(d), "src %u <= reserve %u mL", t->src_ml, c->cfg.min_src_ml);
            fault(c, F_SOURCE_LOW, now, d);
        } else if (in_phase > c->cfg.flow_grace_ms && t->flow_ml_s < c->cfg.flow_min_ml_s) {
            snprintf(d, sizeof(d), "flow %u < %u mL/s, pump actual=%u", t->flow_ml_s,
                     c->cfg.flow_min_ml_s, t->pump_running);
            fault(c, F_NO_FLOW, now, d);
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
