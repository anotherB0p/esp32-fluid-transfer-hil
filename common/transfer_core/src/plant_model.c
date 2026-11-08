#include "plant_model.h"

#include <string.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void plant_init(plant_t *p, float src_ml, float dst_ml, uint32_t now_ms)
{
    memset(p, 0, sizeof(*p));
    p->src_ml = clampf(src_ml, 0.0f, (float)PROTO_TANK_CAPACITY_ML);
    p->dst_ml = clampf(dst_ml, 0.0f, (float)PROTO_TANK_CAPACITY_ML);
    p->last_cmd_ms = now_ms;
    p->have_cmd = false;
    p->failsafe = true; /* safe until the first valid command arrives */
    p->restarted = true;
}

plant_cmd_result_t plant_on_cmd(plant_t *p, const proto_cmd_t *cmd, uint8_t seq, uint32_t now_ms)
{
    const bool run = cmd->pump_on != 0u || cmd->valve_open != 0u;
    plant_cmd_result_t res = PLANT_CMD_APPLIED;

    p->last_cmd_ms = now_ms;
    p->have_cmd = true;
    p->cmd_seq_ack = seq;
    p->failsafe = false;

    if (!p->armed) {
        if (run) {
            p->pump_cmd = false;
            p->valve_cmd = false;
            return PLANT_CMD_REJECTED;
        }
        p->armed = true;
        p->restarted = false;
        res = PLANT_CMD_ARMED;
    }
    p->pump_cmd = cmd->pump_on != 0u;
    p->valve_cmd = cmd->valve_open != 0u;
    return res;
}

bool plant_step(plant_t *p, uint32_t now_ms)
{
    bool entered_failsafe = false;

    /* 1. Link supervision: the plant protects itself, independent of the controller. */
    if (!p->have_cmd || (uint32_t)(now_ms - p->last_cmd_ms) > PLANT_CMD_TIMEOUT_MS) {
        if (!p->failsafe) {
            entered_failsafe = true;
        }
        p->failsafe = true;
        p->armed = false;
        p->have_cmd = false;
        p->pump_cmd = false;
        p->valve_cmd = false;
    }

    /* 2. Actuators: actual state follows command unless a fault is injected. */
    p->pump_running = p->pump_cmd && !p->fault_pump_dead;

    if (!p->fault_valve_stuck && p->valve_cmd != p->valve_open) {
        p->valve_travel_ms += PLANT_DT_MS;
        if (p->valve_travel_ms >= PLANT_VALVE_TRAVEL_MS) {
            p->valve_open = p->valve_cmd;
            p->valve_travel_ms = 0;
        }
    } else {
        p->valve_travel_ms = 0;
    }

    /* 3. Flow and levels. */
    const float dt_s = (float)PLANT_DT_MS / 1000.0f;
    float target = 0.0f;
    if (p->pump_running && p->valve_open && p->src_ml > 0.0f &&
        p->dst_ml < (float)PROTO_TANK_CAPACITY_ML) {
        target = PLANT_NOMINAL_FLOW_ML_S;
    }
    p->flow_ml_s += (target - p->flow_ml_s) * PLANT_FLOW_ALPHA;
    if (!p->valve_open || (p->flow_ml_s < 0.5f && target == 0.0f)) {
        p->flow_ml_s = 0.0f; /* a closed valve stops the flow at once; the pump only coasts */
    }

    float moved = p->flow_ml_s * dt_s;
    if (moved > p->src_ml) {
        moved = p->src_ml;
    }
    if (moved > (float)PROTO_TANK_CAPACITY_ML - p->dst_ml) {
        moved = (float)PROTO_TANK_CAPACITY_ML - p->dst_ml;
    }
    p->src_ml -= moved;
    p->dst_ml += moved;
    p->src_ml = clampf(p->src_ml - p->leak_ml_s * dt_s, 0.0f, (float)PROTO_TANK_CAPACITY_ML);
    if (p->src_ml <= 0.0f && target > 0.0f) {
        p->flow_ml_s = 0.0f; /* pump runs dry */
    }

    return entered_failsafe;
}

void plant_fill_telem(const plant_t *p, proto_telem_t *t)
{
    t->src_ml = (uint16_t)(p->src_ml + 0.5f);
    t->dst_ml = (uint16_t)(p->dst_ml + 0.5f);
    t->flow_ml_s = (uint16_t)(p->flow_ml_s + 0.5f);
    t->valve_open = p->valve_open ? 1u : 0u;
    t->pump_running = p->pump_running ? 1u : 0u;
    t->cmd_seq_ack = p->cmd_seq_ack;
    t->flags = (uint8_t)((p->failsafe ? PLANT_FLAG_FAILSAFE : 0u) |
                         (p->armed ? 0u : PLANT_FLAG_NOT_ARMED) |
                         (p->restarted ? PLANT_FLAG_RESTARTED : 0u) |
                         ((p->fault_valve_stuck || p->fault_pump_dead || p->leak_ml_s > 0.0f)
                              ? PLANT_FLAG_FAULT_INJ
                              : 0u));
}
