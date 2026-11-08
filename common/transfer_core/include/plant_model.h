/*
 * Simulated plant: two tanks, one pump, one on/off valve in series.
 *
 * This is a deliberately simple discrete-time model for exercising the controller.
 * It is NOT a hydraulic calculation:
 *   - fixed time step PLANT_DT_MS (100 ms);
 *   - flow exists only when the pump is actually running AND the valve is actually
 *     open AND the source is not empty;
 *   - flow follows its target with a first-order lag (PLANT_FLOW_ALPHA per step),
 *     a crude stand-in for pump spin-up / pipe filling;
 *   - the valve moves to the commanded position after PLANT_VALVE_TRAVEL_MS;
 *   - no pressure, no gravity head, no leakage unless injected.
 * Units: mL and mL/s, levels stored as float internally, reported as integer mL.
 *
 * Commanded vs actual state are separate on purpose: the "stuck valve" and
 * "dead pump" faults make the actual state ignore the command.
 *
 * Pure C99, no platform dependencies.
 */
#ifndef PLANT_MODEL_H
#define PLANT_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#include "proto.h"

#define PLANT_DT_MS 100u
#define PLANT_NOMINAL_FLOW_ML_S 50.0f
#define PLANT_FLOW_ALPHA 0.3f
#define PLANT_VALVE_TRAVEL_MS 300u
#define PLANT_CMD_TIMEOUT_MS 300u /* no valid CMD for this long -> failsafe */

typedef struct {
    /* simulated physical state */
    float src_ml;
    float dst_ml;
    float flow_ml_s;
    bool valve_open;   /* actual */
    bool pump_running; /* actual */
    uint32_t valve_travel_ms;

    /* commands as last applied (after failsafe override) */
    bool pump_cmd;
    bool valve_cmd;

    /* injected faults (simulated) */
    bool fault_valve_stuck; /* valve keeps its current position */
    bool fault_pump_dead;   /* pump never runs */
    float leak_ml_s;        /* source loses liquid outside the pipe */
    float dst_inflow_ml_s;  /* destination receives liquid from outside the transfer */
    bool fault_dst_frozen;  /* destination level sensor reports a frozen value */
    float dst_frozen_value;

    /* link supervision */
    bool armed; /* false after boot and after failsafe: run commands are ignored until the
                   controller sends a safe command (pump off, valve closed). Prevents the plant
                   from resuming a transfer on its own after a reboot or link recovery. */
    bool restarted; /* true from boot until the first arming */
    bool have_ctl_hello;  /* a controller HELLO was received since boot */
    bool ctl_compatible;  /* ... and it speaks PROTO_VERSION */
    uint8_t ctl_proto_ver;
    bool failsafe;
    uint32_t last_cmd_ms;
    bool have_cmd;
    uint8_t cmd_seq_ack;
} plant_t;

void plant_init(plant_t *p, float src_ml, float dst_ml, uint32_t now_ms);

typedef enum {
    PLANT_CMD_APPLIED = 0,
    PLANT_CMD_ARMED,    /* safe command received while not armed: now armed */
    PLANT_CMD_REJECTED, /* run command while not armed: ignored, actuators kept safe */
    PLANT_CMD_NO_PEER,  /* safe command, but no compatible controller HELLO yet: stays unarmed */
} plant_cmd_result_t;

/* Record a controller HELLO. Arming requires a compatible one. Returns true if the
 * controller's protocol version matches PROTO_VERSION. */
bool plant_on_hello(plant_t *p, const proto_hello_t *h);

/* Apply a valid CMD frame received at now_ms. */
plant_cmd_result_t plant_on_cmd(plant_t *p, const proto_cmd_t *cmd, uint8_t seq, uint32_t now_ms);

/*
 * Advance the model by one step of PLANT_DT_MS. Checks CMD freshness first; if the
 * last valid CMD is older than PLANT_CMD_TIMEOUT_MS (or none since boot), forces the
 * safe state: pump off, valve commanded closed. Returns true if failsafe was entered
 * during this step (for logging).
 */
bool plant_step(plant_t *p, uint32_t now_ms);

void plant_fill_telem(const plant_t *p, proto_telem_t *t);

#endif /* PLANT_MODEL_H */
