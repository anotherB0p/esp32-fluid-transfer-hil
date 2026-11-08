/*
 * Flight-recorder style "black box" for the controller.
 *
 * Samples (one per telemetry period) go into a ring of BB_LEN entries. When a fault is
 * triggered the recorder keeps recording BB_POST more samples, so the snapshot shows
 * BB_LEN - BB_POST samples before the fault and BB_POST after it (the reaction: pump off,
 * valve closing). Then the ring freezes until the snapshot has been saved and bb_rearm()
 * is called. Pure C99, no allocation.
 */
#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

#define BB_LEN 40u  /* 4 s at 10 Hz */
#define BB_POST 10u /* 1 s after the trigger */

#define BB_IO_PUMP_CMD 0x01u
#define BB_IO_VALVE_CMD 0x02u
#define BB_IO_PUMP_ACT 0x04u
#define BB_IO_VALVE_ACT 0x08u

typedef struct {
    uint32_t t_ms;
    uint8_t state;
    uint8_t phase;
    uint8_t io;    /* BB_IO_* */
    uint8_t flags; /* plant telemetry flags */
    uint16_t src_ml;
    uint16_t dst_ml;
    uint16_t flow_ml_s;
    uint16_t link_age_ms;
} bb_sample_t;

typedef struct {
    bb_sample_t ring[BB_LEN];
    uint32_t written;   /* samples written since (re)arm, saturates at BB_LEN for ordering */
    uint32_t head;      /* next slot */
    bool triggered;
    uint32_t post_left;
    bool frozen;        /* snapshot complete, waiting for bb_rearm() */
    uint8_t cause;
    uint32_t trigger_ms;
} blackbox_t;

void bb_init(blackbox_t *bb);

/* Adds a sample unless frozen. Returns true exactly once, when a triggered capture
 * becomes complete (BB_POST samples after the trigger). */
bool bb_add(blackbox_t *bb, const bb_sample_t *s);

/* Starts the post-trigger countdown. Ignored if a capture is already in progress. */
void bb_trigger(blackbox_t *bb, uint8_t cause, uint32_t now_ms);

/* Number of valid samples and access in chronological order (0 = oldest). */
uint32_t bb_count(const blackbox_t *bb);
const bb_sample_t *bb_at(const blackbox_t *bb, uint32_t i);

/* Clears the trigger and unfreezes; keeps the recorded history. */
void bb_rearm(blackbox_t *bb);

#endif /* BLACKBOX_H */
