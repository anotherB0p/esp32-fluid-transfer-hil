/*
 * Inter-board link protocol (UART, 115200 8N1).
 *
 * Frame layout (all multi-byte fields little-endian):
 *
 *   +------+-----+------+-----+-----------------+--------+
 *   | 0xA5 | LEN | TYPE | SEQ | PAYLOAD (0..24) | CRC16  |
 *   +------+-----+------+-----+-----------------+--------+
 *            LEN = number of bytes in TYPE + SEQ + PAYLOAD (2..26)
 *            CRC16 = CRC-16/CCITT-FALSE over LEN..PAYLOAD (SOF excluded)
 *
 * The decoder is a byte-wise state machine: it never blocks, never allocates and
 * resynchronises on the next 0xA5 after any error (bad length, bad CRC, junk bytes
 * from a board that is booting). Each message type has a fixed payload length and
 * value ranges; frames that pass the CRC but violate them are rejected as well.
 *
 * Pure C99, no platform dependencies: compiled into both firmwares and the host tests.
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTO_SOF 0xA5u
#define PROTO_MAX_PAYLOAD 24u
#define PROTO_MAX_FRAME (1u + 1u + 2u + PROTO_MAX_PAYLOAD + 2u) /* 30 bytes */

/* Simulated tank geometry, shared by both boards so both validate the same ranges. */
#define PROTO_TANK_CAPACITY_ML 1000u
#define PROTO_MAX_FLOW_ML_S 200u

typedef enum {
    MSG_CMD = 0x01,   /* controller -> plant, every 100 ms */
    MSG_TELEM = 0x02, /* plant -> controller, every 100 ms */
} proto_msg_type_t;

/* Controller -> plant: commanded actuator state. */
typedef struct {
    uint8_t pump_on;    /* 0/1 */
    uint8_t valve_open; /* 0/1 */
} proto_cmd_t;
#define PROTO_CMD_LEN 2u

/* Plant flags reported in telemetry. */
#define PLANT_FLAG_FAILSAFE 0x01u /* plant forced pump off / valve close itself (no fresh CMD) */
#define PLANT_FLAG_FAULT_INJ 0x02u /* at least one simulated fault is injected (for logs only) */
#define PLANT_FLAG_NOT_ARMED 0x04u /* after boot/failsafe: run commands ignored until a safe CMD */
#define PLANT_FLAG_RESTARTED 0x08u /* plant booted and has not been armed since */
#define PLANT_FLAGS_ALL \
    (PLANT_FLAG_FAILSAFE | PLANT_FLAG_FAULT_INJ | PLANT_FLAG_NOT_ARMED | PLANT_FLAG_RESTARTED)

/* Plant -> controller: simulated sensors and actual actuator state. */
typedef struct {
    uint16_t src_ml;      /* source tank level, mL, 0..capacity */
    uint16_t dst_ml;      /* destination tank level, mL, 0..capacity */
    uint16_t flow_ml_s;   /* measured transfer flow, mL/s, 0..PROTO_MAX_FLOW_ML_S */
    uint8_t valve_open;   /* ACTUAL valve position (limit switch), 0/1 */
    uint8_t pump_running; /* ACTUAL pump state, 0/1 */
    uint8_t cmd_seq_ack;  /* SEQ of the last valid CMD the plant applied */
    uint8_t flags;        /* PLANT_FLAG_* */
} proto_telem_t;
#define PROTO_TELEM_LEN 10u

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout. */
uint16_t proto_crc16(const uint8_t *data, size_t len);

/* Encoders. Return frame length written to out (out must hold PROTO_MAX_FRAME). */
size_t proto_encode_cmd(uint8_t seq, const proto_cmd_t *cmd, uint8_t *out);
size_t proto_encode_telem(uint8_t seq, const proto_telem_t *t, uint8_t *out);

/* Streaming decoder. */
typedef struct {
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t len_errors;     /* LEN outside 2..26 or wrong for the TYPE */
    uint32_t range_errors;   /* CRC ok but a field is out of range */
    uint32_t unknown_type;
    uint32_t junk_bytes;     /* bytes skipped while hunting for SOF */
} proto_stats_t;

typedef struct {
    uint8_t state;
    uint8_t len;
    uint8_t pos;
    uint8_t buf[2u + PROTO_MAX_PAYLOAD + 2u]; /* LEN..CRC */
    proto_stats_t stats;
} proto_decoder_t;

typedef struct {
    proto_msg_type_t type;
    uint8_t seq;
    union {
        proto_cmd_t cmd;
        proto_telem_t telem;
    } u;
} proto_msg_t;

void proto_decoder_init(proto_decoder_t *d);

/* Feed one byte. Returns true when a complete, valid message was decoded into *msg. */
bool proto_decoder_feed(proto_decoder_t *d, uint8_t byte, proto_msg_t *msg);

#endif /* PROTO_H */
