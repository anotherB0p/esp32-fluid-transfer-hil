#include "proto.h"

#include <string.h>

enum { ST_SOF = 0, ST_LEN, ST_BODY };

uint16_t proto_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static size_t finish_frame(uint8_t *out, uint8_t type, uint8_t seq, size_t payload_len)
{
    out[0] = PROTO_SOF;
    out[1] = (uint8_t)(2u + payload_len);
    out[2] = type;
    out[3] = seq;
    uint16_t crc = proto_crc16(&out[1], 1u + 2u + payload_len);
    put_u16(&out[4 + payload_len], crc);
    return 4u + payload_len + 2u;
}

size_t proto_encode_cmd(uint8_t seq, const proto_cmd_t *cmd, uint8_t *out)
{
    out[4] = cmd->pump_on ? 1u : 0u;
    out[5] = cmd->valve_open ? 1u : 0u;
    return finish_frame(out, MSG_CMD, seq, PROTO_CMD_LEN);
}

size_t proto_encode_telem(uint8_t seq, const proto_telem_t *t, uint8_t *out)
{
    put_u16(&out[4], t->src_ml);
    put_u16(&out[6], t->dst_ml);
    put_u16(&out[8], t->flow_ml_s);
    out[10] = t->valve_open ? 1u : 0u;
    out[11] = t->pump_running ? 1u : 0u;
    out[12] = t->cmd_seq_ack;
    out[13] = t->flags;
    return finish_frame(out, MSG_TELEM, seq, PROTO_TELEM_LEN);
}

void proto_decoder_init(proto_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = ST_SOF;
}

static bool is_bool(uint8_t v)
{
    return v <= 1u;
}

/* buf = LEN TYPE SEQ PAYLOAD (CRC already checked). */
static bool parse_body(proto_decoder_t *d, proto_msg_t *msg)
{
    const uint8_t len = d->buf[0];
    const uint8_t type = d->buf[1];
    const uint8_t *p = &d->buf[3];
    const uint8_t plen = (uint8_t)(len - 2u);

    msg->seq = d->buf[2];
    switch (type) {
    case MSG_CMD:
        if (plen != PROTO_CMD_LEN) {
            d->stats.len_errors++;
            return false;
        }
        if (!is_bool(p[0]) || !is_bool(p[1])) {
            d->stats.range_errors++;
            return false;
        }
        msg->type = MSG_CMD;
        msg->u.cmd.pump_on = p[0];
        msg->u.cmd.valve_open = p[1];
        return true;
    case MSG_TELEM: {
        if (plen != PROTO_TELEM_LEN) {
            d->stats.len_errors++;
            return false;
        }
        proto_telem_t t = {
            .src_ml = get_u16(&p[0]),
            .dst_ml = get_u16(&p[2]),
            .flow_ml_s = get_u16(&p[4]),
            .valve_open = p[6],
            .pump_running = p[7],
            .cmd_seq_ack = p[8],
            .flags = p[9],
        };
        if (t.src_ml > PROTO_TANK_CAPACITY_ML || t.dst_ml > PROTO_TANK_CAPACITY_ML ||
            t.flow_ml_s > PROTO_MAX_FLOW_ML_S || !is_bool(t.valve_open) ||
            !is_bool(t.pump_running) ||
            (t.flags & (uint8_t)~PLANT_FLAGS_ALL) != 0u) {
            d->stats.range_errors++;
            return false;
        }
        msg->type = MSG_TELEM;
        msg->u.telem = t;
        return true;
    }
    default:
        d->stats.unknown_type++;
        return false;
    }
}

bool proto_decoder_feed(proto_decoder_t *d, uint8_t byte, proto_msg_t *msg)
{
    switch (d->state) {
    case ST_SOF:
        if (byte == PROTO_SOF) {
            d->state = ST_LEN;
        } else {
            d->stats.junk_bytes++;
        }
        return false;

    case ST_LEN:
        if (byte < 2u || byte > 2u + PROTO_MAX_PAYLOAD) {
            d->stats.len_errors++;
            /* This byte may itself be the SOF of the next frame. */
            d->state = (byte == PROTO_SOF) ? ST_LEN : ST_SOF;
            return false;
        }
        d->len = byte;
        d->buf[0] = byte;
        d->pos = 1;
        d->state = ST_BODY;
        return false;

    case ST_BODY:
    default:
        d->buf[d->pos++] = byte;
        if (d->pos < 1u + d->len + 2u) {
            return false;
        }
        d->state = ST_SOF;
        {
            const uint16_t rx_crc = get_u16(&d->buf[1u + d->len]);
            if (proto_crc16(d->buf, 1u + d->len) != rx_crc) {
                d->stats.crc_errors++;
                return false;
            }
        }
        if (!parse_body(d, msg)) {
            return false;
        }
        d->stats.frames_ok++;
        return true;
    }
}
