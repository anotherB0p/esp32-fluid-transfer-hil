#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t src_ml;
    uint16_t dst_ml;
    uint16_t flow_ml_s;
    bool valve_open;
    bool pump_running;
    bool failsafe;
    bool fault_injected;
} oled_view_t;

/* Returns false if the display does not answer; the plant keeps running without it. */
bool oled_init(void);
void oled_set_view(const oled_view_t *v);

#endif
