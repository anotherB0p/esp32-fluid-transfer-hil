#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "controller_fsm.h"

/* On-board WS2812 (GPIO21): IDLE white, CHECK yellow, TRANSFER blue, COMPLETE green,
 * FAULT blinking red. Link down outside FAULT: blinking magenta. */
void status_led_init(void);
void status_led_update(ctl_state_t state, bool link_ok, uint32_t now_ms);

#endif
