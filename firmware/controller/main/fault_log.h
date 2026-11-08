/*
 * Persistent fault log and boot counter (NVS).
 *
 * The last FAULT_LOG_LEN faults survive reboots and power cycles, so a fault that ended
 * in a reset can still be analysed. Writes go through a queue to a low-priority task:
 * the 10 ms control loop never waits for flash.
 */
#ifndef FAULT_LOG_H
#define FAULT_LOG_H

#include <stdint.h>

#define FAULT_LOG_LEN 8

/* Initialises NVS, increments and returns the boot counter. */
uint32_t fault_log_init(void);
uint32_t fault_log_boot_count(void);

/* Non-blocking: queues the record (dropped and counted if the queue is full). */
void fault_log_record(uint8_t cause, const char *cause_name, const char *detail, uint32_t uptime_ms);

void fault_log_print(void);
void fault_log_clear(void);

/* Black box: hand over a completed capture (copied; non-blocking, dropped if the previous
 * one is still being written). The writer task stores it in NVS and prints it. */
#include "blackbox.h"
void fault_log_save_blackbox(const blackbox_t *bb, const char *cause_name);
/* Print the last stored black box (survives reboots). */
void fault_log_print_blackbox(void);

#endif
