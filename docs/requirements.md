# Requirements

Requirements of the bench, numbered for traceability. "Verified by" lists host unit tests
(`host:` = function in `tests/test_core.c`) and hardware scenarios (`hw:` = scenario in
`tools/bench.py`). `tools/trace.py` checks that every referenced test exists. It reads the
results of the latest host run and of the hardware logs in `logs/` and writes
[traceability.md](traceability.md). CI fails if a requirement has no verification or
references a test that does not exist.

Values are those of this bench (simulated plant, 100 ms frames). They are design
targets for a demonstrator, not flight requirements.

| ID | Requirement | Verified by |
|---|---|---|
| REQ-FSM-001 | A transfer starts only on an explicit operator `start`; after a controller reboot the controller is IDLE and nothing resumes. | host:test_controller_reboot_does_not_resume, hw:ctl_reboot |
| REQ-FSM-002 | Before pumping the controller checks: fresh link, plant armed and compatible, pump off and valve closed, destination below target, source >= needed + 100 mL reserve. Any failed check is a FAULT and the pump is never commanded on. | host:test_loop_insufficient_source, host:test_loop_start_without_link, hw:insufficient_source |
| REQ-FSM-003 | The transfer stops at the target; final destination level within +-3 mL of a 500 mL target. | host:test_loop_happy_path, hw:happy |
| REQ-FSM-004 | A FAULT is latched with its first cause and a detail text. It is left only by `reset`, and only while the plant is observed safe (fresh link, armed, compatible, pump off, valve closed, destination below high limit). | host:test_loop_valve_stuck_open_on_stop, host:test_loop_telemetry_lost, hw:valve_stuck_open |
| REQ-FSM-005 | Operator `stop` during CHECK/TRANSFER is a latched `OPERATOR_STOP`; `start` is refused while in FAULT. | host:test_loop_operator_stop_and_start_refused_in_fault, hw:fault_log_persist |
| REQ-ACT-001 | The valve must report OPEN within 1 s of the command, otherwise `VALVE_NOT_OPEN`; the pump is not started before the valve is confirmed open. | host:test_loop_stuck_valve_closed, hw:valve_stuck_closed |
| REQ-ACT-002 | When stopping, pump off and valve closed must be confirmed within 1 s, otherwise `VALVE_NOT_CLOSED`. | host:test_loop_valve_stuck_open_on_stop, hw:valve_stuck_open |
| REQ-ACT-003 | Flow must reach 20 mL/s within 1.5 s of the pump command, otherwise `NO_FLOW`. | host:test_loop_dead_pump_no_flow, hw:no_flow |
| REQ-SAF-001 | Source at or below the 100 mL reserve during a transfer is `SOURCE_LOW`; the pump is stopped. | host:test_loop_source_low_by_leak, hw:source_low |
| REQ-SAF-002 | Destination above the 950 mL high limit is `OVERFILL` in any state; `reset` is refused while it is still above. | host:test_loop_overfill_in_idle, hw:overfill_idle |
| REQ-LNK-001 | No valid telemetry for 300 ms is `LINK_LOST`; the controller then commands the safe state. | host:test_loop_telemetry_lost, hw:link_telem_lost, hw:plant_hang |
| REQ-LNK-002 | No valid command for 300 ms: the plant itself switches the pump off and closes the valve, independent of the controller. | host:test_plant_cmd_timeout_failsafe, host:test_loop_commands_lost_plant_failsafe, hw:link_cmd_lost, hw:ctl_hang, hw:wire_pull |
| REQ-LNK-003 | After boot or failsafe the plant ignores run commands until it receives a safe command from a compatible controller (no resume on link recovery or reboot). | host:test_plant_ignores_run_cmd_until_armed, host:test_plant_disarms_after_failsafe, host:test_loop_fast_plant_reboot_mid_transfer, hw:plant_reboot |
| REQ-LNK-004 | Telemetry that no longer acknowledges recent commands (newest acknowledged command older than 400 ms) is `CMD_NOT_ACKED`. | host:test_loop_ack_frozen |
| REQ-LNK-005 | Frames with a bad CRC, length, type or value are rejected and counted; a single bit error costs at most one frame; the decoder resynchronises after junk. | host:test_all_1_and_2_bit_errors_rejected, host:test_single_bit_error_loses_only_that_frame, host:test_decoder_fuzz, host:test_junk_and_resync, host:test_range_check_after_valid_crc, hw:noisy_link |
| REQ-LNK-006 | With heavy corruption (30 % of bytes) the system ends in a latched FAULT with the pump off. | hw:noise_heavy |
| REQ-ID-001 | Each board reports its firmware version and exchanges HELLO (protocol version, role, boot id) with its peer. | host:test_hello_roundtrip_and_role_check, hw:version_handshake |
| REQ-ID-002 | A peer with another protocol version is refused: the controller refuses `start`, the plant does not arm. | host:test_loop_plant_speaks_other_protocol, host:test_loop_controller_speaks_other_protocol, hw:proto_mismatch |
| REQ-ID-003 | A plant reboot during a transfer is detected (RESTARTED flag, boot id change, or link loss) and latched. | host:test_loop_reboot_seen_via_hello_boot_id, host:test_loop_fast_plant_reboot_mid_transfer, hw:plant_reboot |
| REQ-SEN-001 | Destination level unchanged for 1 s while flow >= 20 mL/s is `SENSOR_IMPLAUSIBLE`. | host:test_loop_dst_sensor_frozen, hw:sensor_frozen |
| REQ-SEN-002 | A destination change above 20 mL between two telemetry frames during a transfer is `SENSOR_IMPLAUSIBLE`. | host:test_loop_dst_sensor_jump, hw:sensor_jump_blackbox |
| REQ-DIA-001 | The last 8 faults and a boot counter survive reboots (NVS). | hw:fault_log_persist |
| REQ-DIA-002 | A black box records ~3 s before and 1 s after each fault and keeps the last capture across reboots. | host:test_blackbox_pre_and_post_trigger, hw:sensor_jump_blackbox |
| REQ-DIA-003 | A hung control loop is reset by the task watchdog within ~1 s and the reset reason is reported at boot. | hw:ctl_hang, hw:plant_hang |
| REQ-OTA-001 | A firmware update cannot start while a transfer is active. | hw:ota_refused_in_transfer |
| REQ-OTA-002 | An image whose SHA-256 does not match the announced one is not activated; the running image is unchanged. | hw:ota_bad_sha |
| REQ-OTA-003 | A good image is transferred over the wired console, verified (SHA-256 + image validation), booted from the other A/B slot and kept after its self-test passes. | hw:ota_update_good |
| REQ-OTA-004 | A new image that does not pass its self-test (compatible plant, working link) within 10 s is rolled back to the previous image by the bootloader. | hw:ota_update_rollback |
| REQ-DIA-004 | Link statistics (accepted/rejected frames, sequence gaps, command round-trip time) are reported in `STAT`. | hw:happy, hw:noisy_link |
