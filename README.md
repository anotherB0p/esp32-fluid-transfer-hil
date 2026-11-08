# Two-board liquid transfer bench (ESP32-S3 controller + ESP32-C3 plant simulator)

[![ci](https://github.com/anotherB0p/esp32-fluid-transfer-hil/actions/workflows/ci.yml/badge.svg)](https://github.com/anotherB0p/esp32-fluid-transfer-hil/actions/workflows/ci.yml)

A small embedded demo built for the Spaceium Embedded Software Engineer take-home
("build a small embedded or hardware-related project in 3 hours"). The project is a
supervisory controller for a two-tank transfer: a state machine with checks before
start, supervision during the transfer, latched faults, an explicit reset, and safe
reactions to link loss and reboots. It runs on two real microcontrollers connected by
a UART link.

## What is real and what is simulated

| Real (runs on the microcontrollers) | Simulated (in software, on the C3) |
|---|---|
| Controller state machine, timeouts, interlocks, fault latching (ESP32-S3) | Liquid, both tanks, pump, valve, level and flow sensors |
| Inter-board protocol over a physical 3-wire UART link (framing, CRC, validation, freshness) | Pump spin-up/coast-down, valve travel time |
| Plant-side failsafe and arming logic (ESP32-C3) | Faults: stuck valve, dead pump, leak (injected from the plant console) |
| Status RGB LED (S3) and OLED tank picture (C3) | |

No pumps, valves, relays or mains are connected. The plant model is a deliberately
simple discrete-time model (see [Plant model](#plant-model)), not a hydraulic
calculation.

## Hardware

| Role | Board | Chip (verified with `esptool flash-id`) | PC port |
|---|---|---|---|
| Controller | ESP32-S3-Zero form factor (inventory A13) | ESP32-S3 (QFN56) rev v0.2, 4 MB flash, 2 MB PSRAM, USB-Serial/JTAG | COM7 on the author's PC |
| Plant simulator | ESP32-C3 with built-in 0.42" 72x40 SSD1306 OLED (inventory A09) | ESP32-C3 (QFN32) rev v0.4, 4 MB flash, USB-Serial/JTAG | COM8 on the author's PC |

Both boards are powered from their own USB cables (same PC). The bench script finds
the boards by USB serial number (= MAC), not by COM number.

### Wiring (UART1, 115200 8N1, 3.3 V on both sides)

| Wire | Controller S3 pad (GPIO) | | Plant C3 pad (GPIO) | Signal |
|---|---|---|---|---|
| 1 | `1` (GPIO1, UART1 TX) | -> | `4` (GPIO4, UART1 RX) | commands |
| 2 | `2` (GPIO2, UART1 RX) | <- | `3` (GPIO3, UART1 TX) | telemetry |
| 3 | `GND` | -- | `GD` | ground |

`3V3` and `5V` are not connected between the boards. Pin choice avoids strapping pins
(S3: 0/3/45/46, C3: 2/8/9), USB pins, flash/PSRAM pins, the S3 RGB LED (GPIO21), the
C3 OLED I2C pins (GPIO5/6) and the UART0 pads, because UART0 TX carries the ROM boot
log at every reset. Sources and the full verification are in
[docs/hardware.md](docs/hardware.md).

## Architecture

```
            USB console (operator)                         USB console (fault injection)
                    |                                                   |
 +------------------v------------------+   UART1 115200   +-------------v---------------+
 | ESP32-S3  controller                |  CMD  every 100ms| ESP32-C3  plant simulator   |
 |  control task, 10 ms period:        |----------------->|  plant task, 10 ms period:  |
 |   decode TELEM -> ctl_on_telem()    |                  |   decode CMD -> plant_on_cmd|
 |   console: start/stop/reset/status  |<-----------------|   plant_step() every 100 ms |
 |   ctl_tick() -> pump/valve command  | TELEM every 100ms|   failsafe + arming         |
 |   WS2812 state colour               |                  |   OLED task (200 ms)        |
 +-------------------------------------+                  +-----------------------------+
          common/transfer_core: proto.c, controller_fsm.c, plant_model.c
          (pure C99, also compiled and unit-tested on the PC)
```

```
common/transfer_core/   protocol, controller FSM, plant model (no ESP-IDF dependencies)
common/bench_io/        ESP-IDF helpers: monotonic ms clock, UART1 link, USB console
firmware/controller/    ESP32-S3 application (+ status LED)
firmware/plant/         ESP32-C3 application (+ OLED)
tests/                  host unit tests (gcc), incl. closed-loop controller<->plant tests
tools/bench.py          hardware scenario runner, writes logs/
tools/bringup/          board bring-up and wiring diagnostic firmware (preparation)
docs/                   hardware verification, time log, Russian guide, video script
```

The controller loop never blocks: UART reads and console reads are non-blocking,
UART writes go to a 512-byte ring buffer (a frame is at most 30 bytes every 100 ms),
and all timeouts use `esp_timer` milliseconds with wrap-safe arithmetic.

### Controller state machine

```
IDLE --start--> CHECK --checks ok--> TRANSFER --------------------------> COMPLETE
                  |                   OPEN_VALVE -> PUMPING -> STOPPING     |
                  |                         |                               | start / reset
                  +--------- any failed check or supervision -----> FAULT (latched)
                                                                      | reset (only if safe)
                                                                      v
                                                                     IDLE
```

- **CHECK** waits up to 1 s for a working link (fresh telemetry, plant armed and not in
  failsafe). It then requires pump off and valve closed, destination below target, and
  `source >= (target - destination) + 100 mL reserve`.
- **TRANSFER/OPEN_VALVE**: valve commanded open, pump off; the valve must report open
  within 1 s.
- **TRANSFER/PUMPING**: pump on. Supervision: valve still open, flow >= 20 mL/s after a
  1.5 s grace period, source above the reserve, destination below the 950 mL high limit.
- **TRANSFER/STOPPING**: pump off, valve commanded closed; both must be confirmed
  within 1 s, otherwise `VALVE_NOT_CLOSED`.
- **FAULT** stores the first cause plus a detail string and commands the safe state.
  `reset` is accepted only with fresh telemetry, plant armed and not in failsafe, pump
  not running, and valve closed. If the valve is stuck open, the reset is refused.
- **Nothing resumes on its own.** After a controller reboot the controller starts in
  IDLE. Link recovery does not clear a FAULT.

Fault causes: `LINK_LOST`, `PLANT_NO_CMD`, `PLANT_RESTARTED`, `PLANT_NOT_ARMED`,
`PLANT_NOT_SAFE`, `TARGET_REACHED`, `INSUFFICIENT_SOURCE`, `VALVE_NOT_OPEN`, `NO_FLOW`,
`SOURCE_LOW`, `OVERFILL`, `VALVE_NOT_CLOSED`, `OPERATOR_STOP`.

### Protocol

```
| 0xA5 | LEN | TYPE | SEQ | PAYLOAD (0..24) | CRC16 (LE) |      LEN = 2 + payload length
```

- CRC-16/CCITT-FALSE over LEN..PAYLOAD. Frames are at most 30 bytes.
- `CMD` (0x01, controller -> plant, 100 ms): `pump_on`, `valve_open`.
- `TELEM` (0x02, plant -> controller, 100 ms): `src_ml`, `dst_ml`, `flow_ml_s`, actual
  `valve_open`, actual `pump_running`, `cmd_seq_ack`, `flags` (FAILSAFE, FAULT_INJ,
  NOT_ARMED, RESTARTED).
- The decoder is a byte-wise state machine. It rejects bad length, bad CRC, a length
  wrong for the type, and out-of-range values (levels > 1000 mL, flow > 200 mL/s,
  booleans other than 0/1, unknown flags). After any error it resynchronises on the
  next `0xA5`. Counters for each error class appear in every `STAT` line.
- Freshness: the controller treats telemetry older than 300 ms as `LINK_LOST`. The
  plant treats commands older than 300 ms as lost and goes to failsafe **on its own**:
  pump off, valve commanded closed.
- **Arming:** after boot and after any failsafe the plant is `NOT_ARMED`. It ignores
  run commands until it receives one safe command (pump off, valve closed). This was
  added after a bench finding (see below).

### Plant model

- Fixed step of 100 ms. Tanks are 1000 mL each. Initial levels after boot are
  src 800 / dst 100 mL.
- Flow exists only if the pump is actually running, the valve is actually open, the
  source is not empty and the destination is not full. Flow follows its target
  (50 mL/s) with a first-order lag (alpha 0.3 per step). A closed valve stops flow at once.
- The valve moves to the commanded position after 300 ms of travel.
- Injected faults: `valve_stuck` (the valve keeps its current position), `pump_dead`
  (the pump never runs), `leak <mL/s>` (the source loses liquid outside the pipe).
- Not modelled: pressure, gravity head, pipe volume, sensor noise.

## Build, flash, run

Requirements: ESP-IDF v6.0 at `C:\esp\release-v6.0\esp-idf` (the helper script
sources its `export.ps1`). The host tests need any gcc; the author's PC uses MinGW-w64
GCC 15.2 installed with `scoop install gcc`. The bench script needs Python with
pyserial (the ESP-IDF Python env has it).

```powershell
# host unit tests (Windows; on Linux/macOS: bash tests/run_tests.sh)
.\tests\run_tests.ps1

# build + flash (replace COM ports with yours)
.\tools\idf.ps1 -Project firmware\controller -Target esp32s3 -Port COM7 build flash
.\tools\idf.ps1 -Project firmware\plant      -Target esp32c3 -Port COM8 build flash

# hardware scenarios (boards found by USB serial number)
python tools\bench.py list
python tools\bench.py run all
python tools\bench.py send ctl "start 500" -w 12     # one command, watch both consoles
```

Any serial terminal at 115200 works for manual use, for example
`idf.py -p COM7 monitor`. Each board prints a `STAT` line every second and an `EVT`
line on every transition.

| Controller console (S3) | |
|---|---|
| `start [target_mL]` | start a transfer (default 500 mL) |
| `stop` | operator stop -> latched `OPERATOR_STOP` |
| `reset` | leave FAULT/COMPLETE if the plant is observed safe |
| `status` | print the current state |
| `mute <ms>` | test hook: stop sending CMD for a while |
| `faults [clear]` | (v2) show / clear the persistent fault log |
| `noise <ppm>` | (v2) test hook: flip random bits in transmitted CMD bytes |
| `hang` | (v2) test hook: stop the control loop, the watchdog must reset the board |
| `reboot` | software restart |

| Plant console (C3) | |
|---|---|
| `set src|dst <mL>` | set a simulated level |
| `fault valve_stuck <0|1>` | freeze the valve in its current position |
| `fault pump_dead <0|1>` | the pump never runs |
| `fault leak <mL/s>` | the source leaks |
| `fault clear` | remove all injected faults |
| `mute <ms>` | test hook: send no TELEM and ignore CMD for a while |
| `noise <ppm>` | (v2) test hook: flip random bits in transmitted TELEM bytes |
| `hang` | (v2) test hook: stop the plant loop, the watchdog must reset the board |
| `status`, `reboot` | |

LED (S3): IDLE dim white, CHECK yellow, TRANSFER blue, COMPLETE green, FAULT blinking
red, link down in IDLE blinking magenta. OLED (C3): two tanks with levels in mL, the
pump box is filled when running, the valve shows an X when closed, dots move in the
pipe while liquid flows, `FS` blinks in failsafe, `!` means a fault is injected.

## Test results

Each kind of evidence is kept separate:

1. **Host unit tests** (`tests/run_tests.ps1`, gcc `-std=c99 -Wall -Wextra -Werror
   -Wconversion`): **27 tests, 122 checks, 0 failed.** They cover protocol
   round-trips, the CRC check value, junk/resync, corrupted frames, bad lengths,
   range checks, the plant model, and closed-loop controller<->plant runs through the
   real encoder/decoder (happy path, every fault, link loss in each direction, reboots).
   They test logic only, not drivers or timing on the boards.
2. **Build:** both firmwares build with ESP-IDF v6.0 without compiler warnings.
3. **Flash:** esptool wrote both images with "Hash of data verified".
4. **Hardware runs** (`python tools/bench.py run all`). A scenario passes only when
   the boards' own log lines show the expected reaction, not when a command was
   merely sent. Final run of the **build-window** firmware:
   `logs/20261108T174015Z_run_all.log`, **10/10 PASS** (the table below). For the v2
   firmware see [v2 hardware results](#v2-hardware-results): 15/15 PASS.

The numbers in this section (27 tests, 122 checks) describe the build-window version;
v2 has 31 tests and 134 checks.

| Scenario | Initial conditions | Action | Expected | Observed on hardware |
|---|---|---|---|---|
| happy | src 800, dst 100 | `start 500` | IDLE->CHECK->TRANSFER->COMPLETE, pump off, valve closed | PASS, 8.98 s; dst 509 mL (9 mL overshoot, see limitations) |
| insufficient_source | src 300, dst 100 | `start 500` | FAULT in CHECK, pump never on | PASS: `CHECK: src 300 < needed 500 mL (incl. reserve 100)` |
| valve_stuck_closed | valve_stuck=1 (closed) | `start 500` | `VALVE_NOT_OPEN` after the 1 s deadline, pump never on | PASS: `valve commanded OPEN, actual CLOSED after 1010 ms` |
| no_flow | pump_dead=1 | `start 500` | `NO_FLOW` after the 1.5 s grace period | PASS: `flow 0 < 20 mL/s, pump actual=0`, 1.53 s after PUMPING |
| source_low | src 800, dst 100 | `start 700`, then `fault leak 150` | `SOURCE_LOW`, pump stopped | PASS: `src 92 <= reserve 100 mL` |
| valve_stuck_open | transfer to 300 | `fault valve_stuck 1` while pumping | `VALVE_NOT_CLOSED`; reset refused until repaired | PASS: first reset `refused: valve still open`, accepted after `fault clear` |
| link_telem_lost | pumping | plant `mute 2500` | controller `LINK_LOST`; plant failsafe on its own; FAULT stays latched after the link returns | PASS: controller `no valid telemetry for 310 ms`; plant `no valid CMD for 380 ms -> pump OFF` |
| link_cmd_lost | pumping | controller `mute 2000` | plant failsafe on its own; controller `PLANT_NO_CMD` | PASS: plant failsafe at `380 ms` without CMD |
| ctl_reboot | pumping | controller `reboot` | controller boots IDLE; plant pump off; no resume | PASS |
| plant_reboot | pumping | plant `reboot` | `PLANT_RESTARTED`; the rebooted plant must not obey the old RUN command | PASS: plant `NOT_ARMED: run command (pump=1 valve=1) IGNORED` |
| wire_pull (manual), run 2 | pumping | owner unplugs only the CMD wire (S3 `1` -> C3 `4`) for ~3.9 s, then plugs it back | plant failsafe on its own; controller FAULT; latched after reconnect | PASS (`logs/20261108T175154Z_run_wire_pull.log`): plant `no valid CMD for 310 ms -> pump OFF, valve commanded CLOSED`; telemetry kept flowing, so the controller reported the correct cause `PLANT_NO_CMD`; plant re-armed only by the controller's safe command; FAULT latched until `reset`. A ~200 ms contact bounce while re-inserting was shorter than the timeout and was correctly ignored |
| wire_pull (manual), run 1 | pumping | owner accidentally disturbed **both** link wires for < 1 s | as above | PASS (`logs/20261108T175106Z_run_wire_pull.log`): plant failsafe after 320 ms; controller `LINK_LOST: no valid telemetry for 310 ms`; FAULT latched |

**Link-loss reaction times.** The most precise numbers are the on-device ages printed
in the fault lines: the controller detects missing telemetry when its last valid frame
is 310 ms old (timeout 300 ms + 10 ms tick), and the plant enters failsafe when its last
valid command is 380 ms old (timeout 300 ms, checked on its 100 ms model step). Times
measured on the PC clock between the `mute` acknowledgement and the reaction line were
246-388 ms. They include USB-CDC buffering, so treat them as indicative only.

**Reboots.** Controller reboot: it starts in IDLE and commands pump off / valve closed.
During the ~100 ms reboot the plant keeps the last command, which is shorter than its
300 ms timeout. Plant reboot: the C3 is back in ~120 ms. It boots NOT_ARMED + RESTARTED,
ignores run commands, and the controller latches `PLANT_RESTARTED`.

## v2: improvements after the 3-hour window

Everything in this section was added after the build window and is logged separately in
[docs/timelog.md](docs/timelog.md).

| Area | What was added |
|---|---|
| Task watchdog | Both boards subscribe their 10 ms loop to the ESP-IDF task watchdog (1 s, panic -> reset). The reset reason is printed at boot (`reset_reason=TASK_WDT`). Test hook `hang`. |
| Persistent fault log | Controller: boot counter plus the last 8 faults (boot number, uptime, cause, detail) in NVS, written by a low-priority task so the control loop never waits for flash. Console `faults [clear]`. |
| Command acknowledgement | The controller remembers when each CMD SEQ was sent and matches it with `cmd_seq_ack` in telemetry: command round-trip time (min/avg/max) in `STAT diag`, plus an independent detector `CMD_NOT_ACKED` (400 ms) that does not rely on the plant reporting its own failsafe. |
| Stop anticipation | Overshoot 9 mL -> under 1 mL (499.3 mL for target 500 on hardware). |
| Decoder hardening | LEN/TYPE consistency check (see the second finding below). |
| Noise injection | `noise <ppm>` on either board flips random bits in its transmitted bytes, on the real wire. |
| Robustness tests (host) | Exhaustive 1-bit and 2-bit error test (8256 corrupted frames, 0 accepted); single-bit-error burst test; 2 MB fuzz run with 20 000 valid frames mixed in (all recovered, 0 false accepts); frozen-acknowledgement test. 31 tests, 134 checks. |
| CI | GitHub Actions: host tests with `-Werror`, cppcheck (warning/style/performance/portability, 0 findings locally with 2.22), and both firmware builds in `espressif/idf:v6.0`. All firmware components now also build with `-Wall -Wextra -Werror`. |

### v2 hardware results

Final run: `logs/20261108T184102Z_run_all.log`, **15/15 PASS**. These are the first 10
scenarios again on the v2 firmware, plus:

| Scenario | Action | Expected | Observed on hardware |
|---|---|---|---|
| happy (v2) | `start 500` | final level within +-3 mL | PASS: 499.3 mL, 8.74 s; command RTT min/avg/max 45/50/90 ms |
| noisy_link | `noise 3000` (0.3 % of bytes) on both TX lines, `start 500` | damaged frames rejected, transfer still completes | PASS: COMPLETE, rejected frames on both sides; also passed 3/3 separate runs |
| noise_heavy | plant `noise 300000` (30 %) while pumping | controller faults, plant ends safe | PASS: `LINK_LOST: no valid telemetry for 310 ms`, pump OFF |
| ctl_hang | controller task `hang` while pumping | plant protects itself; watchdog resets controller; comes back IDLE | PASS: plant failsafe `no valid CMD for 350 ms`; controller rebooted 1.12 s after the hang with `reset_reason=TASK_WDT`, IDLE, pump off |
| plant_hang | plant task `hang` while pumping | controller faults; watchdog resets plant; plant boots NOT_ARMED | PASS: `LINK_LOST` after 310 ms of missing telemetry, plant `reset_reason=TASK_WDT`, re-armed only by a safe command |
| fault_log_persist | `stop` -> fault, `reset`, `reboot`, `faults` | fault from the previous boot still listed, boot counter +1 | PASS: `boot=10 ... OPERATOR_STOP` listed after reboot, boot 10 -> 11 |

`plant_reboot` now accepts `PLANT_RESTARTED` or `LINK_LOST`. Which one fires depends on
where in the 100 ms cycle the ~120 ms reboot falls (both seen on the bench). In both
cases the rebooted plant never ran the pump on its own.

### Failure modes and coverage (FMEA-style)

| Failure | Detected by | Reaction | Measured / designed time | Covered by |
|---|---|---|---|---|
| Telemetry lost (wire, plant hang, plant reboot) | controller: no valid TELEM for 300 ms | FAULT `LINK_LOST`, safe commands | 310 ms (on-device age) | link_telem_lost, plant_hang, wire_pull run 1, host `test_loop_telemetry_lost` |
| Commands lost (wire, controller hang/reboot) | plant: no valid CMD for 300 ms | plant **itself** pump off + valve close, NOT_ARMED; controller FAULT `PLANT_NO_CMD` (or `CMD_NOT_ACKED`) | 310-390 ms | link_cmd_lost, ctl_hang, wire_pull run 2, host tests |
| Plant alive but not applying commands | controller: newest acknowledged CMD older than 400 ms | FAULT `CMD_NOT_ACKED` | 300-520 ms (host) | host `test_loop_ack_frozen` only |
| Plant reboot faster than the link timeout | RESTARTED / NOT_ARMED flags | plant ignores run commands until a safe CMD; FAULT `PLANT_RESTARTED` | first telemetry after boot | plant_reboot, host `test_loop_fast_plant_reboot_mid_transfer` |
| Controller reboot | (plant keeps the last command < 300 ms) | controller boots IDLE, commands safe state, no resume | ~100 ms reboot | ctl_reboot, host `test_controller_reboot_does_not_resume` |
| Software hang (either board) | task watchdog 1 s | panic reset (`TASK_WDT`); the other board protects within ~0.3 s | ~1.1 s to reboot | ctl_hang, plant_hang |
| Bit errors on the link | CRC-16, LEN/TYPE consistency, range checks | frame dropped (exactly one frame per error), counters in `STAT` | per frame | noisy_link, noise_heavy, host bit-error/fuzz tests |
| Valve does not open | 1 s feedback deadline | FAULT `VALVE_NOT_OPEN`, pump never started | 1010 ms | valve_stuck_closed |
| Valve does not close | 1 s feedback deadline | FAULT `VALVE_NOT_CLOSED`, reset refused until closed | 1010 ms | valve_stuck_open |
| Pump does not deliver flow | flow < 20 mL/s after 1.5 s | FAULT `NO_FLOW` | 1.5 s | no_flow |
| Source runs low / leak | source <= 100 mL reserve | FAULT `SOURCE_LOW` | per telemetry frame | source_low |
| Not enough liquid to start | CHECK | FAULT `INSUFFICIENT_SOURCE`, pump never started | immediate | insufficient_source |
| Destination above high limit | dst > 950 mL | FAULT `OVERFILL` | per telemetry frame | **not tested** |
| Fault evidence lost by a reset | NVS fault log | last 8 faults survive reboot | - | fault_log_persist |

## Engineering challenge found on the bench

The first hardware run of `plant_reboot` showed that the ESP32-C3 reboots in about
**120 ms**, faster than the 300 ms link timeout. The controller never saw a link gap
and kept sending "pump ON, valve OPEN". The freshly booted plant **accepted it and
started acting on it**. The system ended in FAULT only by luck, with a misleading cause
(`VALVE_NOT_OPEN`, because the valve starts closed after boot). The host closed-loop
tests did not catch this, because they never modelled a reboot shorter than the timeout.

Fix: the plant is **NOT_ARMED** after boot and after any failsafe. It ignores run
commands until it sees one safe command, and it reports `RESTARTED`/`NOT_ARMED` in
telemetry. The controller latches `PLANT_RESTARTED` (or `PLANT_NOT_ARMED`). A
regression test `test_loop_fast_plant_reboot_mid_transfer` reproduces the exact case.
Other things found during the work, all documented in `docs/`: two wiring mistakes
located by a pulse-count wiring diagnostic, a model bug (flow through a closed valve),
and bugs in the bench script itself.

**Second finding (v2, after the window): one flipped bit cost up to three frames.**
With 0.5 % byte noise injected on the real wire, a transfer tripped `LINK_LOST` after
only 7 corrupted bytes. If the bit flip hit the LEN field (e.g. 12 -> 28), the decoder
waited for a 31-byte body and swallowed the next, intact frame. Two or three consecutive
frames were lost, which is exactly the 300 ms link timeout. Fix: LEN must be the length
of a known message type, and TYPE must match LEN. A single flipped bit cannot satisfy
both, so the decoder resynchronises on the very next SOF. The regression test
`test_single_bit_error_loses_only_that_frame` tries every bit position of a frame in a
4-frame stream. It fails on the old decoder ("2 frames lost") and passes on the new one
("1 frame lost").

## Known limitations

- The model is not physics: no pressure, no pipe volume, fixed nominal flow.
- **Overshoot** (fixed in v2). In the build-window version the destination ended about
  9 mL above target. v2 stops early when `dst + flow x 180 ms >= target`; the lead time
  is a fixed constant calibrated on this bench, not adapted online.
- Link timeouts are fixed at 300 ms (3 frames). There is no retransmission, which is
  acceptable because commands are state, not events, and are resent every 100 ms.
  Under sustained byte noise above ~1 % the link therefore trips `LINK_LOST` by design.
- Old or duplicate frames are counted (`seq_gaps`) but not rejected.
- `OVERFILL` (destination above the 950 mL high limit) has no test: the model cannot
  overfill while the controller works, and there is no injection hook for it yet.
- Bit errors are injected in firmware (`noise`); real electrical faults (EMI, short to
  supply, GND disconnection) were not tested. Both boards also share ground through the
  PC USB, so a missing GND wire would go unnoticed on this bench. No long runs (hours),
  no timing analysis under load.
- The NVS fault log is written from a low-priority task; `faults clear` from the console
  can race with a fault being written at the same moment (test hook only).
- The bench timing numbers come from one PC and one pair of boards.

## How AI was used, and what the author did

Built with Claude (Anthropic, Claude Code desktop app, model Opus 5.5) as an agent
with access to the terminal, the build tools and the serial ports of this bench only.

- **AI:** proposed the architecture, wrote all code in this repository (firmware,
  shared core, host tests, bench and bring-up tools) and the documentation drafts.
  Selected the two boards from the author's photo inventory, verified pinouts against
  ESP-IDF docs and vendor sources, ran builds, flashing and serial captures, wrote the
  wiring diagnostic, analysed the logs and fixed the defects found.
- **Author:** defined the task, constraints and safety rules for the agent; provided
  the board inventory; soldered and wired the boards (and re-wired after the diagnostic
  located two mistakes); visually confirmed the LED and OLED; performed the physical
  wire-pull test; reviewed the results; records the demo video (not yet recorded when
  this README was written).

The author should review and adjust this section to match their own review effort.

## Time

The build window started at 2026-11-08 18:23 after a separate preparation phase:
board selection, pin verification and toolchain setup. The preparation was committed
separately before the start. See [docs/timelog.md](docs/timelog.md) for what was done when.
