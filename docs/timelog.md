# Time log

Preparation is logged separately from the 3-hour build window.
The window starts only when the owner says "START".

## Preparation (not part of the 3 hours)

| Time (local) | Activity |
|--------------|----------|
| 2026-11-07 ~23:00 | Read the job posting; checked the environment (Windows 11, ESP-IDF v6.0 with esp32s3/esp32c3, esptool 5.4.0, pyserial 3.5). |
| 2026-11-07 23:37 | Reviewed the board inventory photos; selected A13 (ESP32-S3-Zero) and A09 (ESP32-C3 0.42" OLED); drafted the pin table (`docs/hardware.md`). |

| 2026-11-07 23:40 | Owner connected A13 then A09; identified COM7 / COM8 with read-only esptool `flash-id` and `get-security-info`. |
| 2026-11-07 ~00:00 | Wrote and flashed the bring-up probe (`tools/bringup`): WS2812 cycle on S3, I2C scan + OLED all-on on C3. Fixed two helper-script issues (export.ps1 stderr, positional argument binding). |
| 2026-11-08 ~00:10 | Owner visually confirmed the RGB LED cycle (A13) and the lit OLED (A09). Added a UART1 PING test to the probe, flashed both boards; logs before wiring show no RX traffic (pull-up idle, no noise). |
| 2026-11-08 ~00:20 | First wiring attempt: S3 dropped off USB repeatedly (owner: wrong pads used). Owner re-wired. Link test: no bytes received in either direction although both boards ran without resets. |
| 2026-11-08 ~00:35 | Added a wiring diagnostic (`-DBRINGUP_PINMAP=1`: S3 drives GPIO1/2/43/44 with distinct square waves; C3 samples pads as inputs only, flashed first to avoid output contention). Result: C3 GPIO20 sees S3 GPIO43 (20 edges/2 s), C3 GPIO21 sees S3 GPIO44 (2 edges/2 s); nothing on C3 GPIO3/4. The wires are on the TX/RX-labelled UART0 pads, crossed correctly, not on the pads in `docs/hardware.md`. |
| 2026-11-08 ~00:50 | Owner re-wired "by the table"; first diagnostic saw nothing on C3 GPIO3/4. Extended diagnostic (S3 pad sends N pulses = its GPIO number, weakest drive strength; C3 listens on all pads): C3 GPIO2 receives S3 GPIO2; S3 GPIO1 not seen on any C3 pad. Static highs on C3 GPIO5/6 (OLED I2C pull-ups) and GPIO8/9 (board pull-ups) are expected. |
| 2026-11-08 ~01:00 | Owner moved the wires to the bottom row of the C3. Pin map: S3 GPIO1 -> C3 GPIO4, S3 GPIO2 -> C3 GPIO3 (matches the table). Flashed the link test (S3 first, to avoid output contention): PING received in both directions, loopback self-test PASS on both. **Preparation complete.** |
| 2026-11-08 ~01:15 | With the owner's approval installed a host C compiler for PC unit tests: `scoop install gcc` -> GCC 15.2.0 (MinGW-w64, nuwen distro), user-level, no admin. Scoop also updated itself and its buckets, and installed `7zip` 26.06 as a dependency. Verified: a test program compiles with `-std=c11 -Wall -Wextra -Werror` and runs. |

## Build window (3 hours)

**START: 2026-11-08 18:23:00** (owner said "START"). Preparation was committed just before
the start as `5f12dd9`.

| Time (local) | Activity |
|--------------|----------|
| 18:23 | Start. Common core: protocol, plant model, controller FSM. |
| ~18:28 | Host tests (gcc): 24 tests; one failure exposed a model bug (flow kept decaying through a closed valve), fixed. |
| ~18:33 | Controller (S3) and plant (C3) firmware, OLED + RGB LED; both build without warnings; flashed. |
| 18:35 | **First end-to-end transfer on both boards: PASS** (9.0 s, 100 -> 509 mL for target 500). |
| 18:36 | `tools/bench.py` + 10 scenarios: 7 PASS, 2 FAIL. One failure was a bench-script bug (shared read cursor across boards). The other was a real defect: the C3 reboots in ~120 ms, faster than the 300 ms link timeout, and the freshly booted plant obeyed the controller's still-running RUN command. |
| ~18:39 | Fix: plant arming (NOT_ARMED after boot/failsafe, run commands ignored until a safe CMD) + RESTARTED flag -> controller fault PLANT_RESTARTED. 3 new host tests (27 tests, 122 checks, all pass). Rebuilt and reflashed both boards. |
| 18:40 | Hardware re-run of all 10 scenarios: **10/10 PASS** (`logs/20261108T174015Z_run_all.log`). |
| 18:42 | Owner confirmed the RGB LED and the OLED tank picture visually. |
| 18:43-18:50 | Manual wire-pull scenario. Two runs failed because of bench-script issues (a COMPLETE line consumed while waiting for FAULT; then a 3-minute window with no wire pulled). Script fixed. README, video script and Russian guide written meanwhile. |
| 18:51 | Manual wire-pull: PASS (`logs/20261108T175106Z_run_wire_pull.log`). Plant failsafe after 320 ms without CMD, controller LINK_LOST after 310 ms without TELEM, FAULT latched after the link returned, reset accepted. Both directions dropped for < 1 s (owner: both wires came off by accident). |
| 18:52 | Final host test run: 27 tests, 122 checks, 0 failed. |
| 18:52-18:53 | Manual wire-pull repeated with only the CMD wire removed (~3.9 s): PASS, controller cause `PLANT_NO_CMD`, plant failsafe after 310 ms (`logs/20261108T175154Z_run_wire_pull.log`). Work in the build window ends here (~30 min of the 3 h budget used); everything after this is listed separately below. |

## After the build window

Nothing yet.
