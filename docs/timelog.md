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

Not started.
