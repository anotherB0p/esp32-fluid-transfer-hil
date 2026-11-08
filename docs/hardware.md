# Hardware and wiring

Status: **confirmed on hardware** on 2026-11-08: chip ID, on-board LED/OLED,
wire-by-wire pin map and a bidirectional UART1 PING test (see the end of this file).

## Boards selected (from the owner's inventory photos)

| Role | Inventory ID | Board | Chip (marking) | USB |
|------|--------------|-------|----------------|-----|
| Controller | A13 | ESP32-S3-Zero form factor (silkscreen "ESP32-S3-Zero") | ESP32-S3FH4R2 (4 MB flash, 2 MB PSRAM in package) | native USB Serial/JTAG |
| Plant model | A09 | ESP32-C3 board with built-in 0.42" 72x40 OLED (01Space-style) | ESP32-C3 FH4 (4 MB flash in package) | native USB Serial/JTAG |

Why these two: both are supported by the installed ESP-IDF v6.0, both have native
USB (no separate USB-UART adapter or programmer needed), and each has an on-board
indicator that helps the video: an RGB LED on the controller, an OLED on the plant model.
Other boards in the inventory (STM32G431/G030/G031, RP2040, CH592, ESP8266, ESP32-P4)
were not chosen, to avoid toolchain setup inside the time budget.

## On-hardware identification (esptool v5.4.0, read-only commands `flash-id`, `get-security-info`)

| Board | Port (this PC) | USB ID | esptool report |
|-------|----------------|--------|----------------|
| A13 controller | COM7 | 303A:1001, serial `AC:27:6E:CE:BC:50` | ESP32-S3 (QFN56) rev v0.2; embedded flash 4 MB (XMC, quad); embedded PSRAM 2 MB (AP_3v3, quad); 40 MHz crystal; USB-Serial/JTAG; Secure Boot disabled; Flash Encryption disabled; no keys burned |
| A09 plant | COM8 | 303A:1001, serial `08:92:72:87:09:CC` | ESP32-C3 (QFN32) rev v0.4; embedded flash 4 MB (XMC); 40 MHz crystal; USB-Serial/JTAG; Secure Boot disabled; Flash Encryption disabled; no keys burned |

Bring-up probe (`tools/bringup`, ESP-IDF v6.0), flashed 2026-11-07, logs read over USB:

| Board | Flash result | Log evidence | Visual check by owner |
|-------|--------------|--------------|------------------------|
| A13 (COM7) | 3 regions written, "Hash of data verified" | `WS2812 on GPIO21 set to RED/GREEN/BLUE` every 1 s | confirmed by owner ("both work") |
| A09 (COM8) | 3 regions written, "Hash of data verified" | `I2C device ACK at 0x3C (SDA=GPIO5 SCL=GPIO6)`, `SSD1306 at 0x3C: all-pixels-on command -> ESP_OK` | confirmed by owner ("both work") |

A log line proves the firmware ran the code. It does not prove that the LED/OLED
lit up: that needs the owner's visual confirmation.

COM numbers are assigned by Windows and can change. The USB serial number (equal to
the chip MAC) is the stable identifier.

## Pins that must NOT be used

| Chip | Avoid | Reason | Source |
|------|-------|--------|--------|
| ESP32-S3 | GPIO0, 3, 45, 46 | strapping pins | ESP-IDF v6.0 `docs/en/api-reference/peripherals/gpio/esp32s3.inc` |
| ESP32-S3 | GPIO19, 20 | USB Serial/JTAG (our console + flashing) | same |
| ESP32-S3 | GPIO26-37 | in-package flash/PSRAM | same + Waveshare wiki ("GPIO33~37 reserved for Octal PSRAM") |
| ESP32-S3 | GPIO43, 44 | UART0: ROM boot log on TX at every reset | Waveshare wiki ("TX is GPIO43, RX is GPIO44") |
| ESP32-S3 | GPIO21 | on-board WS2812 RGB LED | Waveshare wiki ("uses GPIO21 to connect with WS2812") |
| ESP32-C3 | GPIO2, 8, 9 | strapping pins (GPIO9 = BOOT) | ESP-IDF v6.0 `.../gpio/esp32c3.inc` |
| ESP32-C3 | GPIO12-17 | SPI flash | same |
| ESP32-C3 | GPIO18, 19 | USB Serial/JTAG | same |
| ESP32-C3 | GPIO20, 21 | UART0: ROM boot log on TX at every reset | ESP32-C3 default UART0 pins; silkscreen RX/TX |
| ESP32-C3 | GPIO5, 6 | on-board OLED I2C SDA/SCL | 01Space/ESP32-C3-0.42LCD examples: `#define SDA_PIN 5`, `#define SCL_PIN 6` |

## Inter-board link: UART1, 115200 8N1, 3.3 V logic on both sides

| # | Controller (S3) pad | S3 GPIO | Function | Wire | Plant (C3) pad | C3 GPIO | Function |
|---|---------------------|---------|----------|------|----------------|---------|----------|
| 1 | `1` | GPIO1 | UART1 TX | -> | `4` | GPIO4 | UART1 RX |
| 2 | `2` | GPIO2 | UART1 RX | <- | `3` | GPIO3 | UART1 TX |
| 3 | `GND` | GND | ground reference | -- | `GD` | GND | ground reference |

Not connected: `3V3`, `5V` / VBUS. Each board is powered from its own USB cable to
the same PC. Both sides are 3.3 V logic, so no level shifting is needed. No external
loads (pumps, valves, relays) are connected: actuators exist only in software.

RX pins use the internal pull-up, so a disconnected wire reads as idle (high)
rather than as noise.

## Wiring verification (2026-11-08)

1. **Pin map** (`tools/bringup`, built with `-DBRINGUP_PINMAP=1`). Each free S3 pad
   sends a burst of N pulses, where N is its GPIO number, at the weakest drive
   strength. C3 listens on all its pads as inputs only. Result on the final wiring:
   `C3 GPIO4 received a burst of 1 pulses -> wired to S3 GPIO1` and
   `C3 GPIO3 received a burst of 2 pulses -> wired to S3 GPIO2`; no other C3 pad toggled.
   Earlier attempts found the wires on the TX/RX-labelled UART0 pads, then on C3 pad `2`
   (top row) instead of `3`/`4` (bottom row). The diagnostic located both mistakes
   without guessing.
2. **Link test** (default build). Each board sends `PING <target> <n>` once per second
   on UART1. S3 log: `link RX: "PING esp32c3 1..5"`. C3 log: `link RX: "PING esp32s3 1..6"`.
   Both boards: `link internal loopback self-test: PASS`.
3. **Observation for the protocol design.** S3 received `LOOPBACKPING esp32c3 0`:
   the C3 internal-loopback self-test also drove the real TX pin, so the test bytes
   reached the other board. The link therefore has to tolerate stray bytes; framing
   with length and CRC must resynchronise. The same applies to garbage bytes from a
   board that is resetting.

GND is not covered by these tests: both boards also share ground through the PC's USB.
The owner reported that the wiring follows the table, including GND-GD. This is not measured.

## Fallback

If the wired link is unavailable, both boards can be bridged through the PC (two
USB-CDC ports plus a Python relay). The link then depends on the PC, and the
documentation must say so.
