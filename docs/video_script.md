# 5-minute video script

Goal: show it working, explain what it does, walk through the architecture, explain
how AI was used, and describe one engineering challenge (the five points from the job
posting). Say plainly what is real hardware and what is simulated.

Setup before recording: both boards on the desk with the three link wires visible and
the OLED readable by the camera. On screen, run `python tools/dashboard.py` and open
http://127.0.0.1:8765. It shows tanks, command vs actual, the chart, the events of both
boards, and has buttons for every command, so no terminal typing is needed on camera.
Keep README and docs/traceability.md open in another window.
Good live demos with the dashboard: `start 500` (normal), `valve stuck` then
`start 500`, `plant hang (WDT)` during a transfer (watchdog reset + latched fault), and
`plant noise 0.3%` during a transfer (CRC counters grow, transfer still completes).
After a fault, `black box` prints the 4 s around it.

## 0:00-0:30 What it is

- "Two real microcontrollers on a UART link. The ESP32-S3 on the left is the
  controller; the ESP32-C3 on the right simulates the plant: two tanks, a pump and a
  valve."
- "The liquid, pump, valve and sensors are simulated in software on the C3. The state
  machine, the protocol, the timeouts and the safety reactions run on the real chips."
- Point at the three wires: TX->RX, RX<-TX, GND. No power between the boards.

## 0:30-1:30 Normal transfer (live)

- On the controller console: `start 500`.
- Show the LED going yellow (CHECK), then blue (TRANSFER), then green (COMPLETE).
- On the OLED: the valve X disappears, the pump box fills, dots run in the pipe, and the
  levels change.
- Read out the EVT lines: `CHECK ok`, `valve open confirmed after ~300 ms`,
  `target reached`, `COMPLETE`. Mention that the 9 mL overshoot is expected (see
  limitations).

## 1:30-2:30 Faults (live, pick two or three)

- Plant console: `fault valve_stuck 1`, controller `reset` then `start 500`. Result:
  `VALVE_NOT_OPEN` after 1 s, the pump never starts, the LED blinks red.
  `fault clear`, then `reset`.
- **Physical link loss:** start a transfer, then pull the S3 `1` -> C3 `4` wire.
  Point out that the plant stops the pump **by itself** (FS on the OLED), the
  controller latches a FAULT, and plugging the wire back does **not** resume anything.
  Only `reset` returns to IDLE.
- Optional: `fault valve_stuck 1` while pumping shows `VALVE_NOT_CLOSED`; `reset` is
  refused while the valve is open.

## 2:30-3:30 Architecture (README diagram)

- `common/transfer_core` is pure C99: protocol, controller FSM and plant model. The
  same code is compiled into both firmwares and into the PC unit tests.
- Protocol: SOF, LEN, TYPE, SEQ, payload, CRC16. Range checks, resync on junk,
  freshness timeouts on both sides, plant-side failsafe, arming.
- Non-blocking 10 ms loop on each board; FAULT is latched and reset only when the
  plant is observed safe.
- Tests: 27 host tests including closed-loop runs, then `tools/bench.py`, which drives
  both real boards through 10 scenarios and checks the boards' own log lines. 10/10 pass.

## 3:30-4:15 One engineering challenge

- "On the bench the ESP32-C3 rebooted in about 120 ms, faster than my 300 ms link
  timeout. The controller never noticed, kept sending pump ON, and the freshly booted
  plant obeyed it. The unit tests could not catch this, because they never modelled a
  reboot that short."
- "Fix: after boot or failsafe the plant is NOT_ARMED and ignores run commands until it
  receives a safe command, and it reports RESTARTED. The controller then latches
  PLANT_RESTARTED. I added a regression test for this exact case."
- Show the log lines: `NOT_ARMED: run command ... IGNORED` and
  `FAULT cause=PLANT_RESTARTED`.

## 4:15-5:00 How AI was used

- Be specific and honest (adjust to what you actually did): Claude (Claude Code) wrote
  the code, tests and bench tooling, chose the boards from photos of my inventory,
  checked pinouts, flashed the boards and analysed the logs. I set the requirements and
  safety rules, did all the soldering and wiring, checked the hardware visually, ran
  the physical tests and reviewed the result.
- Mention one place where the AI's first attempt was wrong and the hardware showed it:
  the wiring diagnostic, the fast-reboot defect, and bench-script bugs.
- Close with limitations: simple model, overshoot, no electrical noise tests.
