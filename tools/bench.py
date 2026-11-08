"""Bench driver for the two-board transfer demo.

Opens both USB consoles at once (controller = ESP32-S3, plant = ESP32-C3), stamps
every received line with one PC monotonic clock, sends operator / fault-injection
commands and waits for expected log lines. Boards are found by USB serial number
(= chip MAC), not by COM number.

  python tools/bench.py list                        # scenarios
  python tools/bench.py run happy link_telem ...    # run scenarios (or 'all')
  python tools/bench.py send ctl "start 500" -w 12  # one command, watch both consoles
  python tools/bench.py watch -w 10                 # just watch both consoles

Every run writes logs/<UTC time>_<name>.log containing all lines from both boards.
A command written to a port proves only that the PC sent it; a scenario passes only
when the boards' own log lines show the expected reaction.
"""
from __future__ import annotations

import argparse
import datetime as dt
import pathlib
import queue
import re
import sys
import threading
import time

import serial
import serial.tools.list_ports

BOARDS = {
    "ctl": "AC:27:6E:CE:BC:50",   # A13, ESP32-S3, controller
    "plant": "08:92:72:87:09:CC",  # A09, ESP32-C3, plant simulator
}
ROOT = pathlib.Path(__file__).resolve().parent.parent
LOGDIR = ROOT / "logs"


class StepFailed(Exception):
    pass


def find_ports() -> dict[str, str]:
    found = {}
    for p in serial.tools.list_ports.comports():
        sn = (p.serial_number or "").upper()
        for name, mac in BOARDS.items():
            if sn == mac:
                found[name] = p.device
    missing = [n for n in BOARDS if n not in found]
    if missing:
        raise SystemExit(f"board(s) not connected: {missing}; seen: "
                         f"{[(p.device, p.serial_number) for p in serial.tools.list_ports.comports()]}")
    return found


class Bench:
    def __init__(self, log_name: str, echo: bool = True):
        self.t0 = time.monotonic()
        self.ports = find_ports()
        self.ser: dict[str, serial.Serial] = {}
        self.lines: list[tuple[float, str, str]] = []
        self.q: "queue.Queue[tuple[float, str, str]]" = queue.Queue()
        self.stop = threading.Event()
        self.echo = echo
        LOGDIR.mkdir(exist_ok=True)
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.log_path = LOGDIR / f"{stamp}_{log_name}.log"
        self.log = open(self.log_path, "w", encoding="utf-8")
        self.note(f"ports: {self.ports}")
        for name, dev in self.ports.items():
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = dev, 115200, 0.05
            s.dtr = s.rts = False  # do not reset the boards on open
            s.open()
            self.ser[name] = s
            threading.Thread(target=self._reader, args=(name,), daemon=True).start()
        # One read cursor per board: waiting for a plant line must not consume
        # controller lines that arrived in between (and vice versa).
        self.cursor = {name: 0 for name in BOARDS}

    def now(self) -> float:
        return time.monotonic() - self.t0

    def _write_log(self, t: float, src: str, text: str) -> None:
        line = f"[{t:9.3f}] {src:<5} {text}"
        self.log.write(line + "\n")
        self.log.flush()
        if self.echo:
            print(line, flush=True)

    def note(self, text: str) -> None:
        self._write_log(self.now(), "bench", text)

    def _reader(self, name: str) -> None:
        buf = b""
        while not self.stop.is_set():
            try:
                buf += self.ser[name].read(256)
            except serial.SerialException:
                time.sleep(0.2)  # board resetting / USB re-enumerating
                try:
                    self.ser[name].close()
                    self.ser[name].open()
                except serial.SerialException:
                    pass
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                t = self.now()
                text = raw.decode("utf-8", "replace").rstrip("\r")
                self.q.put((t, name, text))

    def _pump(self, timeout: float) -> None:
        try:
            t, name, text = self.q.get(timeout=timeout)
        except queue.Empty:
            return
        self.lines.append((t, name, text))
        self._write_log(t, name, text)

    def send(self, board: str, cmd: str) -> float:
        # Only lines that arrive after the command count as its reaction.
        while not self.q.empty():
            self._pump(0)
        for name in self.cursor:
            self.cursor[name] = len(self.lines)
        t = self.now()
        self._write_log(t, "bench", f"--> {board}: {cmd}")
        self.ser[board].write((cmd + "\n").encode())
        return t

    def wait(self, board: str, pattern: str, timeout: float) -> tuple[float, str]:
        """Wait for a NEW line (after the current cursor) from board matching pattern."""
        rx = re.compile(pattern)
        deadline = self.now() + timeout
        while True:
            while self.cursor[board] < len(self.lines):
                t, name, text = self.lines[self.cursor[board]]
                self.cursor[board] += 1
                if name == board and rx.search(text):
                    return t, text
            if self.now() > deadline:
                raise StepFailed(f"timeout {timeout}s waiting for {board}: /{pattern}/")
            self._pump(0.05)

    def absent(self, board: str, pattern: str, duration: float) -> None:
        """Fail if board prints a line matching pattern within duration."""
        rx = re.compile(pattern)
        end = self.now() + duration
        while self.now() < end:
            self._pump(0.05)
            while self.cursor[board] < len(self.lines):
                t, name, text = self.lines[self.cursor[board]]
                self.cursor[board] += 1
                if name == board and rx.search(text):
                    raise StepFailed(f"unexpected {board} line: {text}")

    def idle(self, seconds: float) -> None:
        end = self.now() + seconds
        while self.now() < end:
            self._pump(0.05)
        for name in self.cursor:
            self.cursor[name] = len(self.lines)

    def close(self) -> None:
        self.stop.set()
        time.sleep(0.1)
        for s in self.ser.values():
            s.close()
        self.log.close()


# ----------------------------------------------------------------------------- scenarios
# Each scenario returns a dict of measured values (for the results table).

def prepare(b: Bench, src: int = 800, dst: int = 100) -> None:
    """Common initial conditions: no injected faults, known levels, controller IDLE."""
    b.send("plant", "fault clear")
    b.wait("plant", r"all injected faults cleared", 2)
    b.send("plant", f"set src {src}")
    b.wait("plant", rf"source level set to {src}", 2)
    b.send("plant", f"set dst {dst}")
    b.wait("plant", rf"destination level set to {dst}", 2)
    b.idle(0.6)
    b.send("ctl", "status")
    t, line = b.wait("ctl", r"^STAT t=\d+ state=", 2)
    if "state=IDLE" not in line:
        b.send("ctl", "reset")
        b.wait("ctl", r"reset accepted", 3)
    b.send("ctl", "status")
    t, line = b.wait("ctl", r"^STAT t=\d+ state=IDLE.*link=OK", 2)


def sc_happy(b: Bench) -> dict:
    prepare(b)
    t0 = b.send("ctl", "start 500")
    b.wait("ctl", r"IDLE -> CHECK", 2)
    b.wait("ctl", r"CHECK -> TRANSFER phase=OPEN_VALVE", 3)
    b.wait("ctl", r"phase=PUMPING : valve open confirmed", 3)
    b.wait("ctl", r"phase=STOPPING : target reached", 20)
    t1, _ = b.wait("ctl", r"TRANSFER -> COMPLETE", 3)
    b.send("ctl", "status")
    _, diag = b.wait("ctl", r"^STAT diag ", 2)
    rtt = re.search(r"rtt_ms last=\d+ min=(\d+) avg=(\d+) max=(\d+)", diag)
    b.idle(1.2)
    b.send("plant", "status")
    _, st = b.wait("plant", r"^STAT t=", 2)
    if "pump cmd=0 act=0 valve cmd=0 act=0" not in st:
        raise StepFailed(f"plant not safe after COMPLETE: {st}")
    m = re.search(r"dst=([\d.]+)", st)
    dst = float(m.group(1)) if m else -1.0
    if abs(dst - 500.0) > 3.0:
        raise StepFailed(f"final level {dst} mL not within +-3 mL of target 500")
    return {"transfer_s": round(t1 - t0, 2), "plant_dst_ml": dst,
            "cmd_rtt_ms_min_avg_max": rtt.groups() if rtt else None}


def sc_insufficient_source(b: Bench) -> dict:
    prepare(b, src=300, dst=100)
    b.send("ctl", "start 500")
    _, line = b.wait("ctl", r"-> FAULT cause=INSUFFICIENT_SOURCE", 3)
    b.absent("plant", r"pump cmd=1", 1.5)
    b.send("ctl", "reset")
    b.wait("ctl", r"FAULT -> IDLE : reset accepted", 3)
    return {"fault_line": line.split(" : ", 1)[-1]}


def sc_valve_stuck_closed(b: Bench) -> dict:
    prepare(b)
    b.send("plant", "fault valve_stuck 1")
    b.wait("plant", r"valve_stuck=1 \(valve frozen CLOSED\)", 2)
    t0 = b.send("ctl", "start 500")
    b.wait("ctl", r"phase=OPEN_VALVE", 3)
    t1, line = b.wait("ctl", r"-> FAULT cause=VALVE_NOT_OPEN", 4)
    b.send("plant", "status")
    _, st = b.wait("plant", r"^STAT t=", 2)
    if "pump cmd=0 act=0" not in st or "act=0 | failsafe" not in st:
        raise StepFailed(f"pump/valve not safe: {st}")
    b.send("plant", "fault clear")
    b.wait("plant", r"cleared", 2)
    b.idle(0.5)
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {"detect_after_start_s": round(t1 - t0, 2), "fault_line": line.split(" : ", 1)[-1]}


def sc_no_flow(b: Bench) -> dict:
    prepare(b)
    b.send("plant", "fault pump_dead 1")
    b.wait("plant", r"pump_dead=1", 2)
    b.send("ctl", "start 500")
    t0, _ = b.wait("ctl", r"phase=PUMPING", 4)
    t1, line = b.wait("ctl", r"-> FAULT cause=NO_FLOW", 4)
    b.send("plant", "fault clear")
    b.wait("plant", r"cleared", 2)
    b.idle(0.8)
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {"detect_after_pump_cmd_s": round(t1 - t0, 2), "fault_line": line.split(" : ", 1)[-1]}


def sc_source_low(b: Bench) -> dict:
    prepare(b)
    b.send("ctl", "start 700")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.idle(2.0)
    b.send("plant", "fault leak 150")
    b.wait("plant", r"source leak 150", 2)
    _, line = b.wait("ctl", r"-> FAULT cause=SOURCE_LOW", 10)
    b.idle(1.2)
    b.send("plant", "status")
    _, st = b.wait("plant", r"^STAT t=", 2)
    if "pump cmd=0 act=0" not in st:
        raise StepFailed(f"pump not stopped: {st}")
    b.send("plant", "fault clear")
    b.wait("plant", r"cleared", 2)
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {"fault_line": line.split(" : ", 1)[-1]}


def sc_valve_stuck_open(b: Bench) -> dict:
    prepare(b)
    b.send("ctl", "start 300")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.send("plant", "fault valve_stuck 1")
    b.wait("plant", r"valve frozen OPEN", 2)
    _, line = b.wait("ctl", r"-> FAULT cause=VALVE_NOT_CLOSED", 12)
    b.idle(0.5)
    b.send("ctl", "reset")
    _, refused = b.wait("ctl", r"reset refused: valve still open", 2)
    b.send("plant", "fault clear")
    b.wait("plant", r"cleared", 2)
    b.idle(0.8)
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {"fault_line": line.split(" : ", 1)[-1], "first_reset": refused.strip()}


def sc_link_telem_lost(b: Bench) -> dict:
    """Plant goes silent (no TELEM, ignores CMD) during transfer."""
    prepare(b)
    b.send("ctl", "start 600")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.idle(1.0)
    b.send("plant", "mute 2500")
    t_mute, _ = b.wait("plant", r"TEST: link muted", 2)
    t_ctl, line = b.wait("ctl", r"-> FAULT cause=LINK_LOST", 2)
    t_fs, fs = b.wait("plant", r"FAILSAFE: no valid CMD", 2)
    b.wait("plant", r"link unmuted", 4)
    b.idle(2.0)
    b.send("ctl", "status")
    _, st = b.wait("ctl", r"^STAT t=", 2)
    if "state=FAULT" not in st or "link=OK" not in st:
        raise StepFailed(f"expected latched FAULT with link OK, got: {st}")
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {
        "ctl_detect_after_mute_ms": round((t_ctl - t_mute) * 1000),
        "plant_failsafe_after_mute_ms": round((t_fs - t_mute) * 1000),
        "ctl_detail": line.split(" : ", 1)[-1],
        "plant_detail": fs.split("FAILSAFE: ", 1)[-1],
        "after_restore": "FAULT latched, link OK, no auto-resume",
    }


def sc_link_cmd_lost(b: Bench) -> dict:
    """Controller stops sending CMD (telemetry still flows) during transfer."""
    prepare(b)
    b.send("ctl", "start 600")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.idle(1.0)
    b.send("ctl", "mute 2000")
    t_mute, _ = b.wait("ctl", r"TEST: CMD transmission muted", 2)
    t_fs, fs = b.wait("plant", r"FAILSAFE: no valid CMD", 2)
    # Two independent detectors race here: the plant's own failsafe flag (PLANT_NO_CMD)
    # and the controller's acknowledgement check (CMD_NOT_ACKED). Either is correct.
    t_ctl, line = b.wait("ctl", r"-> FAULT cause=(PLANT_NO_CMD|CMD_NOT_ACKED)", 2)
    b.wait("ctl", r"CMD transmission resumed", 3)
    b.idle(1.0)
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {
        "ctl_fault": line.split("-> FAULT ", 1)[-1],
        "plant_failsafe_after_mute_ms": round((t_fs - t_mute) * 1000),
        "ctl_fault_after_mute_ms": round((t_ctl - t_mute) * 1000),
        "plant_detail": fs.split("FAILSAFE: ", 1)[-1],
    }


def sc_ctl_reboot(b: Bench) -> dict:
    prepare(b)
    b.send("ctl", "start 600")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.idle(1.0)
    b.send("ctl", "reboot")
    t_boot, _ = b.wait("ctl", r"BOOT controller", 8)
    b.idle(1.5)
    b.send("ctl", "status")
    _, st = b.wait("ctl", r"^STAT t=", 3)
    if "state=IDLE" not in st:
        raise StepFailed(f"controller did not come back IDLE: {st}")
    b.send("plant", "status")
    _, pst = b.wait("plant", r"^STAT t=", 2)
    if "pump cmd=0 act=0" not in pst:
        raise StepFailed(f"plant pump not off after controller reboot: {pst}")
    return {"after_reboot": "controller IDLE, plant pump off, transfer not resumed"}


def sc_plant_reboot(b: Bench) -> dict:
    prepare(b)
    b.send("ctl", "start 600")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.idle(1.0)
    t0 = b.send("plant", "reboot")
    # The C3 is back in ~120 ms; whether the telemetry gap exceeds the 310 ms link timeout
    # depends on where in the 100 ms cycles the reboot falls. Both outcomes are correct as
    # long as the rebooted plant never runs the pump on its own (checked below).
    t1, line = b.wait("ctl", r"-> FAULT cause=(PLANT_RESTARTED|LINK_LOST)", 5)
    b.wait("plant", r"BOOT plant simulator", 8)
    b.wait("plant", r"ARMED: safe command received", 3)  # FAULT sends safe commands
    b.send("plant", "status")
    _, pst = b.wait("plant", r"^STAT t=", 2)
    if "pump cmd=0 act=0" not in pst:
        raise StepFailed(f"plant pump not off after reboot: {pst}")
    b.idle(2.0)
    b.send("ctl", "status")
    _, st = b.wait("ctl", r"^STAT t=", 2)
    if "state=FAULT" not in st:
        raise StepFailed(f"expected latched FAULT after plant reboot: {st}")
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {"ctl_fault_after_reboot_cmd_ms": round((t1 - t0) * 1000),
            "fault_line": line.split("-> FAULT ", 1)[-1]}


def _counter(line: str, name: str) -> int:
    m = re.search(rf"\b{name}=(\d+)", line)
    return int(m.group(1)) if m else -1


def sc_noisy_link(b: Bench) -> dict:
    """Bit-flip noise on both TX lines (0.3 % of bytes): CRC must reject damaged frames and
    the transfer must still complete because 3 consecutive losses are rare."""
    prepare(b)
    b.send("ctl", "status")
    _, st0 = b.wait("ctl", r"^STAT t=\d+ state=", 2)
    b.send("plant", "status")
    _, ps0 = b.wait("plant", r"^STAT t=", 2)
    b.send("ctl", "noise 3000")
    b.wait("ctl", r"noise 3000 ppm", 2)
    b.send("plant", "noise 3000")
    b.wait("plant", r"noise 3000 ppm", 2)
    try:
        b.send("ctl", "start 500")
        _, end = b.wait("ctl", r"-> (COMPLETE|FAULT cause=)", 20)
        b.idle(1.2)
        b.send("ctl", "status")
        _, st1 = b.wait("ctl", r"^STAT t=\d+ state=", 2)
        b.send("plant", "status")
        _, ps1 = b.wait("plant", r"^STAT t=", 2)
    finally:
        b.send("ctl", "noise 0")
        b.send("plant", "noise 0")
        b.idle(0.5)
    if "COMPLETE" not in end:
        raise StepFailed(f"transfer did not complete under 0.3 % byte noise: {end}")
    ctl_rej = sum(_counter(st1, k) - _counter(st0, k) for k in ("crc_err", "len_err", "range_err"))
    plant_rej = sum(_counter(ps1, k) - _counter(ps0, k) for k in ("crc_err", "len_err", "range_err"))
    # Bytes each side corrupted on its own TX (cumulative since that board booted).
    b.send("ctl", "status")
    _, d1 = b.wait("ctl", r"^STAT diag ", 2)
    ctl_tx_bad = _counter(d1, "tx_corrupted")
    plant_tx_bad = _counter(ps1, "tx_corrupted")
    if ctl_tx_bad + plant_tx_bad <= 0:
        raise StepFailed("no bytes were corrupted - noise injection did not happen")
    if ctl_rej + plant_rej <= 0:
        raise StepFailed("bytes were corrupted but no frame was rejected")
    return {"result": "COMPLETE",
            "plant_tx_corrupted_bytes_since_boot": plant_tx_bad, "ctl_rejected_frames": ctl_rej,
            "ctl_tx_corrupted_bytes_since_boot": ctl_tx_bad, "plant_rejected_frames": plant_rej,
            "ctl_ok_frames": _counter(st1, "rx_ok") - _counter(st0, "rx_ok")}


def sc_noise_heavy(b: Bench) -> dict:
    """30 % of plant TX bytes corrupted: nearly no valid telemetry -> controller must fault
    and command the safe state; plant (still receiving clean CMD) obeys."""
    prepare(b)
    b.send("ctl", "start 600")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.idle(1.0)
    b.send("plant", "noise 300000")
    try:
        _, line = b.wait("ctl", r"-> FAULT cause=", 5)
        b.idle(1.2)
    finally:
        b.send("plant", "noise 0")
        b.idle(0.8)
    b.send("plant", "status")
    _, pst = b.wait("plant", r"^STAT t=", 2)
    if "pump cmd=0 act=0" not in pst:
        raise StepFailed(f"pump not off: {pst}")
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {"ctl_fault": line.split("-> FAULT ", 1)[-1], "plant_pump": "OFF"}


def sc_ctl_hang(b: Bench) -> dict:
    """Controller task hangs during transfer: the plant must protect itself, the task
    watchdog must reset the controller, and the controller must come back IDLE."""
    prepare(b)
    b.send("ctl", "start 600")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.idle(1.0)
    t0 = b.send("ctl", "hang")
    b.wait("ctl", r"TEST: control task hangs", 2)
    t_fs, fs = b.wait("plant", r"FAILSAFE: no valid CMD", 2)
    t_boot, boot = b.wait("ctl", r"BOOT controller .*reset_reason=(\w+)", 10)
    reason = re.search(r"reset_reason=(\w+)", boot).group(1)
    if reason != "TASK_WDT":
        raise StepFailed(f"expected reset_reason=TASK_WDT, got {reason}")
    b.idle(1.5)
    b.send("ctl", "status")
    _, st = b.wait("ctl", r"^STAT t=\d+ state=", 3)
    if "state=IDLE" not in st:
        raise StepFailed(f"controller not IDLE after watchdog reset: {st}")
    b.send("plant", "status")
    _, pst = b.wait("plant", r"^STAT t=", 2)
    if "pump cmd=0 act=0" not in pst:
        raise StepFailed(f"pump not off: {pst}")
    return {"plant_failsafe_after_hang_ms": round((t_fs - t0) * 1000),
            "ctl_reboot_after_hang_ms": round((t_boot - t0) * 1000), "reset_reason": reason,
            "plant_detail": fs.split("FAILSAFE: ", 1)[-1]}


def sc_plant_hang(b: Bench) -> dict:
    """Plant task hangs during transfer: the controller must fault on missing telemetry,
    the watchdog must reset the plant, and the rebooted plant must come up NOT_ARMED."""
    prepare(b)
    b.send("ctl", "start 600")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.idle(1.0)
    t0 = b.send("plant", "hang")
    b.wait("plant", r"TEST: plant task hangs", 2)
    t1, line = b.wait("ctl", r"-> FAULT cause=", 3)
    _, boot = b.wait("plant", r"BOOT plant simulator .*reset_reason=(\w+)", 10)
    reason = re.search(r"reset_reason=(\w+)", boot).group(1)
    if reason != "TASK_WDT":
        raise StepFailed(f"expected reset_reason=TASK_WDT, got {reason}")
    b.wait("plant", r"ARMED: safe command received", 3)
    b.idle(1.0)
    b.send("ctl", "status")
    _, st = b.wait("ctl", r"^STAT t=\d+ state=", 2)
    if "state=FAULT" not in st:
        raise StepFailed(f"expected latched FAULT: {st}")
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return {"ctl_fault": line.split("-> FAULT ", 1)[-1],
            "ctl_fault_after_hang_ms": round((t1 - t0) * 1000), "plant_reset_reason": reason}


def sc_fault_log_persist(b: Bench) -> dict:
    """A fault recorded before a reboot must still be listed after it (NVS)."""
    prepare(b)
    b.send("ctl", "start 600")
    b.wait("ctl", r"phase=PUMPING", 4)
    b.send("ctl", "stop")
    b.wait("ctl", r"-> FAULT cause=OPERATOR_STOP", 2)
    b.idle(0.5)
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    b.send("ctl", "status")
    _, d0 = b.wait("ctl", r"^STAT diag boot=(\d+)", 2)
    boot_before = _counter(d0, "boot")
    b.send("ctl", "reboot")
    b.wait("ctl", r"BOOT controller", 8)
    b.idle(1.0)
    b.send("ctl", "faults")
    _, last = b.wait("ctl", rf"FAULTLOG #\d+ boot={boot_before} .*OPERATOR_STOP", 3)
    b.send("ctl", "status")
    _, d1 = b.wait("ctl", r"^STAT diag boot=(\d+)", 2)
    if _counter(d1, "boot") != boot_before + 1:
        raise StepFailed(f"boot counter did not increment: {boot_before} -> {d1}")
    return {"boot_before": boot_before, "boot_after": _counter(d1, "boot"),
            "persisted": last.split("FAULTLOG ", 1)[-1]}


def sc_wire_pull(b: Bench) -> dict:
    """MANUAL: operator physically unplugs one link wire during a long transfer."""
    # Keep transfers running (refill and restart after COMPLETE) for up to 3 minutes,
    # so the operator can pull the wire whenever ready.
    b.note(">>> OPERATOR: unplug ONE link wire during the transfer (up to 3 min) <<<")
    deadline = b.now() + 180
    line = None
    while line is None:
        if b.now() > deadline:
            raise StepFailed("no FAULT within 3 minutes (wire not pulled?)")
        prepare(b, src=1000, dst=0)
        b.send("ctl", "start 900")
        b.wait("ctl", r"phase=PUMPING", 4)
        _, ev = b.wait("ctl", r"-> (FAULT cause=|COMPLETE)", 30)
        if "FAULT" in ev:
            line = ev
        else:
            b.note("transfer completed without a fault; restarting")
    res = {"ctl_fault": line.split("-> FAULT ", 1)[-1]}
    try:
        _, fs = b.wait("plant", r"FAILSAFE: no valid CMD", 1.5)
        res["plant"] = fs.split("FAILSAFE: ", 1)[-1]
    except StepFailed:
        res["plant"] = "no FAILSAFE line (plant still receives CMD; controller commands safe state)"
    b.idle(1.5)
    b.send("plant", "status")
    _, pst = b.wait("plant", r"^STAT t=", 2)
    res["plant_pump"] = "OFF" if "pump cmd=0 act=0" in pst else "ON (!)"
    b.note(">>> OPERATOR: plug the wire back in (waiting up to 120 s for link OK) <<<")
    deadline = b.now() + 120
    while True:
        b.send("ctl", "status")
        _, st = b.wait("ctl", r"^STAT t=", 2)
        if "link=OK" in st and "plant_failsafe=0" in st:
            break
        if b.now() > deadline:
            raise StepFailed("link did not come back within 120 s")
        b.idle(1.0)
    b.idle(2.0)
    b.send("ctl", "status")
    _, st = b.wait("ctl", r"^STAT t=", 2)
    if "state=FAULT" not in st:
        raise StepFailed(f"expected latched FAULT after reconnect: {st}")
    res["after_reconnect"] = "FAULT latched, no auto-resume"
    b.send("ctl", "reset")
    b.wait("ctl", r"reset accepted", 3)
    return res


MANUAL = {"wire_pull": sc_wire_pull}

SCENARIOS = {
    "happy": sc_happy,
    "insufficient_source": sc_insufficient_source,
    "valve_stuck_closed": sc_valve_stuck_closed,
    "no_flow": sc_no_flow,
    "source_low": sc_source_low,
    "valve_stuck_open": sc_valve_stuck_open,
    "link_telem_lost": sc_link_telem_lost,
    "link_cmd_lost": sc_link_cmd_lost,
    "ctl_reboot": sc_ctl_reboot,
    "plant_reboot": sc_plant_reboot,
    "noisy_link": sc_noisy_link,
    "noise_heavy": sc_noise_heavy,
    "ctl_hang": sc_ctl_hang,
    "plant_hang": sc_plant_hang,
    "fault_log_persist": sc_fault_log_persist,
}


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    r = sub.add_parser("run")
    r.add_argument("names", nargs="+")
    r.add_argument("-q", "--quiet", action="store_true")
    s = sub.add_parser("send")
    s.add_argument("board", choices=list(BOARDS))
    s.add_argument("text")
    s.add_argument("-w", "--watch", type=float, default=3.0)
    w = sub.add_parser("watch")
    w.add_argument("-w", "--watch", type=float, default=5.0)
    a = ap.parse_args()

    if a.cmd == "list":
        print("\n".join(SCENARIOS))
        print("manual (operator action needed):", ", ".join(MANUAL))
        return 0
    if a.cmd in ("send", "watch"):
        b = Bench(a.cmd)
        try:
            if a.cmd == "send":
                b.send(a.board, a.text)
            b.idle(a.watch)
        finally:
            b.close()
        print(f"log: {b.log_path}")
        return 0

    names = list(SCENARIOS) if a.names == ["all"] else a.names
    SCENARIOS.update(MANUAL)  # manual ones run only when named explicitly
    results = []
    b = Bench("run_" + "_".join(names) if len(names) < 4 else "run_all", echo=not a.quiet)
    try:
        b.idle(0.5)
        for name in names:
            b.note(f"===== scenario {name} =====")
            try:
                res = SCENARIOS[name](b)
                results.append((name, "PASS", res))
                b.note(f"===== {name}: PASS {res}")
            except StepFailed as e:
                results.append((name, "FAIL", {"error": str(e)}))
                b.note(f"===== {name}: FAIL {e}")
                b.idle(1.0)
    finally:
        b.close()
    print("\nRESULTS")
    for name, verdict, res in results:
        print(f"  {verdict:4} {name:22} {res}")
    print(f"log: {b.log_path}")
    return 0 if all(v == "PASS" for _, v, _ in results) else 1


if __name__ == "__main__":
    sys.exit(main())
