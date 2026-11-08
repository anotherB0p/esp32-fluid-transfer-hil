"""Wired firmware update of the controller (ESP32-S3) over its USB console, A/B + rollback.

  python tools/ota_update.py [--bin firmware/controller/build_esp32s3/transfer_controller.bin]

Sends the image in 384-byte base64 chunks, each acknowledged by the board ("OTA OK <n>"),
with the SHA-256 of the whole image announced up front and checked by the board before it
switches slots. Then follows the reboot: the new image must pass its self-test (compatible
plant, working link) within 10 s, otherwise the bootloader returns to the previous image.
The protocol logic lives in bench.ota_flow (also used by the OTA bench scenarios).
Everything is logged to logs/<UTC>_ota_update.log.
"""
from __future__ import annotations

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from bench import ROOT, Bench, ota_flow  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=str(ROOT / "firmware/controller/build_esp32s3/transfer_controller.bin"))
    a = ap.parse_args()
    image = pathlib.Path(a.bin).read_bytes()
    b = Bench("ota_update", echo=False)
    try:
        b.idle(0.5)
        res = ota_flow(b, image)
    finally:
        b.close()
    for k, v in res.items():
        print(f"{k}: {v}")
    print(f"log: {b.log_path}")
    return 0 if res.get("result") == "kept" else 1


if __name__ == "__main__":
    sys.exit(main())
