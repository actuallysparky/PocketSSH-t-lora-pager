#!/usr/bin/env python3
"""
Push a local binary file to PocketSSH over USB serial using the `serialrx` protocol.

Device-side flow:
1) Boot PocketSSH on T-Pager.
2) Run this script from host.

Default behavior:
- Sends a control line (`__pocketctl serialrx <name>`) to trigger receiver mode.
- Waits for firmware readiness marker in serial logs.
- Streams BEGIN/DATA/END payload.
- Defaults to staging `/sdcard/PocketSSH-TPager.bin` on device.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time
import zlib

try:
    import serial  # type: ignore
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_BUILD_BIN = REPO_ROOT / "build" / "PocketSSH.bin"
DEFAULT_PACKAGED_BIN = REPO_ROOT.parent / "PocketSSH-TPager.bin"
DEFAULT_REMOTE_NAME = "PocketSSH-TPager.bin"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send binary to PocketSSH serialrx receiver")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/cu.usbmodem201101)")
    parser.add_argument(
        "--file",
        default=str(DEFAULT_PACKAGED_BIN),
        help=f"Path to binary to send (default: {DEFAULT_PACKAGED_BIN})",
    )
    parser.add_argument(
        "--chunk-bytes",
        type=int,
        default=128,
        help="Bytes per DATA frame before hex encoding (default: 128)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--tail-seconds",
        type=float,
        default=3.0,
        help="Seconds to read device output after transfer (default: 3)",
    )
    parser.add_argument(
        "--remote-name",
        default=DEFAULT_REMOTE_NAME,
        help=f"Destination filename on SD root (default: {DEFAULT_REMOTE_NAME})",
    )
    parser.add_argument(
        "--no-trigger",
        action="store_true",
        help="Do not send __pocketctl trigger; assume serialrx is already active",
    )
    parser.add_argument(
        "--trigger-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for serialrx readiness after trigger (default: 10)",
    )
    parser.add_argument(
        "--trigger-command",
        default="",
        help="Override control command (default: '__pocketctl serialrx <remote-name>')",
    )
    return parser.parse_args()


def wait_for_ready(ser: serial.Serial, timeout_s: float) -> bool:
    deadline = time.time() + max(0.1, timeout_s)
    window = ""
    markers = ("pocketctl serialrx_ready", "serialrx ready:", "serialrx: waiting for begin")
    while time.time() < deadline:
        chunk = ser.read(4096)
        if not chunk:
            continue
        text = chunk.decode(errors="ignore")
        sys.stdout.write(text)
        sys.stdout.flush()
        window = (window + text).lower()
        if len(window) > 4096:
            window = window[-4096:]
        if any(marker in window for marker in markers):
            return True
    return False


def main() -> int:
    args = parse_args()
    path = pathlib.Path(args.file)
    if not path.exists() and path.resolve() == DEFAULT_PACKAGED_BIN.resolve() and DEFAULT_BUILD_BIN.exists():
        print(f"Packaged binary not found at {path}; falling back to {DEFAULT_BUILD_BIN}")
        path = DEFAULT_BUILD_BIN
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        return 2

    data = path.read_bytes()
    total = len(data)
    crc32 = zlib.crc32(data) & 0xFFFFFFFF
    chunk = max(1, args.chunk_bytes)

    print(f"Sending {total} bytes from {path}")
    print(f"CRC32: {crc32:08x}")
    print(f"Remote filename: {args.remote_name}")

    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=5) as ser:
        # Give the USB CDC/JTAG bridge a brief settle.
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        if not args.no_trigger:
            trigger = args.trigger_command.strip() or f"__pocketctl serialrx {args.remote_name}"
            print(f"Triggering receiver: {trigger}")
            ser.write((trigger + "\n").encode("ascii"))
            ser.flush()
            if not wait_for_ready(ser, args.trigger_timeout):
                print("Timed out waiting for serialrx readiness marker.", file=sys.stderr)
                return 3
        else:
            print("No trigger mode: assuming serialrx is already active.")

        header = f"BEGIN {total} {crc32:08x}\n".encode("ascii")
        ser.write(header)

        sent = 0
        for offset in range(0, total, chunk):
            block = data[offset : offset + chunk]
            line = b"DATA " + block.hex().encode("ascii") + b"\n"
            ser.write(line)
            sent += len(block)

            # Pace slightly to avoid overwhelming the receiver and VFS.
            if (offset // chunk) % 32 == 0:
                ser.flush()
                time.sleep(0.003)

            if total > 0:
                pct = int((sent * 100) / total)
                if pct % 10 == 0 and (sent == len(block) or sent == total):
                    print(f"  {pct}% ({sent}/{total})")

        ser.write(b"END\n")
        ser.flush()
        print("Transfer sent. Tailing device output...")

        deadline = time.time() + max(0.0, args.tail_seconds)
        while time.time() < deadline:
            chunk_in = ser.read(4096)
            if not chunk_in:
                continue
            sys.stdout.write(chunk_in.decode(errors="ignore"))
            sys.stdout.flush()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
