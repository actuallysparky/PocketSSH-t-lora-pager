#!/usr/bin/env python3
"""
Flash only one app partition on an ESP32-S3 device, leaving bootloader and
partition table untouched.

Typical usage:
  python3 misc/flash_app_partition.py --port /dev/cu.usbmodem201101 --bin build/PocketSSH.bin
  python3 misc/flash_app_partition.py --port /dev/cu.usbmodem201101 --partition ota_0
  python3 misc/flash_app_partition.py --port /dev/cu.usbmodem201101 --list
"""

from __future__ import annotations

import argparse
import csv
import os
import pathlib
import subprocess
import sys
import tempfile
from glob import glob
from typing import List, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Flash only a chosen app partition")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/cu.usbmodem201101)")
    parser.add_argument("--bin", default="build/PocketSSH.bin", help="Application binary path")
    parser.add_argument("--partition", default="", help="Target partition name (e.g. ota_0, factory)")
    parser.add_argument("--baud", type=int, default=460800, help="Flash baud rate (default: 460800)")
    parser.add_argument("--list", action="store_true", help="List app partitions and exit")
    return parser.parse_args()


def run(cmd: List[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def pick_python_with_esptool() -> str:
    candidates = [sys.executable]
    candidates.extend(sorted(glob(str(pathlib.Path.home() / ".espressif" / "python_env" / "*" / "bin" / "python"))))
    for py in candidates:
        try:
            subprocess.run(
                [py, "-m", "esptool", "version"],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            return py
        except Exception:
            continue
    raise RuntimeError("No python interpreter with esptool module was found")


def find_idf_path() -> pathlib.Path:
    env = os.environ.get("IDF_PATH")
    if env:
        p = pathlib.Path(env)
        if p.exists():
            return p
    fallback = pathlib.Path.home() / "esp" / "esp-idf"
    if fallback.exists():
        return fallback
    raise RuntimeError("IDF_PATH not set and ~/esp/esp-idf not found")


def read_partition_table_bin(py: str, port: str, out_path: pathlib.Path) -> None:
    cmd = [
        py,
        "-m",
        "esptool",
        "--chip",
        "esp32s3",
        "--port",
        port,
        "read_flash",
        "0x8000",
        "0x1000",
        str(out_path),
    ]
    print("Reading partition table from device...")
    print(" ".join(cmd))
    run(cmd)


def parse_partitions(idf_path: pathlib.Path, part_bin: pathlib.Path) -> List[Tuple[str, str, str, int, int, str]]:
    gen = idf_path / "components" / "partition_table" / "gen_esp32part.py"
    if not gen.exists():
        raise RuntimeError(f"gen_esp32part.py not found at {gen}")
    cmd = [sys.executable, str(gen), str(part_bin)]
    out = run(cmd).stdout

    rows: List[Tuple[str, str, str, int, int, str]] = []
    def parse_num(text: str) -> int:
        v = text.strip()
        if not v:
            return 0
        mul = 1
        suffix = v[-1].upper()
        if suffix in ("K", "M"):
            mul = 1024 if suffix == "K" else 1024 * 1024
            v = v[:-1]
        return int(v, 0) * mul

    reader = csv.reader(line for line in out.splitlines() if line.strip() and not line.strip().startswith("#"))
    for row in reader:
        if len(row) < 6:
            continue
        name = row[0].strip()
        ptype = row[1].strip()
        subtype = row[2].strip()
        offset = parse_num(row[3])
        size = parse_num(row[4])
        flags = row[5].strip() if len(row) > 5 else ""
        rows.append((name, ptype, subtype, offset, size, flags))
    return rows


def pick_partition(app_parts: List[Tuple[str, str, str, int, int, str]], requested: str) -> Tuple[str, int, int]:
    if not app_parts:
        raise RuntimeError("No app partitions found")

    if requested:
        for name, _, _, offset, size, _ in app_parts:
            if name == requested:
                return name, offset, size
        raise RuntimeError(f"Requested partition '{requested}' not found")

    # Default preference keeps factory/launcher style setups safer by targeting OTA slots first.
    pref = ("ota_0", "ota_1", "ota_2", "ota_3", "factory")
    by_name = {p[0]: p for p in app_parts}
    for name in pref:
        if name in by_name:
            p = by_name[name]
            return p[0], p[3], p[4]

    p = app_parts[0]
    return p[0], p[3], p[4]


def flash_app(py: str, port: str, baud: int, image: pathlib.Path, offset: int) -> None:
    cmd = [
        py,
        "-m",
        "esptool",
        "--chip",
        "esp32s3",
        "--port",
        port,
        "--baud",
        str(baud),
        "--before",
        "default_reset",
        "--after",
        "hard_reset",
        "write_flash",
        hex(offset),
        str(image),
    ]
    print("Flashing app partition only:")
    print(" ".join(cmd))
    run(cmd)


def main() -> int:
    args = parse_args()
    image = pathlib.Path(args.bin)
    if not image.exists():
        print(f"Binary not found: {image}", file=sys.stderr)
        return 2

    try:
        idf_path = find_idf_path()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 3
    try:
        py = pick_python_with_esptool()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 4

    with tempfile.TemporaryDirectory() as td:
        part_bin = pathlib.Path(td) / "partition-table.bin"
        try:
            read_partition_table_bin(py, args.port, part_bin)
            parts = parse_partitions(idf_path, part_bin)
        except subprocess.CalledProcessError as exc:
            print(exc.stdout, file=sys.stderr)
            return exc.returncode or 5
        except RuntimeError as exc:
            print(str(exc), file=sys.stderr)
            return 6

    app_parts = [p for p in parts if p[1] == "app"]
    print("Detected app partitions:")
    for name, _, subtype, offset, size, _ in app_parts:
        print(f"  - {name:10s} subtype={subtype:8s} offset={hex(offset)} size={hex(size)}")

    if args.list:
        return 0

    try:
        target_name, target_offset, target_size = pick_partition(app_parts, args.partition.strip())
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 7

    img_size = image.stat().st_size
    if img_size > target_size:
        print(
            f"Image too large for partition '{target_name}': image={img_size} bytes partition={target_size} bytes",
            file=sys.stderr,
        )
        return 8

    print(f"Target partition: {target_name} @ {hex(target_offset)} (size {hex(target_size)})")
    try:
        flash_app(py, args.port, args.baud, image, target_offset)
    except subprocess.CalledProcessError as exc:
        print(exc.stdout, file=sys.stderr)
        return exc.returncode or 9

    print("Done. Only app partition was flashed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
