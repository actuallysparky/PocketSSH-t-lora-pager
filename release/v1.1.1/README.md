# PocketSSH T-Lora Pager v1.1.1

Patch release for the T-Pager port with PSRAM restoration, larger scrollback, SD/runtime stability fixes, and standardized TPager artifact naming.

## Files
- `PocketSSH-TPager.bin` (app)
- `bootloader.bin`
- `partition-table.bin`
- `flash_args`
- `SHA256SUMS.txt`

## Flash command
From repo root:

```sh
python -m esptool --chip esp32s3 -b 460800 --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_freq 80m --flash_size 16MB \
  0x0 release/v1.1.1/bootloader.bin \
  0x8000 release/v1.1.1/partition-table.bin \
  0x10000 release/v1.1.1/PocketSSH-TPager.bin
```

## SD staging contract
For launcher-based updates, stage the latest packaged app binary to:
- `/sdcard/PocketSSH-TPager.bin`
