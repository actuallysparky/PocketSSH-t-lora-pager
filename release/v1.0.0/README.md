# PocketSSH T-Pager v1.0.0 (Clean Build)

This release build is credential-clean:
- no Wi-Fi SSID/password baked in
- no host/user baked in
- boot auto-test hook defaults to disabled

## Files
- `PocketSSH-tpager-v1.0.0-clean.bin` (app)
- `bootloader.bin`
- `partition-table.bin`
- `flash_args`
- `SHA256SUMS.txt`

## Flash command
From repo root:

```sh
python -m esptool --chip esp32s3 -b 460800 --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_freq 80m --flash_size 16MB \
  0x0 release/v1.0.0/bootloader.bin \
  0x8000 release/v1.0.0/partition-table.bin \
  0x10000 release/v1.0.0/PocketSSH-tpager-v1.0.0-clean.bin
```

Or use ESP-IDF with `release/v1.0.0/flash_args` as reference.
