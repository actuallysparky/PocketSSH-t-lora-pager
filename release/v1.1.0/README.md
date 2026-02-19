# PocketSSH T-Lora Pager v1.1.0

This release is a clean TPAGER build from source.

## Highlights
- `ssh_config` host aliases (`connect <alias>`, `hosts`)
- `wifi_config` profiles (`wifi`, `wifi <Network|SSID>`, `wifi auto`)
- Status bar context (Wi-Fi SSID + connected SSH host)
- Font size mode (`fontsize`, `fontsize big|normal`, plus `fontsize` in `ssh_config`)
- Encoder modifiers for no-touch hardware:
  - encoder: command history
  - `Alt` + encoder: input cursor left/right
  - `Caps` + encoder: terminal buffer scroll up/down
- Power management command: `shutdown` / `poweroff`

## Files
- `PocketSSH-T-Lora-Pager.bin` (app)
- `bootloader.bin`
- `partition-table.bin`
- `flash_args`
- `SHA256SUMS.txt`

## Flash command
From repo root:

```sh
python -m esptool --chip esp32s3 -b 460800 --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_freq 80m --flash_size 16MB \
  0x0 release/v1.1.0/bootloader.bin \
  0x8000 release/v1.1.0/partition-table.bin \
  0x10000 release/v1.1.0/PocketSSH-T-Lora-Pager.bin
```
