# PocketSSH T-Lora Pager Variant

This repository is a focused fork of the original PocketSSH project:

- Source project: https://github.com/0015/PocketSSH
- Original authors and project maintainers retain full credit for PocketSSH.

This variant focuses on the LilyGO T-Lora Pager / T-Pager hardware and its related UX improvements.

## v1.1 Highlights (T-Lora Pager)
- T-Pager hardware bring-up and clean firmware packaging.
- OpenSSH-style host aliases from `/sdcard/ssh_keys/ssh_config`:
  - `connect <alias>`
  - `hosts`
- Stored Wi-Fi profiles from `/sdcard/ssh_keys/wifi_config`:
  - `wifi`
  - `wifi <Network|SSID>`
  - `wifi auto`
- Status bar context improvements:
  - active Wi-Fi SSID
  - connected SSH host/IP
- Terminal font-size mode:
  - `fontsize` toggle
  - `fontsize big|normal`
  - default from `ssh_config` via `fontsize`
- Encoder interaction model for no-touch T-Lora Pager hardware:
  - encoder: command history
  - `Alt` + encoder: input cursor left/right
  - `Caps` + encoder: terminal buffer scroll up/down
- Power command:
  - `shutdown` / `poweroff` enters deep sleep (wake by BOOT or encoder button)

## Build/Deploy Contract (T-Pager)
- Packaged artifact name is `PocketSSH-TPager.bin`.
- Every install cycle must also stage the same build to SD at `/sdcard/PocketSSH-TPager.bin`.
- Recommended host flow:
  1. `idf.py -DTPAGER_TARGET=ON -DTPAGER_DIAG=OFF build`
  2. `./misc/package_tpager_bin.sh`
  3. `~/.espressif/python_env/idf5.5_py3.14_env/bin/python ./misc/serial_push_bin.py --port /dev/cu.usbmodem201101`

For full project features, documentation, and history, see the upstream repository above.
