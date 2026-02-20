#pragma once

#include <cinttypes>

#include "esp_err.h"

namespace tpager {

struct SdDiagStats {
    bool mounted = false;
    bool keys_dir_created = false;
    int32_t dir_entries = 0;
    int32_t pem_files = 0;
};

// Mount /sdcard via SDSPI on T-Pager shared SPI bus and count keys in /sdcard/ssh_keys.
esp_err_t sd_mount_and_scan_keys(SdDiagStats *stats);
esp_err_t sd_unmount();
bool sd_is_mounted();

}  // namespace tpager
