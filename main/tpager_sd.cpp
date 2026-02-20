#include "tpager_sd.hpp"

#include <cstring>
#include <dirent.h>
#include <strings.h>
#include <sys/stat.h>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tpager_xl9555.hpp"

namespace tpager {
namespace {

constexpr const char *kTag = "tpager_sd";
constexpr const char *kMountPoint = "/sdcard";
constexpr const char *kKeysDir = "/sdcard/ssh_keys";

constexpr spi_host_device_t kSpiHost = SPI2_HOST;
constexpr gpio_num_t kSpiMosi = GPIO_NUM_34;
constexpr gpio_num_t kSpiMiso = GPIO_NUM_33;
constexpr gpio_num_t kSpiSclk = GPIO_NUM_35;
constexpr gpio_num_t kSdCs = GPIO_NUM_21;
constexpr gpio_num_t kDisplayCs = GPIO_NUM_38;

sdmmc_card_t *g_card = nullptr;
bool g_mounted = false;

bool has_pem_extension(const char *name)
{
    if (name == nullptr) {
        return false;
    }
    const size_t len = std::strlen(name);
    if (len < 5) {
        return false;
    }
    return strcasecmp(name + len - 4, ".pem") == 0;
}

esp_err_t ensure_spi_bus()
{
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = kSpiMosi;
    bus_cfg.miso_io_num = kSpiMiso;
    bus_cfg.sclk_io_num = kSpiSclk;
    bus_cfg.quadwp_io_num = GPIO_NUM_NC;
    bus_cfg.quadhd_io_num = GPIO_NUM_NC;
    bus_cfg.max_transfer_sz = 4096;

    esp_err_t ret = spi_bus_initialize(kSpiHost, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(kTag, "SPI bus already initialized");
        return ESP_OK;
    }
    return ret;
}

esp_err_t scan_keys_dir(SdDiagStats *stats)
{
    DIR *dir = opendir(kKeysDir);
    if (dir == nullptr) {
        ESP_LOGW(kTag, "keys dir missing, creating %s", kKeysDir);
        if (mkdir(kKeysDir, 0755) == 0) {
            stats->keys_dir_created = true;
            dir = opendir(kKeysDir);
        }
    }
    if (dir == nullptr) {
        ESP_LOGW(kTag, "unable to open keys dir after create attempt");
        return ESP_OK;
    }

    struct dirent *entry = nullptr;
    while ((entry = readdir(dir)) != nullptr) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        stats->dir_entries++;
        if (has_pem_extension(entry->d_name)) {
            stats->pem_files++;
            ESP_LOGI(kTag, "found key file: %s", entry->d_name);
        }
    }
    closedir(dir);
    return ESP_OK;
}

esp_err_t try_mount_card(sdmmc_card_t **out_card)
{
    if (out_card == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // Shared SPI contract: keep both CS lines deasserted before SDSPI probing.
    gpio_reset_pin(kDisplayCs);
    gpio_set_direction(kDisplayCs, GPIO_MODE_OUTPUT);
    gpio_set_level(kDisplayCs, 1);

    gpio_reset_pin(kSdCs);
    gpio_set_direction(kSdCs, GPIO_MODE_OUTPUT);
    gpio_set_level(kSdCs, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // Launcher transitions can leave the bus in marginal state; probe at lower speed.
    host.max_freq_khz = 4000;
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = kSdCs;
    slot_cfg.host_id = kSpiHost;

    return esp_vfs_fat_sdspi_mount(kMountPoint, &host, &slot_cfg, &mount_cfg, out_card);
}

void maybe_power_cycle_sd()
{
    tpager::Xl9555 xl = {};
    (void)tpager::xl9555_init(&xl, I2C_NUM_0, 0x20, pdMS_TO_TICKS(20));
    if (tpager::xl9555_probe(xl) != ESP_OK) {
        return;
    }
    if (tpager::xl9555_set_dir(xl, tpager::XL9555_PIN_SD_POWER_EN, true) != ESP_OK) {
        return;
    }

    // Recovery contract: force a short rail cycle so SDSPI can re-probe card state
    // after launcher/app transitions.
    (void)tpager::xl9555_write_pin(xl, tpager::XL9555_PIN_SD_POWER_EN, false);
    vTaskDelay(pdMS_TO_TICKS(25));
    (void)tpager::xl9555_write_pin(xl, tpager::XL9555_PIN_SD_POWER_EN, true);
    vTaskDelay(pdMS_TO_TICKS(30));
}

}  // namespace

esp_err_t sd_mount_and_scan_keys(SdDiagStats *stats)
{
    ESP_RETURN_ON_FALSE(stats != nullptr, ESP_ERR_INVALID_ARG, kTag, "stats must not be null");
    *stats = {};

    if (g_mounted) {
        stats->mounted = true;
        ESP_RETURN_ON_ERROR(scan_keys_dir(stats), kTag, "scan keys failed");
        ESP_LOGI(kTag, "SD scan done: entries=%" PRId32 ", pem=%" PRId32, stats->dir_entries, stats->pem_files);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ensure_spi_bus(), kTag, "SPI init failed");
    maybe_power_cycle_sd();

    esp_err_t ret = ESP_FAIL;
    for (int attempt = 1; attempt <= 4; ++attempt) {
        ret = try_mount_card(&g_card);
        if (ret == ESP_OK) {
            break;
        }

        ESP_LOGW(kTag, "sd mount attempt #%d failed: %s", attempt, esp_err_to_name(ret));

        // Recovery contract: launcher/runtime transitions can leave SPI host state
        // inconsistent for SDSPI mount. Reset host state and retry with backoff.
        if (g_card != nullptr) {
            esp_vfs_fat_sdcard_unmount(kMountPoint, g_card);
            g_card = nullptr;
        }
        esp_err_t free_ret = spi_bus_free(kSpiHost);
        if (free_ret != ESP_OK && free_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(kTag, "spi_bus_free during recovery returned %s", esp_err_to_name(free_ret));
        }
        maybe_power_cycle_sd();
        vTaskDelay(pdMS_TO_TICKS(30 * attempt));
        ESP_RETURN_ON_ERROR(ensure_spi_bus(), kTag, "SPI re-init failed");
    }
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "sd mount failed after retries: %s", esp_err_to_name(ret));
        return ret;
    }

    g_mounted = true;
    stats->mounted = true;

    ESP_RETURN_ON_ERROR(scan_keys_dir(stats), kTag, "scan keys failed");

    ESP_LOGI(kTag, "SD scan done: entries=%" PRId32 ", pem=%" PRId32, stats->dir_entries, stats->pem_files);
    return ESP_OK;
}

esp_err_t sd_unmount()
{
    if (!g_mounted) {
        return ESP_OK;
    }
    esp_vfs_fat_sdcard_unmount(kMountPoint, g_card);
    g_card = nullptr;
    g_mounted = false;
    return ESP_OK;
}

bool sd_is_mounted()
{
    return g_mounted;
}

}  // namespace tpager
