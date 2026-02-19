/*
 * T-Pager Diagnostic Firmware
 *
 * Bring-up contract:
 * - Use documented T-Pager wiring as source of truth.
 * - Verify shared I2C devices, XL9555 GPIO control, keyboard power/reset gating,
 *   and rotary encoder behavior before feature integration.
 * - Prefer polling-based input diagnostics first for robust early validation.
 */

#include <array>
#include <cinttypes>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "tpager_xl9555.hpp"

namespace {

constexpr const char *kTag = "tpager_diag";

constexpr i2c_port_t kI2CPort = I2C_NUM_0;
constexpr gpio_num_t kI2CSda = GPIO_NUM_3;
constexpr gpio_num_t kI2CScl = GPIO_NUM_2;
constexpr uint32_t kI2CFreqHz = 400000;

constexpr uint8_t kTCA8418Addr = 0x34;

constexpr gpio_num_t kEncoderA = GPIO_NUM_40;
constexpr gpio_num_t kEncoderB = GPIO_NUM_41;
constexpr gpio_num_t kEncoderCenter = GPIO_NUM_7;

constexpr TickType_t ticks_from_ms(uint32_t ms)
{
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

constexpr TickType_t kI2CTimeoutTicks = ticks_from_ms(20);
tpager::Xl9555 g_xl9555;

esp_err_t i2c_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = kI2CSda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = kI2CScl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = kI2CFreqHz;

    ESP_RETURN_ON_ERROR(i2c_param_config(kI2CPort, &conf), kTag, "i2c_param_config failed");
    esp_err_t install_ret = i2c_driver_install(kI2CPort, conf.mode, 0, 0, 0);
    if (install_ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(kTag, "I2C driver already installed on port %d", static_cast<int>(kI2CPort));
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(install_ret, kTag, "i2c_driver_install failed");
    return ESP_OK;
}

esp_err_t i2c_probe(uint8_t address)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, static_cast<uint8_t>((address << 1) | I2C_MASTER_WRITE), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(kI2CPort, cmd, kI2CTimeoutTicks);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(kI2CPort, addr, &reg, 1, value, 1, kI2CTimeoutTicks);
}

void diag_i2c_scan()
{
    ESP_LOGI(kTag, "diag_i2c_scan: start");
    for (uint8_t addr = 0x03; addr <= 0x77; ++addr) {
        esp_err_t ret = i2c_probe(addr);
        if (ret == ESP_OK) {
            ESP_LOGI(kTag, "diag_i2c_scan: found device @ 0x%02X", addr);
        }
        if ((addr & 0x07) == 0) {
            vTaskDelay(1);
        }
    }
    ESP_LOGI(kTag, "diag_i2c_scan: done");
}

void diag_xl9555_dump()
{
    ESP_LOGI(kTag, "diag_xl9555_dump: start");
    uint8_t regs[8] = {};
    esp_err_t ret = tpager::xl9555_dump_regs(g_xl9555, regs);
    if (ret != ESP_OK) {
        ESP_LOGW(kTag, "diag_xl9555_dump: read failed: %s", esp_err_to_name(ret));
        return;
    }
    for (uint8_t reg = 0; reg < 8; ++reg) {
        ESP_LOGI(kTag, "diag_xl9555_dump: reg[0x%02X] = 0x%02X", reg, regs[reg]);
        vTaskDelay(1);
    }
    ESP_LOGI(kTag, "diag_xl9555_dump: done");
}

bool probe_tca8418(uint8_t *cfg_out)
{
    if (i2c_probe(kTCA8418Addr) != ESP_OK) {
        return false;
    }
    uint8_t cfg = 0;
    esp_err_t reg_ret = i2c_read_reg(kTCA8418Addr, 0x01, &cfg);  // CFG register
    if (reg_ret != ESP_OK) {
        ESP_LOGW(kTag, "TCA8418 probe ACKed but CFG read failed: %s", esp_err_to_name(reg_ret));
        return false;
    }
    if (cfg_out) {
        *cfg_out = cfg;
    }
    return true;
}

bool diag_keyboard_reset(uint8_t kb_power_pin)
{
    ESP_LOGI(kTag, "diag_keyboard_reset: trying power pin XL9555 GPIO%u, reset pin GPIO%u",
             kb_power_pin, tpager::XL9555_PIN_KB_RESET);

    if (tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_KB_RESET, true) != ESP_OK ||
        tpager::xl9555_set_dir(g_xl9555, kb_power_pin, true) != ESP_OK) {
        ESP_LOGE(kTag, "diag_keyboard_reset: failed to configure XL9555 pin directions");
        return false;
    }

    // Deterministic power/reset sequence before probing the controller.
    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, kb_power_pin, false));
    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_KB_RESET, false));
    vTaskDelay(ticks_from_ms(30));
    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, kb_power_pin, true));
    vTaskDelay(ticks_from_ms(30));
    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_KB_RESET, true));
    vTaskDelay(ticks_from_ms(30));

    for (int attempt = 1; attempt <= 5; ++attempt) {
        uint8_t cfg = 0;
        if (probe_tca8418(&cfg)) {
            ESP_LOGI(kTag, "diag_keyboard_reset: TCA8418 alive (CFG=0x%02X) on attempt %d", cfg, attempt);
            return true;
        }
        vTaskDelay(ticks_from_ms(20));
    }

    ESP_LOGW(kTag, "diag_keyboard_reset: TCA8418 not detected with power pin GPIO%u", kb_power_pin);
    return false;
}

int8_t encoder_delta_from_transition(uint8_t prev_ab, uint8_t curr_ab)
{
    static constexpr std::array<int8_t, 16> lut = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0,
    };
    return lut[(prev_ab << 2) | curr_ab];
}

void diag_encoder_ticks(uint32_t sample_ms)
{
    ESP_LOGI(kTag, "diag_encoder_ticks: start (%" PRIu32 " ms)", sample_ms);

    gpio_config_t cfg = {};
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pin_bit_mask = (1ULL << kEncoderA) | (1ULL << kEncoderB) | (1ULL << kEncoderCenter);
    ESP_ERROR_CHECK(gpio_config(&cfg));

    auto read_ab = []() -> uint8_t {
        uint8_t a = static_cast<uint8_t>(gpio_get_level(kEncoderA) ? 1 : 0);
        uint8_t b = static_cast<uint8_t>(gpio_get_level(kEncoderB) ? 1 : 0);
        return static_cast<uint8_t>((a << 1) | b);
    };

    uint8_t prev_ab = read_ab();
    int prev_center = gpio_get_level(kEncoderCenter);
    int32_t net = 0;
    int32_t transitions = 0;

    int64_t start_us = esp_timer_get_time();
    int64_t last_report_us = start_us;
    while ((esp_timer_get_time() - start_us) < static_cast<int64_t>(sample_ms) * 1000) {
        uint8_t curr_ab = read_ab();
        if (curr_ab != prev_ab) {
            int8_t step = encoder_delta_from_transition(prev_ab, curr_ab);
            if (step != 0) {
                net += step;
                ++transitions;
            }
            prev_ab = curr_ab;
        }

        int center = gpio_get_level(kEncoderCenter);
        if (center != prev_center) {
            ESP_LOGI(kTag, "diag_encoder_ticks: center=%s", center ? "released" : "pressed");
            prev_center = center;
        }

        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_report_us) >= 1000000) {
            ESP_LOGI(kTag, "diag_encoder_ticks: net=%" PRId32 ", transitions=%" PRId32, net, transitions);
            last_report_us = now_us;
        }
        vTaskDelay(1);
    }

    ESP_LOGI(kTag, "diag_encoder_ticks: done net=%" PRId32 ", transitions=%" PRId32, net, transitions);
}

void run_diag()
{
    ESP_LOGI(kTag, "===== T-PAGER DIAGNOSTIC BOOT =====");
    ESP_LOGI(kTag, "I2C: SDA=%d SCL=%d @ %" PRIu32 "Hz", kI2CSda, kI2CScl, kI2CFreqHz);
    ESP_LOGI(kTag, "Expected I2C devices: XL9555@0x%02X, TCA8418@0x%02X", g_xl9555.address, kTCA8418Addr);
    ESP_LOGI(kTag, "Encoder: A=%d B=%d Center=%d", kEncoderA, kEncoderB, kEncoderCenter);

    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(tpager::xl9555_init(&g_xl9555, kI2CPort, 0x20, kI2CTimeoutTicks));
    diag_i2c_scan();

    if (tpager::xl9555_probe(g_xl9555) != ESP_OK) {
        ESP_LOGE(kTag, "XL9555 not detected at 0x%02X; keyboard reset/power diagnostics skipped", g_xl9555.address);
    } else {
        diag_xl9555_dump();

        ESP_LOGW(kTag,
                 "LilyGo docs conflict for keyboard power gate (GPIO10 vs GPIO8). Trying GPIO10 first, then GPIO8.");
        bool keyboard_ok = diag_keyboard_reset(tpager::XL9555_PIN_KB_POWER_EN_PRIMARY);
        if (!keyboard_ok) {
            keyboard_ok = diag_keyboard_reset(tpager::XL9555_PIN_KB_POWER_EN_FALLBACK);
        }
        ESP_LOGI(kTag, "diag_keyboard_reset: result=%s", keyboard_ok ? "PASS" : "FAIL");

        bool kb_reset_level = false;
        if (tpager::xl9555_read_pin(g_xl9555, tpager::XL9555_PIN_KB_RESET, &kb_reset_level) == ESP_OK) {
            ESP_LOGI(kTag, "diag_keyboard_reset: reset pin level=%d", kb_reset_level ? 1 : 0);
        }
    }

    // Polling-first encoder diagnostics as agreed.
    diag_encoder_ticks(15000);
    ESP_LOGI(kTag, "===== T-PAGER DIAGNOSTIC COMPLETE =====");
}

}  // namespace

extern "C" void app_main(void)
{
    // Ensure verbose bring-up logs are visible even when project default log level is warning.
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    auto run_diag_task = [](void *) {
        run_diag();
        vTaskDelete(nullptr);
    };
    xTaskCreatePinnedToCore(run_diag_task, "tpager_diag_task", 8192, nullptr, 5, nullptr, 1);

    while (true) {
        // Keep firmware alive for serial monitoring and repeated manual checks.
        vTaskDelay(ticks_from_ms(1000));
    }
}
