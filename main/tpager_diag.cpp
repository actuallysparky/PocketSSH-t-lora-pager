/*
 * T-Pager Diagnostic Firmware
 *
 * Bring-up contract:
 * - Use documented T-Pager wiring as source of truth.
 * - Verify shared I2C devices, XL9555 GPIO control, keyboard power/reset gating,
 *   and rotary encoder behavior before feature integration.
 * - Prefer polling-based input diagnostics first for robust early validation.
 */

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "tpager_display.hpp"
#include "tpager_encoder.hpp"
#include "tpager_sd.hpp"
#include "tpager_tca8418.hpp"
#include "tpager_xl9555.hpp"

namespace {

constexpr const char *kTag = "tpager_diag";

constexpr i2c_port_t kI2CPort = I2C_NUM_0;
constexpr gpio_num_t kI2CSda = GPIO_NUM_3;
constexpr gpio_num_t kI2CScl = GPIO_NUM_2;
constexpr uint32_t kI2CFreqHz = 400000;

constexpr uint8_t kTCA8418Addr = 0x34;
constexpr gpio_num_t kKeyboardIrq = GPIO_NUM_6;

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
tpager::Tca8418 g_tca8418;
tpager::Tca8418State g_tca8418_state;
tpager::DiagDisplay g_display;
std::vector<std::string> g_echo_history;
constexpr size_t kMaxEchoHistory = 24;
TaskHandle_t g_diag_task_handle = nullptr;
volatile uint32_t g_keyboard_irq_count = 0;

void IRAM_ATTR keyboard_irq_isr(void *)
{
    g_keyboard_irq_count = g_keyboard_irq_count + 1;
    if (g_diag_task_handle == nullptr) {
        return;
    }
    BaseType_t high_priority_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(g_diag_task_handle, &high_priority_wakeup);
    if (high_priority_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

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
    if (tpager::tca8418_probe(g_tca8418) != ESP_OK) {
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

const char *key_name(tpager::Tca8418Key key)
{
    switch (key) {
    case tpager::Tca8418Key::Character:
        return "CHAR";
    case tpager::Tca8418Key::Enter:
        return "ENTER";
    case tpager::Tca8418Key::Backspace:
        return "BACKSPACE";
    case tpager::Tca8418Key::Alt:
        return "ALT";
    case tpager::Tca8418Key::Caps:
        return "CAPS";
    case tpager::Tca8418Key::Symbol:
        return "SYMBOL";
    case tpager::Tca8418Key::Space:
        return "SPACE";
    default:
        return "UNKNOWN";
    }
}

bool to_terminal_byte(const tpager::Tca8418Event &ev, uint8_t *out_byte)
{
    if (out_byte == nullptr || !ev.pressed) {
        return false;
    }

    switch (ev.key) {
    case tpager::Tca8418Key::Character:
    case tpager::Tca8418Key::Space:
        if (ev.ch != '\0') {
            *out_byte = static_cast<uint8_t>(ev.ch);
            return true;
        }
        break;
    case tpager::Tca8418Key::Enter:
        *out_byte = '\r';
        return true;
    case tpager::Tca8418Key::Backspace:
        *out_byte = 0x7F;
        return true;
    default:
        break;
    }
    return false;
}

void push_echo_history(std::string line)
{
    if (line.empty()) {
        return;
    }
    if (g_echo_history.size() >= kMaxEchoHistory) {
        g_echo_history.erase(g_echo_history.begin());
    }
    g_echo_history.push_back(std::move(line));
}

void diag_keyboard_events(uint32_t sample_ms)
{
    ESP_LOGI(kTag, "diag_keyboard_events: init matrix=4x10 (polling+IRQ mode, IRQ pin=%d)", kKeyboardIrq);
    tpager::diag_display_set_stage(&g_display, "Stage: keyboard polling");
    ESP_ERROR_CHECK(tpager::tca8418_configure_matrix(&g_tca8418, 4, 10));
    ESP_ERROR_CHECK(tpager::tca8418_flush_fifo(g_tca8418));

    gpio_config_t irq_cfg = {};
    irq_cfg.mode = GPIO_MODE_INPUT;
    irq_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    irq_cfg.pin_bit_mask = (1ULL << kKeyboardIrq);
    ESP_ERROR_CHECK(gpio_config(&irq_cfg));
    gpio_set_intr_type(kKeyboardIrq, GPIO_INTR_NEGEDGE);
    esp_err_t isr_ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isr_ret);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(kKeyboardIrq, keyboard_irq_isr, nullptr));
    g_diag_task_handle = xTaskGetCurrentTaskHandle();
    g_keyboard_irq_count = 0;

    ESP_LOGI(kTag, "diag_keyboard_events: sampling for %" PRIu32 " ms; press keys now", sample_ms);
    int64_t start_us = esp_timer_get_time();
    int64_t last_report_us = start_us;
    int32_t events = 0;
    int32_t presses = 0;
    int32_t releases = 0;
    int32_t irq_wakes = 0;
    std::string echo_line;
    tpager::diag_display_set_keyboard_stats(&g_display, events, presses, releases, gpio_get_level(kKeyboardIrq));

    while ((esp_timer_get_time() - start_us) < static_cast<int64_t>(sample_ms) * 1000) {
        if (ulTaskNotifyTake(pdTRUE, ticks_from_ms(20)) > 0) {
            irq_wakes++;
        }

        while (true) {
            tpager::Tca8418Event ev = {};
            esp_err_t ret = tpager::tca8418_poll_event(g_tca8418, &g_tca8418_state, &ev);
            if (ret == ESP_ERR_NOT_FOUND) {
                break;
            }
            if (ret != ESP_OK) {
                ESP_LOGW(kTag, "diag_keyboard_events: poll error: %s", esp_err_to_name(ret));
                break;
            }
            if (!ev.valid) {
                break;
            }

            ++events;
            if (ev.pressed) {
                ++presses;
            } else {
                ++releases;
            }

            if (ev.is_gpio) {
                ESP_LOGI(kTag, "diag_keyboard_events: GPIO event raw=0x%02X %s", ev.raw,
                         ev.pressed ? "PRESSED" : "RELEASED");
            } else {
                if (ev.ch == '\n') {
                    ESP_LOGI(kTag,
                             "diag_keyboard_events: raw=0x%02X code=%u row=%u col=%u %s key=%s ch=\\n",
                             ev.raw, ev.code, ev.row, ev.col, ev.pressed ? "PRESSED" : "RELEASED",
                             key_name(ev.key));
                } else if (ev.ch == '\b') {
                    ESP_LOGI(kTag,
                             "diag_keyboard_events: raw=0x%02X code=%u row=%u col=%u %s key=%s ch=\\b",
                             ev.raw, ev.code, ev.row, ev.col, ev.pressed ? "PRESSED" : "RELEASED",
                             key_name(ev.key));
                } else if (ev.ch != '\0') {
                    ESP_LOGI(kTag,
                             "diag_keyboard_events: raw=0x%02X code=%u row=%u col=%u %s key=%s ch='%c'",
                             ev.raw, ev.code, ev.row, ev.col, ev.pressed ? "PRESSED" : "RELEASED",
                             key_name(ev.key), ev.ch);
                } else {
                    ESP_LOGI(kTag,
                             "diag_keyboard_events: raw=0x%02X code=%u row=%u col=%u %s key=%s",
                             ev.raw, ev.code, ev.row, ev.col, ev.pressed ? "PRESSED" : "RELEASED",
                             key_name(ev.key));
                }

                uint8_t tx_byte = 0;
                if (to_terminal_byte(ev, &tx_byte)) {
                    ESP_LOGI(kTag, "diag_keyboard_events: tx_byte=0x%02X", tx_byte);
                    if (tx_byte == 0x7F) {
                        if (!echo_line.empty()) {
                            echo_line.pop_back();
                        }
                    } else if (tx_byte == '\r') {
                        ESP_LOGI(kTag, "diag_keyboard_events: echo_submit=\"%s\"", echo_line.c_str());
                        push_echo_history(echo_line);
                        tpager::diag_display_set_last_line(&g_display, echo_line.c_str());
                        echo_line.clear();
                    } else if (tx_byte >= 0x20 && tx_byte <= 0x7E) {
                        echo_line.push_back(static_cast<char>(tx_byte));
                    }
                }
            }
        }

        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_report_us) >= 1000000) {
            int irq_level = gpio_get_level(kKeyboardIrq);
            uint32_t irq_total = g_keyboard_irq_count;
            ESP_LOGI(kTag,
                     "diag_keyboard_events: irq=%d irq_total=%" PRIu32 " wakes=%" PRId32
                     " events=%" PRId32 " (p=%" PRId32 ", r=%" PRId32 ")",
                     irq_level, irq_total, irq_wakes, events, presses, releases);
            tpager::diag_display_set_keyboard_stats(&g_display, events, presses, releases, irq_level);
            last_report_us = now_us;
        }
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_remove(kKeyboardIrq));
    g_diag_task_handle = nullptr;
    ESP_LOGI(kTag, "diag_keyboard_events: done events=%" PRId32 " (p=%" PRId32 ", r=%" PRId32 ")",
             events, presses, releases);
    tpager::diag_display_set_keyboard_stats(&g_display, events, presses, releases, gpio_get_level(kKeyboardIrq));
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

void diag_encoder_ticks(uint32_t sample_ms)
{
    ESP_LOGI(kTag, "diag_encoder_ticks: start (%" PRIu32 " ms)", sample_ms);
    tpager::diag_display_set_stage(&g_display, "Stage: encoder polling");
    tpager::Encoder enc = {};
    ESP_ERROR_CHECK(tpager::encoder_init(&enc, kEncoderA, kEncoderB, kEncoderCenter));

    int32_t net = 0;
    int32_t transitions = 0;
    int32_t history_index = g_echo_history.empty() ? -1 : static_cast<int32_t>(g_echo_history.size() - 1);
    tpager::diag_display_set_encoder_stats(&g_display, net, transitions);

    int64_t start_us = esp_timer_get_time();
    int64_t last_report_us = start_us;
    while ((esp_timer_get_time() - start_us) < static_cast<int64_t>(sample_ms) * 1000) {
        tpager::EncoderEvent ev = {};
        esp_err_t ret = tpager::encoder_poll(&enc, &ev);
        if (ret == ESP_OK) {
            transitions += ev.transitions;
            if (ev.moved) {
                net += ev.delta;
                ESP_LOGI(kTag,
                         "diag_encoder_ticks: delta=%" PRId32 ", net=%" PRId32 ", transitions=%" PRId32,
                         ev.delta, net, transitions);

                // Contract: encoder movement should be visible in the same harness where keyboard submit logs lines.
                if (!g_echo_history.empty()) {
                    history_index = std::clamp(history_index - ev.delta, static_cast<int32_t>(0),
                                               static_cast<int32_t>(g_echo_history.size() - 1));
                    ESP_LOGI(kTag, "diag_encoder_ticks: history[%ld]=\"%s\"",
                             static_cast<long>(history_index), g_echo_history[history_index].c_str());
                    tpager::diag_display_set_last_line(&g_display, g_echo_history[history_index].c_str());
                }
                tpager::diag_display_set_encoder_stats(&g_display, net, transitions);
            }
            if (ev.button_changed) {
                ESP_LOGI(kTag, "diag_encoder_ticks: center=%s", ev.button_pressed ? "pressed" : "released");
            }
        } else if (ret != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(kTag, "diag_encoder_ticks: poll error: %s", esp_err_to_name(ret));
        }

        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_report_us) >= 1000000) {
            ESP_LOGI(kTag, "diag_encoder_ticks: net=%" PRId32 ", transitions=%" PRId32, net, transitions);
            tpager::diag_display_set_encoder_stats(&g_display, net, transitions);
            last_report_us = now_us;
        }
        vTaskDelay(1);
    }

    ESP_LOGI(kTag, "diag_encoder_ticks: done net=%" PRId32 ", transitions=%" PRId32, net, transitions);
    tpager::diag_display_set_encoder_stats(&g_display, net, transitions);
}

void diag_sd_card()
{
    tpager::diag_display_set_stage(&g_display, "Stage: SD mount");

    // Contract: enable SD rail through XL9555 before SDSPI mount attempt.
    if (tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_SD_POWER_EN, true) == ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_SD_POWER_EN, true));
    } else {
        ESP_LOGW(kTag, "diag_sd: failed to set SD power pin direction");
    }

    bool sd_detect_level = true;
    if (tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_SD_DETECT, false) == ESP_OK &&
        tpager::xl9555_read_pin(g_xl9555, tpager::XL9555_PIN_SD_DETECT, &sd_detect_level) == ESP_OK) {
        ESP_LOGI(kTag, "diag_sd: SD detect level=%d (board-dependent polarity)", sd_detect_level ? 1 : 0);
    } else {
        ESP_LOGW(kTag, "diag_sd: unable to read SD detect pin");
    }

    tpager::SdDiagStats stats = {};
    esp_err_t ret = tpager::sd_mount_and_scan_keys(&stats);
    if (ret != ESP_OK) {
        ESP_LOGW(kTag, "diag_sd: mount/scan failed: %s", esp_err_to_name(ret));
        tpager::diag_display_set_last_line(&g_display, "SD mount failed");
        return;
    }

    ESP_LOGI(kTag, "diag_sd: mounted=%d created_dir=%d entries=%" PRId32 " pem=%" PRId32, stats.mounted ? 1 : 0,
             stats.keys_dir_created ? 1 : 0, stats.dir_entries, stats.pem_files);
    char line[96];
    std::snprintf(line, sizeof(line), "SD pem=%" PRId32 " entries=%" PRId32, stats.pem_files, stats.dir_entries);
    tpager::diag_display_set_last_line(&g_display, line);

    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::sd_unmount());
}

void run_diag()
{
    ESP_LOGI(kTag, "===== T-PAGER DIAGNOSTIC BOOT =====");
    ESP_LOGI(kTag, "I2C: SDA=%d SCL=%d @ %" PRIu32 "Hz", kI2CSda, kI2CScl, kI2CFreqHz);
    ESP_LOGI(kTag, "Expected I2C devices: XL9555@0x%02X, TCA8418@0x%02X", g_xl9555.address, kTCA8418Addr);
    ESP_LOGI(kTag, "Encoder: A=%d B=%d Center=%d", kEncoderA, kEncoderB, kEncoderCenter);

    esp_err_t display_ret = tpager::diag_display_init(&g_display);
    if (display_ret == ESP_OK) {
        tpager::diag_display_set_stage(&g_display, "Stage: display online");
        tpager::diag_display_set_last_line(&g_display, "<none>");
    } else {
        ESP_LOGW(kTag, "Display bring-up failed: %s (continuing with serial diagnostics)",
                 esp_err_to_name(display_ret));
    }

    ESP_ERROR_CHECK(i2c_init());
    tpager::diag_display_set_stage(&g_display, "Stage: I2C scan");
    ESP_ERROR_CHECK(tpager::xl9555_init(&g_xl9555, kI2CPort, 0x20, kI2CTimeoutTicks));
    ESP_ERROR_CHECK(tpager::tca8418_init(&g_tca8418, kI2CPort, kTCA8418Addr, kI2CTimeoutTicks));
    diag_i2c_scan();

    if (tpager::xl9555_probe(g_xl9555) != ESP_OK) {
        ESP_LOGE(kTag, "XL9555 not detected at 0x%02X; keyboard reset/power diagnostics skipped", g_xl9555.address);
    } else {
        diag_xl9555_dump();
        diag_sd_card();

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

        if (keyboard_ok) {
            // Polling-first per bring-up strategy. IRQ observation is logged as telemetry only.
            diag_keyboard_events(10000);
        } else {
            tpager::diag_display_set_stage(&g_display, "Stage: keyboard init failed");
        }
    }

    // Polling-first encoder diagnostics as agreed.
    diag_encoder_ticks(15000);
    tpager::diag_display_set_stage(&g_display, "Stage: diag complete");
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
