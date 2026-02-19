/*
 * T-Pager Base Runtime
 *
 * Contract:
 * - Dedicated TPAGER_TARGET entrypoint (separate from T-Deck deck_base.cpp).
 * - Reuse proven bring-up modules from TPAGER_DIAG for display/input/SD.
 * - Forward hardware keyboard/encoder events into the existing SSHTerminal flow.
 */

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <strings.h>
#include <vector>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "ssh_terminal.hpp"
#include "tpager_display.hpp"
#include "tpager_encoder.hpp"
#include "tpager_sd.hpp"
#include "tpager_tca8418.hpp"
#include "tpager_xl9555.hpp"

namespace {

constexpr const char *kTag = "tpager_base";

constexpr i2c_port_t kI2CPort = I2C_NUM_0;
constexpr gpio_num_t kI2CSda = GPIO_NUM_3;
constexpr gpio_num_t kI2CScl = GPIO_NUM_2;
constexpr uint32_t kI2CFreqHz = 400000;

constexpr uint8_t kTCA8418Addr = 0x34;
constexpr gpio_num_t kKeyboardIrq = GPIO_NUM_6;

constexpr gpio_num_t kEncoderA = GPIO_NUM_40;
constexpr gpio_num_t kEncoderB = GPIO_NUM_41;
constexpr gpio_num_t kEncoderCenter = GPIO_NUM_7;
constexpr gpio_num_t kBootButton = GPIO_NUM_0;
constexpr gpio_num_t kDisplayBacklight = GPIO_NUM_42;

constexpr const char *kKeysDir = "/sdcard/ssh_keys";
constexpr size_t kMaxKeySize = 16 * 1024;

constexpr TickType_t ticks_from_ms(uint32_t ms)
{
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

constexpr TickType_t kI2CTimeoutTicks = ticks_from_ms(20);

tpager::Xl9555 g_xl9555;
tpager::Tca8418 g_tca8418;
tpager::Tca8418State g_tca8418_state;
tpager::Encoder g_encoder;
tpager::DiagDisplay g_display;

SSHTerminal *g_terminal = nullptr;

TaskHandle_t g_runtime_task_handle = nullptr;
volatile uint32_t g_keyboard_irq_count = 0;

int32_t g_keyboard_events = 0;
int32_t g_keyboard_presses = 0;
int32_t g_keyboard_releases = 0;
int32_t g_encoder_net = 0;
int32_t g_encoder_transitions = 0;
bool g_shutdown_requested = false;
bool g_alt_held = false;
bool g_caps_held = false;

void IRAM_ATTR keyboard_irq_isr(void *)
{
    g_keyboard_irq_count = g_keyboard_irq_count + 1;
    if (g_runtime_task_handle == nullptr) {
        return;
    }
    BaseType_t high_priority_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(g_runtime_task_handle, &high_priority_wakeup);
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
    esp_err_t ret = i2c_driver_install(kI2CPort, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(kTag, "I2C already initialized");
        return ESP_OK;
    }
    return ret;
}

bool probe_tca8418()
{
    return tpager::tca8418_probe(g_tca8418) == ESP_OK;
}

bool keyboard_power_reset(uint8_t kb_power_pin)
{
    if (tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_KB_RESET, true) != ESP_OK ||
        tpager::xl9555_set_dir(g_xl9555, kb_power_pin, true) != ESP_OK) {
        return false;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, kb_power_pin, false));
    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_KB_RESET, false));
    vTaskDelay(ticks_from_ms(30));
    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, kb_power_pin, true));
    vTaskDelay(ticks_from_ms(30));
    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_KB_RESET, true));
    vTaskDelay(ticks_from_ms(30));

    for (int i = 0; i < 5; ++i) {
        if (probe_tca8418()) {
            return true;
        }
        vTaskDelay(ticks_from_ms(20));
    }
    return false;
}

void append_terminal_text(const char *text)
{
    if (g_terminal == nullptr || text == nullptr) {
        return;
    }
    if (!lvgl_port_lock(25)) {
        return;
    }
    g_terminal->append_text(text);
    lvgl_port_unlock();
}

void handle_terminal_key(char key)
{
    if (g_terminal == nullptr) {
        return;
    }
    if (!lvgl_port_lock(25)) {
        return;
    }
    g_terminal->handle_key_input(key);
    lvgl_port_unlock();
}

bool to_terminal_char(const tpager::Tca8418Event &ev, char *out_key)
{
    if (out_key == nullptr || !ev.pressed) {
        return false;
    }

    switch (ev.key) {
    case tpager::Tca8418Key::Character:
    case tpager::Tca8418Key::Space:
        if (ev.ch != '\0') {
            *out_key = ev.ch;
            return true;
        }
        break;
    case tpager::Tca8418Key::Enter:
        *out_key = '\n';
        return true;
    case tpager::Tca8418Key::Backspace:
        *out_key = '\b';
        return true;
    default:
        break;
    }
    return false;
}

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

void load_ssh_keys_from_sd()
{
    if (g_terminal == nullptr) {
        return;
    }

    tpager::SdDiagStats stats = {};
    const esp_err_t mount_ret = tpager::sd_mount_and_scan_keys(&stats);
    if (mount_ret != ESP_OK) {
        ESP_LOGW(kTag, "SD mount/scan failed: %s", esp_err_to_name(mount_ret));
        append_terminal_text("SD key scan failed\n");
        return;
    }

    DIR *dir = opendir(kKeysDir);
    if (dir == nullptr) {
        append_terminal_text("No /sdcard/ssh_keys directory\n");
        ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::sd_unmount());
        return;
    }

    int32_t keys_loaded = 0;
    struct dirent *entry = nullptr;
    while ((entry = readdir(dir)) != nullptr) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (!has_pem_extension(entry->d_name)) {
            continue;
        }

        char filepath[256];
        std::snprintf(filepath, sizeof(filepath), "%s/%s", kKeysDir, entry->d_name);

        FILE *f = std::fopen(filepath, "rb");
        if (f == nullptr) {
            ESP_LOGW(kTag, "Failed to open key %s", filepath);
            continue;
        }

        std::fseek(f, 0, SEEK_END);
        const long file_size = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        if (file_size <= 0 || file_size > static_cast<long>(kMaxKeySize)) {
            ESP_LOGW(kTag, "Skipping key %s (size=%ld)", filepath, file_size);
            std::fclose(f);
            continue;
        }

        std::vector<char> key_data(static_cast<size_t>(file_size) + 1, '\0');
        const size_t bytes_read = std::fread(key_data.data(), 1, static_cast<size_t>(file_size), f);
        std::fclose(f);
        if (bytes_read != static_cast<size_t>(file_size)) {
            ESP_LOGW(kTag, "Short read for key %s", filepath);
            continue;
        }

        g_terminal->load_key_from_memory(entry->d_name, key_data.data(), bytes_read);
        keys_loaded++;
    }

    closedir(dir);
    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::sd_unmount());

    char summary[80];
    std::snprintf(summary, sizeof(summary), "Loaded %" PRId32 " key(s) from SD\n", keys_loaded);
    append_terminal_text(summary);
}

void poll_keyboard()
{
    while (true) {
        tpager::Tca8418Event ev = {};
        const esp_err_t ret = tpager::tca8418_poll_event(g_tca8418, &g_tca8418_state, &ev);
        if (ret == ESP_ERR_NOT_FOUND) {
            break;
        }
        if (ret != ESP_OK) {
            ESP_LOGW(kTag, "keyboard poll failed: %s", esp_err_to_name(ret));
            break;
        }
        if (!ev.valid) {
            break;
        }

        if (ev.key == tpager::Tca8418Key::Alt) {
            g_alt_held = ev.pressed;
        } else if (ev.key == tpager::Tca8418Key::Caps) {
            g_caps_held = ev.pressed;
        }

        g_keyboard_events++;
        if (ev.pressed) {
            g_keyboard_presses++;
        } else {
            g_keyboard_releases++;
        }

        char key = '\0';
        if (to_terminal_char(ev, &key)) {
            if (ev.erase_previous_space) {
                handle_terminal_key('\b');
            }
            handle_terminal_key(key);
        }
    }

    tpager::diag_display_set_keyboard_stats(&g_display, g_keyboard_events, g_keyboard_presses, g_keyboard_releases,
                                            gpio_get_level(kKeyboardIrq));
}

void poll_encoder()
{
    tpager::EncoderEvent ev = {};
    if (tpager::encoder_poll(&g_encoder, &ev) != ESP_OK) {
        return;
    }

    g_encoder_transitions += ev.transitions;
    if (ev.moved) {
        g_encoder_net += ev.delta;
        if (g_terminal != nullptr && lvgl_port_lock(25)) {
            int32_t steps = ev.delta;

            // Encoder interaction contract:
            // - default      : command history
            // - ALT + encoder: cursor left/right on input line
            // - CAPS + encoder: terminal output scroll up/down
            // CAPS mode has priority if both modifiers are held.
            const bool scroll_mode = g_caps_held;
            const bool cursor_mode = !scroll_mode && g_alt_held;

            while (steps > 0) {
                if (scroll_mode) {
                    g_terminal->scroll_terminal_output(-1);
                } else if (cursor_mode) {
                    g_terminal->move_cursor_right();
                } else {
                    g_terminal->navigate_history(1);
                }
                steps--;
            }
            while (steps < 0) {
                if (scroll_mode) {
                    g_terminal->scroll_terminal_output(1);
                } else if (cursor_mode) {
                    g_terminal->move_cursor_left();
                } else {
                    g_terminal->navigate_history(-1);
                }
                steps++;
            }
            lvgl_port_unlock();
        }
    }
    if (ev.button_changed && ev.button_pressed) {
        handle_terminal_key('\n');
    }

    tpager::diag_display_set_encoder_stats(&g_display, g_encoder_net, g_encoder_transitions);
}

void runtime_task(void *)
{
    g_runtime_task_handle = xTaskGetCurrentTaskHandle();

    while (true) {
        // Keep a short poll timeout so brief key taps (especially Space fallback)
        // are handled with low latency even if IRQ edges are imperfect.
        (void)ulTaskNotifyTake(pdTRUE, ticks_from_ms(10));
        poll_keyboard();
        poll_encoder();
    }
}

void shutdown_task(void *)
{
    append_terminal_text("Powering down...\n");
    ESP_LOGW(kTag, "Shutdown requested: entering deep sleep");

    // Stop IRQ traffic while we wind down.
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_intr_disable(kKeyboardIrq));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_remove(kKeyboardIrq));

    // Best-effort comms quiesce.
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_disconnect());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_stop());

    // Turn display backlight off.
    gpio_reset_pin(kDisplayBacklight);
    gpio_set_direction(kDisplayBacklight, GPIO_MODE_OUTPUT);
    gpio_set_level(kDisplayBacklight, 0);

    // Power-gate peripherals controlled via expander.
    if (tpager::xl9555_probe(g_xl9555) == ESP_OK) {
        (void)tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_SD_POWER_EN, true);
        (void)tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_KB_RESET, true);
        (void)tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_KB_POWER_EN_PRIMARY, true);
        (void)tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_KB_POWER_EN_FALLBACK, true);
        ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_SD_POWER_EN, false));
        ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_KB_RESET, false));
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_KB_POWER_EN_PRIMARY, false));
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_KB_POWER_EN_FALLBACK, false));
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::sd_unmount());

    // Wake sources: BOOT key or encoder center press.
    gpio_config_t wake_cfg = {};
    wake_cfg.mode = GPIO_MODE_INPUT;
    wake_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    wake_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    wake_cfg.pin_bit_mask = (1ULL << kBootButton) | (1ULL << kEncoderCenter);
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&wake_cfg));

    const uint64_t wake_mask = (1ULL << kBootButton) | (1ULL << kEncoderCenter);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_sleep_enable_ext1_wakeup_io(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW));
#else
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_sleep_enable_ext1_wakeup(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW));
#endif

    vTaskDelay(ticks_from_ms(150));
    esp_deep_sleep_start();
    vTaskDelete(nullptr);
}

}  // namespace

extern "C" void tpager_request_shutdown(void)
{
    if (g_shutdown_requested) {
        return;
    }
    g_shutdown_requested = true;
    xTaskCreatePinnedToCore(shutdown_task, "tpager_shutdown_task", 6144, nullptr, 6, nullptr, 1);
}

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(kTag, "===== TPAGER TARGET BOOT =====");

    ret = tpager::diag_display_init(&g_display);
    if (ret == ESP_OK) {
        tpager::diag_display_set_stage(&g_display, "Stage: init I2C");
        tpager::diag_display_set_last_line(&g_display, "Runtime booting");
    }

    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(tpager::xl9555_init(&g_xl9555, kI2CPort, 0x20, kI2CTimeoutTicks));
    ESP_ERROR_CHECK(tpager::tca8418_init(&g_tca8418, kI2CPort, kTCA8418Addr, kI2CTimeoutTicks));

    if (tpager::xl9555_probe(g_xl9555) == ESP_OK) {
        if (tpager::xl9555_set_dir(g_xl9555, tpager::XL9555_PIN_SD_POWER_EN, true) == ESP_OK) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(tpager::xl9555_write_pin(g_xl9555, tpager::XL9555_PIN_SD_POWER_EN, true));
        }
    } else {
        ESP_LOGW(kTag, "XL9555 not reachable, skipping SD power control");
    }

    tpager::diag_display_set_stage(&g_display, "Stage: keyboard init");
    bool keyboard_ok = keyboard_power_reset(tpager::XL9555_PIN_KB_POWER_EN_PRIMARY);
    if (!keyboard_ok) {
        keyboard_ok = keyboard_power_reset(tpager::XL9555_PIN_KB_POWER_EN_FALLBACK);
    }
    ESP_ERROR_CHECK(tpager::tca8418_configure_matrix(&g_tca8418, 4, 10));
    ESP_ERROR_CHECK(tpager::tca8418_flush_fifo(g_tca8418));

    gpio_config_t irq_cfg = {};
    irq_cfg.mode = GPIO_MODE_INPUT;
    irq_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    irq_cfg.pin_bit_mask = (1ULL << kKeyboardIrq);
    ESP_ERROR_CHECK(gpio_config(&irq_cfg));
    ESP_ERROR_CHECK(gpio_set_intr_type(kKeyboardIrq, GPIO_INTR_NEGEDGE));
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(kKeyboardIrq, keyboard_irq_isr, nullptr));

    tpager::diag_display_set_stage(&g_display, keyboard_ok ? "Stage: keyboard ready" : "Stage: keyboard degraded");
    ESP_LOGI(kTag, "keyboard init: %s", keyboard_ok ? "PASS" : "DEGRADED");

    tpager::diag_display_set_stage(&g_display, "Stage: encoder init");
    ESP_ERROR_CHECK(tpager::encoder_init(&g_encoder, kEncoderA, kEncoderB, kEncoderCenter));
    tpager::diag_display_set_encoder_stats(&g_display, g_encoder_net, g_encoder_transitions);
    tpager::diag_display_set_keyboard_stats(&g_display, g_keyboard_events, g_keyboard_presses, g_keyboard_releases,
                                            gpio_get_level(kKeyboardIrq));

    tpager::diag_display_set_stage(&g_display, "Stage: terminal init");
    g_terminal = new SSHTerminal();
    if (g_terminal != nullptr && lvgl_port_lock(50)) {
        lv_obj_t *screen = g_terminal->create_terminal_screen();
        lv_scr_load(screen);
#ifdef POCKETSSH_VERSION
        g_terminal->append_text("PocketSSH v" POCKETSSH_VERSION "\n");
#else
        g_terminal->append_text("PocketSSH T-Pager\n");
#endif
        g_terminal->append_text("Keyboard + encoder active\n");
        lvgl_port_unlock();
    } else {
        ESP_LOGE(kTag, "Failed to initialize terminal UI");
    }

    load_ssh_keys_from_sd();

    xTaskCreatePinnedToCore(runtime_task, "tpager_runtime_task", 8192, nullptr, 5, nullptr, 1);

    while (true) {
        vTaskDelay(ticks_from_ms(1000));
    }
}
