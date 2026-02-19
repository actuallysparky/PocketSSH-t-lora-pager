#include "tpager_tca8418.hpp"

#include <cctype>

#include "esp_check.h"
#include "esp_timer.h"

namespace {

constexpr uint8_t kRegCfg = 0x01;
constexpr uint8_t kRegIntStat = 0x02;
constexpr uint8_t kRegKeyLckEc = 0x03;
constexpr uint8_t kRegKeyEventA = 0x04;
constexpr uint8_t kRegKpGpio1 = 0x1D;
constexpr uint8_t kRegKpGpio2 = 0x1E;
constexpr uint8_t kRegKpGpio3 = 0x1F;

constexpr uint8_t kCfgAi = 1U << 0;
constexpr uint8_t kIntKey = 1U << 0;
constexpr uint8_t kEventCountMask = 0x0F;
constexpr int64_t kSpaceDebounceUs = 15000;
constexpr int64_t kSpaceReleaseFallbackUs = 40000;

// T-LoRa Pager keyboard map from LilyGo reference firmware.
constexpr char kKeymap[4][10] = {
    {'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p'},
    {'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', '\n'},
    {'\0', 'z', 'x', 'c', 'v', 'b', 'n', 'm', '\0', '\0'},
    {' ', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'},
};

constexpr char kSymbolMap[4][10] = {
    {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'},
    {'*', '/', '+', '-', '=', ':', '\'', '"', '@', '\0'},
    {'\0', '_', '$', ';', '?', '!', ',', '.', '\0', '\0'},
    {' ', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'},
};

// LilyGo keyboard special-key constants are based on the zero-based matrix key index
// (raw TCA8418 code minus 1), not the raw FIFO code itself.
constexpr uint8_t kKeyIndexAlt = 0x14;
constexpr uint8_t kKeyIndexCaps = 0x1C;
constexpr uint8_t kKeyIndexBackspace = 0x1D;
constexpr uint8_t kKeyIndexSpace = 0x1E;

esp_err_t tca_read_reg(const tpager::Tca8418 &dev, uint8_t reg, uint8_t *value)
{
    if (value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_master_write_read_device(dev.port, dev.address, &reg, 1, value, 1, dev.timeout_ticks);
}

esp_err_t tca_write_reg(const tpager::Tca8418 &dev, uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(dev.port, dev.address, data, sizeof(data), dev.timeout_ticks);
}

esp_err_t tca_read_event_count(const tpager::Tca8418 &dev, uint8_t *count)
{
    uint8_t ec = 0;
    ESP_RETURN_ON_ERROR(tca_read_reg(dev, kRegKeyLckEc, &ec), "tpager_tca8418", "KEY_LCK_EC read failed");
    *count = ec & kEventCountMask;
    return ESP_OK;
}

char key_from_maps(bool symbol, bool caps, uint8_t row, uint8_t col)
{
    if (row >= 4 || col >= 10) {
        return '\0';
    }
    char ch = symbol ? kSymbolMap[row][col] : kKeymap[row][col];
    if (!symbol && caps && ch != '\0') {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return ch;
}

bool should_emit_space(tpager::Tca8418State *state)
{
    if (state == nullptr) {
        return false;
    }
    const int64_t now_us = esp_timer_get_time();
    if (now_us - state->last_space_emit_us < kSpaceDebounceUs) {
        return false;
    }
    state->last_space_emit_us = now_us;
    return true;
}

bool should_emit_space_release_fallback(tpager::Tca8418State *state)
{
    if (state == nullptr) {
        return false;
    }
    const int64_t now_us = esp_timer_get_time();
    if (now_us - state->last_space_emit_us < kSpaceReleaseFallbackUs) {
        return false;
    }
    state->last_space_emit_us = now_us;
    return true;
}

}  // namespace

namespace tpager {

esp_err_t tca8418_init(Tca8418 *dev, i2c_port_t port, uint8_t address, TickType_t timeout_ticks)
{
    if (dev == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    dev->port = port;
    dev->address = address;
    dev->timeout_ticks = timeout_ticks;
    return ESP_OK;
}

esp_err_t tca8418_probe(const Tca8418 &dev)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, static_cast<uint8_t>((dev.address << 1) | I2C_MASTER_WRITE), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev.port, cmd, dev.timeout_ticks);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t tca8418_configure_matrix(Tca8418 *dev, uint8_t rows, uint8_t cols)
{
    if (dev == nullptr || rows == 0 || rows > 8 || cols == 0 || cols > 10) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t row_mask = 0;
    for (uint8_t r = 0; r < rows; ++r) {
        row_mask |= static_cast<uint8_t>(1U << r);
    }

    uint8_t col_mask_2 = 0;
    uint8_t col_mask_3 = 0;
    for (uint8_t c = 0; c < cols; ++c) {
        const uint8_t pin = static_cast<uint8_t>(9 + c);  // columns map to pins 9..18
        if (pin <= 16) {
            col_mask_2 |= static_cast<uint8_t>(1U << (pin - 9));
        } else {
            col_mask_3 |= static_cast<uint8_t>(1U << (pin - 17));
        }
    }

    // Contract: keypad scan mode is enabled by configuring row/column pins in KP_GPIO registers.
    ESP_RETURN_ON_ERROR(tca_write_reg(*dev, kRegKpGpio1, row_mask), "tpager_tca8418", "KP_GPIO1 write failed");
    ESP_RETURN_ON_ERROR(tca_write_reg(*dev, kRegKpGpio2, col_mask_2), "tpager_tca8418", "KP_GPIO2 write failed");
    ESP_RETURN_ON_ERROR(tca_write_reg(*dev, kRegKpGpio3, col_mask_3), "tpager_tca8418", "KP_GPIO3 write failed");

    ESP_RETURN_ON_ERROR(tca_write_reg(*dev, kRegCfg, kCfgAi), "tpager_tca8418", "CFG write failed");
    ESP_RETURN_ON_ERROR(tca_write_reg(*dev, kRegIntStat, 0xFF), "tpager_tca8418", "INT_STAT clear failed");

    dev->rows = rows;
    dev->cols = cols;
    return tca8418_flush_fifo(*dev);
}

esp_err_t tca8418_flush_fifo(const Tca8418 &dev)
{
    for (int i = 0; i < 16; ++i) {
        uint8_t count = 0;
        ESP_RETURN_ON_ERROR(tca_read_event_count(dev, &count), "tpager_tca8418", "event count read failed");
        if (count == 0) {
            return ESP_OK;
        }
        uint8_t dummy = 0;
        ESP_RETURN_ON_ERROR(tca_read_reg(dev, kRegKeyEventA, &dummy), "tpager_tca8418", "KEY_EVENT read failed");
    }
    return ESP_OK;
}

esp_err_t tca8418_poll_event(const Tca8418 &dev, Tca8418State *state, Tca8418Event *event)
{
    if (state == nullptr || event == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    *event = Tca8418Event{};

    uint8_t count = 0;
    ESP_RETURN_ON_ERROR(tca_read_event_count(dev, &count), "tpager_tca8418", "event count read failed");
    if (count == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t raw = 0;
    ESP_RETURN_ON_ERROR(tca_read_reg(dev, kRegKeyEventA, &raw), "tpager_tca8418", "KEY_EVENT read failed");
    (void)tca_write_reg(dev, kRegIntStat, kIntKey);
    if (raw == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    event->valid = true;
    event->raw = raw;
    event->pressed = (raw & 0x80U) != 0;
    event->code = static_cast<uint8_t>(raw & 0x7FU);

    if (event->code > 96) {
        event->is_gpio = true;
        return ESP_OK;
    }

    if (event->code == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    event->matrix_index = static_cast<uint8_t>(event->code - 1);
    event->row = static_cast<uint8_t>(event->matrix_index / 10);
    event->col = static_cast<uint8_t>(event->matrix_index % 10);

    // Contract: For this fork, ALT is the number/symbol chord modifier.
    if (event->matrix_index == kKeyIndexAlt) {
        if (event->pressed) {
            state->symbol = true;
            state->symbol_chord_used = false;
            event->key = Tca8418Key::Alt;
        } else {
            event->key = Tca8418Key::Alt;
            state->symbol = false;
            state->symbol_chord_used = false;
        }
        return ESP_OK;
    }

    // Space is a dedicated key; debounce it and synthesize as press on release-only cases.
    if (event->matrix_index == kKeyIndexSpace) {
        event->key = Tca8418Key::Space;
        event->ch = ' ';
        if (event->pressed && should_emit_space(state)) {
            event->pressed = true;
        } else if (!event->pressed && should_emit_space_release_fallback(state)) {
            event->pressed = true;
        } else {
            event->pressed = false;
        }
        return ESP_OK;
    }

    // Caps is treated as momentary shift for this target.
    if (event->matrix_index == kKeyIndexCaps) {
        state->caps = event->pressed;
        event->key = Tca8418Key::Caps;
        return ESP_OK;
    }
    if (event->matrix_index == kKeyIndexBackspace) {
        event->key = Tca8418Key::Backspace;
        event->ch = '\b';
        return ESP_OK;
    }

    if (event->row < dev.rows && event->col < dev.cols) {
        const bool symbol_active = state->symbol;
        event->ch = key_from_maps(symbol_active, state->caps, event->row, event->col);
        if (symbol_active && event->pressed) {
            state->symbol_chord_used = true;
        }
        if (event->ch == '\n') {
            event->key = Tca8418Key::Enter;
        } else if (event->ch == ' ') {
            event->key = Tca8418Key::Space;
        } else if (event->ch != '\0') {
            event->key = Tca8418Key::Character;
        }
    }

    return ESP_OK;
}

}  // namespace tpager
