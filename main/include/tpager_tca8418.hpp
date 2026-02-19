#pragma once

#include <cstdint>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace tpager {

struct Tca8418 {
    i2c_port_t port = I2C_NUM_0;
    uint8_t address = 0x34;
    TickType_t timeout_ticks = pdMS_TO_TICKS(20);
    uint8_t rows = 4;
    uint8_t cols = 10;
};

enum class Tca8418Key : uint8_t {
    Unknown = 0,
    Character,
    Enter,
    Backspace,
    Alt,
    Caps,
    Symbol,
    Space,
};

struct Tca8418State {
    bool alt = false;
    bool caps = false;
    bool symbol = false;
};

struct Tca8418Event {
    bool valid = false;
    bool pressed = false;
    bool is_gpio = false;
    uint8_t raw = 0;
    uint8_t code = 0;
    uint8_t row = 0;
    uint8_t col = 0;
    uint8_t matrix_index = 0;
    Tca8418Key key = Tca8418Key::Unknown;
    char ch = '\0';
};

esp_err_t tca8418_init(Tca8418 *dev, i2c_port_t port, uint8_t address, TickType_t timeout_ticks);
esp_err_t tca8418_probe(const Tca8418 &dev);
esp_err_t tca8418_configure_matrix(Tca8418 *dev, uint8_t rows, uint8_t cols);
esp_err_t tca8418_flush_fifo(const Tca8418 &dev);
esp_err_t tca8418_poll_event(const Tca8418 &dev, Tca8418State *state, Tca8418Event *event);

}  // namespace tpager
