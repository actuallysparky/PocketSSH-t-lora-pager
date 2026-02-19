#pragma once

#include <cstdint>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace tpager {

// T-Pager keyboard gating lines per LilyGo docs + live validation fallback.
constexpr uint8_t XL9555_PIN_KB_RESET = 2;
constexpr uint8_t XL9555_PIN_KB_POWER_EN_PRIMARY = 10;
constexpr uint8_t XL9555_PIN_KB_POWER_EN_FALLBACK = 8;
// T-Pager storage/presence controls from LilyGo docs.
constexpr uint8_t XL9555_PIN_SD_DETECT = 12;
constexpr uint8_t XL9555_PIN_SD_POWER_EN = 14;

struct Xl9555 {
    i2c_port_t port = I2C_NUM_0;
    uint8_t address = 0x20;
    TickType_t timeout_ticks = pdMS_TO_TICKS(20);
};

esp_err_t xl9555_init(Xl9555 *dev, i2c_port_t port, uint8_t address, TickType_t timeout_ticks);
esp_err_t xl9555_probe(const Xl9555 &dev);

esp_err_t xl9555_read_reg(const Xl9555 &dev, uint8_t reg, uint8_t *value);
esp_err_t xl9555_write_reg(const Xl9555 &dev, uint8_t reg, uint8_t value);

esp_err_t xl9555_set_dir(const Xl9555 &dev, uint8_t pin, bool output);
esp_err_t xl9555_write_pin(const Xl9555 &dev, uint8_t pin, bool level);
esp_err_t xl9555_read_pin(const Xl9555 &dev, uint8_t pin, bool *level);
esp_err_t xl9555_dump_regs(const Xl9555 &dev, uint8_t out_regs[8]);

}  // namespace tpager
