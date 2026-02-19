#include "tpager_xl9555.hpp"

#include "esp_check.h"

namespace {

constexpr uint8_t kRegInput0 = 0x00;
constexpr uint8_t kRegOutput0 = 0x02;
constexpr uint8_t kRegConfig0 = 0x06;

esp_err_t check_pin(uint8_t pin)
{
    return pin <= 15 ? ESP_OK : ESP_ERR_INVALID_ARG;
}

}  // namespace

namespace tpager {

esp_err_t xl9555_init(Xl9555 *dev, i2c_port_t port, uint8_t address, TickType_t timeout_ticks)
{
    if (dev == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->port = port;
    dev->address = address;
    dev->timeout_ticks = timeout_ticks;
    return ESP_OK;
}

esp_err_t xl9555_probe(const Xl9555 &dev)
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

esp_err_t xl9555_read_reg(const Xl9555 &dev, uint8_t reg, uint8_t *value)
{
    if (value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_write_read_device(dev.port, dev.address, &reg, 1, value, 1, dev.timeout_ticks);
}

esp_err_t xl9555_write_reg(const Xl9555 &dev, uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(dev.port, dev.address, data, sizeof(data), dev.timeout_ticks);
}

esp_err_t xl9555_set_dir(const Xl9555 &dev, uint8_t pin, bool output)
{
    ESP_RETURN_ON_ERROR(check_pin(pin), "tpager_xl9555", "invalid pin");

    const uint8_t reg = static_cast<uint8_t>(kRegConfig0 + (pin / 8));
    const uint8_t bit = static_cast<uint8_t>(1U << (pin % 8));

    uint8_t cfg = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, reg, &cfg), "tpager_xl9555", "config read failed");

    // XL9555 direction bit semantics: 1=input, 0=output.
    cfg = output ? static_cast<uint8_t>(cfg & ~bit) : static_cast<uint8_t>(cfg | bit);
    return xl9555_write_reg(dev, reg, cfg);
}

esp_err_t xl9555_write_pin(const Xl9555 &dev, uint8_t pin, bool level)
{
    ESP_RETURN_ON_ERROR(check_pin(pin), "tpager_xl9555", "invalid pin");

    const uint8_t reg = static_cast<uint8_t>(kRegOutput0 + (pin / 8));
    const uint8_t bit = static_cast<uint8_t>(1U << (pin % 8));

    uint8_t out = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, reg, &out), "tpager_xl9555", "output read failed");

    out = level ? static_cast<uint8_t>(out | bit) : static_cast<uint8_t>(out & ~bit);
    return xl9555_write_reg(dev, reg, out);
}

esp_err_t xl9555_read_pin(const Xl9555 &dev, uint8_t pin, bool *level)
{
    ESP_RETURN_ON_ERROR(check_pin(pin), "tpager_xl9555", "invalid pin");
    if (level == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t reg = static_cast<uint8_t>(kRegInput0 + (pin / 8));
    const uint8_t bit = static_cast<uint8_t>(1U << (pin % 8));

    uint8_t in = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, reg, &in), "tpager_xl9555", "input read failed");
    *level = (in & bit) != 0;
    return ESP_OK;
}

esp_err_t xl9555_dump_regs(const Xl9555 &dev, uint8_t out_regs[8])
{
    if (out_regs == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t reg = 0; reg < 8; ++reg) {
        esp_err_t ret = xl9555_read_reg(dev, reg, &out_regs[reg]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

}  // namespace tpager
