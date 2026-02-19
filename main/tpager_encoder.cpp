#include "tpager_encoder.hpp"

#include <array>

#include "driver/gpio.h"

namespace {

int8_t transition_step(uint8_t prev_ab, uint8_t curr_ab)
{
    static constexpr std::array<int8_t, 16> kLut = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0,
    };
    return kLut[(prev_ab << 2) | curr_ab];
}

uint8_t read_ab(const tpager::Encoder &enc)
{
    const uint8_t a = static_cast<uint8_t>(gpio_get_level(enc.pin_a) ? 1 : 0);
    const uint8_t b = static_cast<uint8_t>(gpio_get_level(enc.pin_b) ? 1 : 0);
    return static_cast<uint8_t>((a << 1) | b);
}

}  // namespace

namespace tpager {

esp_err_t encoder_init(Encoder *enc, gpio_num_t pin_a, gpio_num_t pin_b, gpio_num_t pin_button)
{
    if (enc == nullptr || pin_a == GPIO_NUM_NC || pin_b == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    enc->pin_a = pin_a;
    enc->pin_b = pin_b;
    enc->pin_button = pin_button;
    enc->has_button = pin_button != GPIO_NUM_NC;
    enc->phase_acc = 0;

    gpio_config_t cfg = {};
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pin_bit_mask = (1ULL << pin_a) | (1ULL << pin_b);
    if (enc->has_button) {
        cfg.pin_bit_mask |= (1ULL << pin_button);
    }
    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    enc->prev_ab = read_ab(*enc);
    enc->prev_button_level = enc->has_button ? gpio_get_level(pin_button) : 1;
    return ESP_OK;
}

esp_err_t encoder_poll(Encoder *enc, EncoderEvent *event)
{
    if (enc == nullptr || event == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    *event = EncoderEvent{};

    const uint8_t curr_ab = read_ab(*enc);
    if (curr_ab != enc->prev_ab) {
        const int8_t step = transition_step(enc->prev_ab, curr_ab);
        if (step != 0) {
            ++event->transitions;
            // Contract: expose one logical tick per full quadrature cycle for stable UI navigation.
            enc->phase_acc = static_cast<int8_t>(enc->phase_acc + step);
            while (enc->phase_acc >= 4) {
                ++event->delta;
                enc->phase_acc = static_cast<int8_t>(enc->phase_acc - 4);
            }
            while (enc->phase_acc <= -4) {
                --event->delta;
                enc->phase_acc = static_cast<int8_t>(enc->phase_acc + 4);
            }
        }
        enc->prev_ab = curr_ab;
    }

    if (enc->has_button) {
        const int button_level = gpio_get_level(enc->pin_button);
        if (button_level != enc->prev_button_level) {
            event->button_changed = true;
            event->button_pressed = (button_level == 0);
            enc->prev_button_level = button_level;
        }
    }

    event->moved = (event->delta != 0);
    if (event->moved || event->button_changed || event->transitions != 0) {
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

}  // namespace tpager
