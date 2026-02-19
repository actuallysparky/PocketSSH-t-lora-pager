#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

namespace tpager {

struct Encoder {
    gpio_num_t pin_a = GPIO_NUM_NC;
    gpio_num_t pin_b = GPIO_NUM_NC;
    gpio_num_t pin_button = GPIO_NUM_NC;
    bool has_button = false;
    uint8_t prev_ab = 0;
    int prev_button_level = 1;
    int8_t phase_acc = 0;
};

struct EncoderEvent {
    bool moved = false;
    int32_t delta = 0;
    int32_t transitions = 0;
    bool button_changed = false;
    bool button_pressed = false;
};

esp_err_t encoder_init(Encoder *enc, gpio_num_t pin_a, gpio_num_t pin_b, gpio_num_t pin_button = GPIO_NUM_NC);
esp_err_t encoder_poll(Encoder *enc, EncoderEvent *event);

}  // namespace tpager
