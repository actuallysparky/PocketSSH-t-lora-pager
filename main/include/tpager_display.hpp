#pragma once

#include <cinttypes>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

namespace tpager {

struct DiagDisplay {
    bool initialized = false;
    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_handle_t panel_handle = nullptr;
    lv_display_t *disp = nullptr;
    lv_obj_t *title_label = nullptr;
    lv_obj_t *stage_label = nullptr;
    lv_obj_t *kbd_label = nullptr;
    lv_obj_t *enc_label = nullptr;
    lv_obj_t *line_label = nullptr;
};

esp_err_t diag_display_init(DiagDisplay *display);
void diag_display_set_stage(DiagDisplay *display, const char *stage);
void diag_display_set_keyboard_stats(DiagDisplay *display, int32_t events, int32_t presses, int32_t releases,
                                     int irq_level);
void diag_display_set_encoder_stats(DiagDisplay *display, int32_t net, int32_t transitions);
void diag_display_set_last_line(DiagDisplay *display, const char *line);

}  // namespace tpager
