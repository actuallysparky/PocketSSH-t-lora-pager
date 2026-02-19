#include "tpager_display.hpp"

#include <cstdio>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_st7796.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace tpager {
namespace {

constexpr const char *kTag = "tpager_display";

constexpr spi_host_device_t kDisplaySpiHost = SPI2_HOST;
constexpr gpio_num_t kDisplayMosi = GPIO_NUM_34;
constexpr gpio_num_t kDisplayMiso = GPIO_NUM_33;
constexpr gpio_num_t kDisplaySclk = GPIO_NUM_35;
constexpr gpio_num_t kDisplayCs = GPIO_NUM_38;
constexpr gpio_num_t kDisplayDc = GPIO_NUM_37;
constexpr gpio_num_t kDisplayReset = GPIO_NUM_NC;
constexpr gpio_num_t kDisplayBacklight = GPIO_NUM_42;

constexpr uint32_t kDisplayPclkHz = 40 * 1000 * 1000;
constexpr uint16_t kDisplayHRes = 480;
constexpr uint16_t kDisplayVRes = 222;
constexpr uint16_t kDisplayGapX = 0;
constexpr uint16_t kDisplayGapY = 49;
constexpr uint16_t kBufferLines = 40;

void set_label_text(lv_obj_t *label, const char *text)
{
    if (label == nullptr) {
        return;
    }
    if (!lvgl_port_lock(25)) {
        return;
    }
    lv_label_set_text(label, text ? text : "");
    lvgl_port_unlock();
}

esp_err_t init_backlight()
{
    gpio_config_t cfg = {};
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pin_bit_mask = (1ULL << kDisplayBacklight);
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), kTag, "backlight gpio config failed");

    // Simple always-on policy for bring-up: keep panel lit while diagnostics run.
    gpio_set_level(kDisplayBacklight, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(kDisplayBacklight, 1);
    return ESP_OK;
}

esp_err_t init_spi_bus()
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = kDisplayMosi;
    buscfg.miso_io_num = kDisplayMiso;
    buscfg.sclk_io_num = kDisplaySclk;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = kDisplayHRes * kBufferLines * sizeof(uint16_t);

    esp_err_t ret = spi_bus_initialize(kDisplaySpiHost, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(kTag, "SPI bus already initialized on host %d", static_cast<int>(kDisplaySpiHost));
        return ESP_OK;
    }
    return ret;
}

esp_err_t init_panel(DiagDisplay *display)
{
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = kDisplayCs,
        .dc_gpio_num = kDisplayDc,
        .spi_mode = 0,
        .pclk_hz = kDisplayPclkHz,
        .trans_queue_depth = 10,
        .on_color_trans_done = nullptr,
        .user_ctx = nullptr,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .flags = {},
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)kDisplaySpiHost, &io_cfg,
                                                  &display->io_handle),
                        kTag, "new panel io failed");

    esp_lcd_panel_dev_config_t panel_cfg = {};
    panel_cfg.reset_gpio_num = kDisplayReset;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    panel_cfg.color_space = ESP_LCD_COLOR_SPACE_RGB;
#elif ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    panel_cfg.rgb_endian = LCD_RGB_ENDIAN_RGB;
#else
    panel_cfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
#endif
    panel_cfg.bits_per_pixel = 16;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7796(display->io_handle, &panel_cfg, &display->panel_handle), kTag,
                        "new ST7796 panel failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(display->panel_handle), kTag, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(display->panel_handle), kTag, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(display->panel_handle, true), kTag,
                        "panel invert failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(display->panel_handle, true), kTag, "panel swap_xy failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(display->panel_handle, true, true), kTag, "panel mirror failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(display->panel_handle, kDisplayGapX, kDisplayGapY), kTag,
                        "panel set_gap failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(display->panel_handle, true), kTag, "panel on failed");
    return ESP_OK;
}

esp_err_t init_lvgl(DiagDisplay *display)
{
    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_cfg.task_stack = 8192;
    lvgl_cfg.task_max_sleep_ms = 50;
    esp_err_t ret = lvgl_port_init(&lvgl_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = display->io_handle;
    disp_cfg.panel_handle = display->panel_handle;
    disp_cfg.buffer_size = kDisplayHRes * kBufferLines;
    disp_cfg.double_buffer = true;
    disp_cfg.hres = kDisplayHRes;
    disp_cfg.vres = kDisplayVRes;
    disp_cfg.monochrome = false;
    disp_cfg.rotation.swap_xy = true;
    disp_cfg.rotation.mirror_x = true;
    disp_cfg.rotation.mirror_y = true;
#if LVGL_VERSION_MAJOR >= 9
    disp_cfg.color_format = LV_COLOR_FORMAT_RGB565;
    disp_cfg.flags.swap_bytes = true;
#endif
    disp_cfg.flags.buff_dma = true;

    display->disp = lvgl_port_add_disp(&disp_cfg);
    if (display->disp == nullptr) {
        return ESP_FAIL;
    }

    if (!lvgl_port_lock(0)) {
        return ESP_ERR_TIMEOUT;
    }
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_color(scr, lv_color_hex(0xF2F7F5), 0);

    lv_obj_t *frame = lv_obj_create(scr);
    lv_obj_set_size(frame, kDisplayHRes, kDisplayVRes);
    lv_obj_set_pos(frame, 0, 0);
    lv_obj_set_style_bg_opa(frame, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(frame, 2, 0);
    lv_obj_set_style_border_color(frame, lv_color_hex(0x00AA66), 0);
    lv_obj_set_style_radius(frame, 0, 0);
    lv_obj_set_style_pad_all(frame, 0, 0);
    lv_obj_move_background(frame);

    display->title_label = lv_label_create(scr);
    lv_obj_align(display->title_label, LV_ALIGN_TOP_LEFT, 16, 12);
    lv_label_set_text(display->title_label, "PocketSSH T-Pager diag");

    display->stage_label = lv_label_create(scr);
    lv_obj_align(display->stage_label, LV_ALIGN_TOP_LEFT, 16, 38);
    lv_label_set_text(display->stage_label, "Stage: boot");

    display->kbd_label = lv_label_create(scr);
    lv_obj_align(display->kbd_label, LV_ALIGN_TOP_LEFT, 16, 64);
    lv_label_set_text(display->kbd_label, "KBD ev=0 p=0 r=0 irq=-");

    display->enc_label = lv_label_create(scr);
    lv_obj_align(display->enc_label, LV_ALIGN_TOP_LEFT, 16, 90);
    lv_label_set_text(display->enc_label, "ENC net=0 trans=0");

    display->line_label = lv_label_create(scr);
    lv_obj_set_width(display->line_label, kDisplayHRes - 24);
    lv_obj_align(display->line_label, LV_ALIGN_BOTTOM_LEFT, 16, -8);
    lv_label_set_text(display->line_label, "Last line: <none>");

    lv_obj_t *corner_tl = lv_label_create(scr);
    lv_obj_align(corner_tl, LV_ALIGN_TOP_LEFT, 2, 2);
    lv_label_set_text(corner_tl, "+");
    lv_obj_t *corner_tr = lv_label_create(scr);
    lv_obj_align(corner_tr, LV_ALIGN_TOP_RIGHT, -2, 2);
    lv_label_set_text(corner_tr, "+");
    lv_obj_t *corner_bl = lv_label_create(scr);
    lv_obj_align(corner_bl, LV_ALIGN_BOTTOM_LEFT, 2, -2);
    lv_label_set_text(corner_bl, "+");
    lv_obj_t *corner_br = lv_label_create(scr);
    lv_obj_align(corner_br, LV_ALIGN_BOTTOM_RIGHT, -2, -2);
    lv_label_set_text(corner_br, "+");

    lvgl_port_unlock();
    return ESP_OK;
}

}  // namespace

esp_err_t diag_display_init(DiagDisplay *display)
{
    ESP_RETURN_ON_FALSE(display != nullptr, ESP_ERR_INVALID_ARG, kTag, "display must not be null");

    ESP_RETURN_ON_ERROR(init_backlight(), kTag, "backlight init failed");
    ESP_RETURN_ON_ERROR(init_spi_bus(), kTag, "spi init failed");
    ESP_RETURN_ON_ERROR(init_panel(display), kTag, "panel init failed");
    ESP_RETURN_ON_ERROR(init_lvgl(display), kTag, "lvgl init failed");

    display->initialized = true;
    diag_display_set_stage(display, "Stage: display online");
    return ESP_OK;
}

void diag_display_set_stage(DiagDisplay *display, const char *stage)
{
    if (display == nullptr || !display->initialized) {
        return;
    }
    set_label_text(display->stage_label, stage);
}

void diag_display_set_keyboard_stats(DiagDisplay *display, int32_t events, int32_t presses, int32_t releases,
                                     int irq_level)
{
    if (display == nullptr || !display->initialized) {
        return;
    }
    char line[96];
    std::snprintf(line, sizeof(line), "KBD ev=%" PRId32 " p=%" PRId32 " r=%" PRId32 " irq=%d", events, presses,
                  releases, irq_level);
    set_label_text(display->kbd_label, line);
}

void diag_display_set_encoder_stats(DiagDisplay *display, int32_t net, int32_t transitions)
{
    if (display == nullptr || !display->initialized) {
        return;
    }
    char line[64];
    std::snprintf(line, sizeof(line), "ENC net=%" PRId32 " trans=%" PRId32, net, transitions);
    set_label_text(display->enc_label, line);
}

void diag_display_set_last_line(DiagDisplay *display, const char *line)
{
    if (display == nullptr || !display->initialized) {
        return;
    }
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Last line: %s", line ? line : "<none>");
    set_label_text(display->line_label, buf);
}

}  // namespace tpager
