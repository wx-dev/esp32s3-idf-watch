/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#include "esp_lcd_st7789v.h"

static const char *TAG = "st7789v";

static esp_err_t panel_st7789v_del(esp_lcd_panel_t *panel);

static esp_err_t panel_st7789v_reset(esp_lcd_panel_t *panel);

static esp_err_t panel_st7789v_init(esp_lcd_panel_t *panel);

static esp_err_t panel_st7789v_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                           const void *color_data);

static esp_err_t panel_st7789v_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);

static esp_err_t panel_st7789v_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);

static esp_err_t panel_st7789v_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);

static esp_err_t panel_st7789v_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);

static esp_err_t panel_st7789v_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save current value of LCD_CMD_COLMOD register
    const st7789v_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} st7789v_panel_t;

esp_err_t
esp_lcd_new_panel_st7789v(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                          esp_lcd_panel_handle_t *ret_panel) {
    esp_err_t ret = ESP_OK;
    st7789v_panel_t *st7789v = NULL;
    gpio_config_t io_conf = {0};

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    st7789v = (st7789v_panel_t *) calloc(1, sizeof(st7789v_panel_t));
    ESP_GOTO_ON_FALSE(st7789v, ESP_ERR_NO_MEM, err, TAG, "no mem for st7789v panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        st7789v->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        st7789v->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }
#else
    switch (panel_dev_config->rgb_endian) {
        case LCD_RGB_ENDIAN_RGB:
            st7789v->madctl_val = 0;
            break;
        case LCD_RGB_ENDIAN_BGR:
            st7789v->madctl_val |= LCD_CMD_BGR_BIT;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb endian");
            break;
    }
#endif

    switch (panel_dev_config->bits_per_pixel) {
        case 16: // RGB565
            st7789v->colmod_val = 0x55;
            st7789v->fb_bits_per_pixel = 16;
            break;
        case 18: // RGB666
            st7789v->colmod_val = 0x66;
            // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
            st7789v->fb_bits_per_pixel = 24;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
            break;
    }

    st7789v->io = io;
    st7789v->reset_gpio_num = panel_dev_config->reset_gpio_num;
    st7789v->reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        st7789v->init_cmds = ((st7789v_vendor_config_t *) panel_dev_config->vendor_config)->init_cmds;
        st7789v->init_cmds_size = ((st7789v_vendor_config_t *) panel_dev_config->vendor_config)->init_cmds_size;
    }
    st7789v->base.del = panel_st7789v_del;
    st7789v->base.reset = panel_st7789v_reset;
    st7789v->base.init = panel_st7789v_init;
    st7789v->base.draw_bitmap = panel_st7789v_draw_bitmap;
    st7789v->base.invert_color = panel_st7789v_invert_color;
    st7789v->base.set_gap = panel_st7789v_set_gap;
    st7789v->base.mirror = panel_st7789v_mirror;
    st7789v->base.swap_xy = panel_st7789v_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    st7789v->base.disp_off = panel_st7789v_disp_on_off;
#else
    st7789v->base.disp_on_off = panel_st7789v_disp_on_off;
#endif
    *ret_panel = &(st7789v->base);
    ESP_LOGD(TAG, "new st7789v panel @%p", st7789v);

    ESP_LOGI(TAG, "LCD panel create success, version: %d.%d.%d", ESP_LCD_ST7789V_VER_MAJOR, ESP_LCD_ST7789V_VER_MINOR,
             ESP_LCD_ST7789V_VER_PATCH);

    return ESP_OK;

    err:
    if (st7789v) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(st7789v);
    }
    return ret;
}

static esp_err_t panel_st7789v_del(esp_lcd_panel_t *panel) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);

    if (st7789v->reset_gpio_num >= 0) {
        gpio_reset_pin(st7789v->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del st7789v panel @%p", st7789v);
    free(st7789v);
    return ESP_OK;
}

static esp_err_t panel_st7789v_reset(esp_lcd_panel_t *panel) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;

    // perform hardware reset
    if (st7789v->reset_gpio_num >= 0) {
        gpio_set_level(st7789v->reset_gpio_num, st7789v->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7789v->reset_gpio_num, !st7789v->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t data_bytes; // Length of data in above data array; 0xFF = end of cmds.
} lcd_init_cmd_t;

static const st7789v_lcd_init_cmd_t vendor_specific_init_default[] = {
        /* {cmd, { data }, data_size, delay_ms} */
        {0x11, (uint8_t []){0x00}, 1, 120},
        {0xB2, (uint8_t []){0x0B, 0x0B, 0x00, 0x33, 0x33}, 5, 0},
        {0xB7, (uint8_t []){0x11}, 1, 0},
        {0xBB, (uint8_t []){0x2F}, 1, 0},
        {0xC0, (uint8_t []){0x2C}, 1, 0},
        {0xC2, (uint8_t []){0x01}, 1, 0},
        {0xC3, (uint8_t []){0x0D}, 1, 0},
        {0xC4, (uint8_t []){0x20}, 1, 0},
        {0xC6, (uint8_t []){0x13}, 1, 0},
        {0xD0, (uint8_t []){0xA4, 0xA1}, 2, 0},
        {0xD6, (uint8_t []){0xA1}, 1, 0},
        {0xE0, (uint8_t []){0xF0, 0x04, 0x07, 0x09, 0x07, 0x13, 0x25, 0x33, 0x3C, 0x34, 0x10, 0x10, 0x29, 0x32}, 14, 0},
        {0xE1, (uint8_t []){0xF0, 0x05, 0x08, 0x0A, 0x09, 0x05, 0x25, 0x32, 0x3B, 0x3B, 0x17, 0x18, 0x2E, 0x37}, 14, 0},
        {0xE4, (uint8_t []){0x25, 0x00, 0x00}, 3, 0},
        {0x21, (uint8_t []){0x00}, 1, 0},
        {0x29, (uint8_t []){0x00}, 1, 0},
        {0x2A, (uint8_t []){0x00, 0x00, 0x00, 0xEF}, 4, 0},
        {0x2B, (uint8_t []){0x00, 0x14, 0x01, 0x2B}, 4, 0},
        {0x2C, (uint8_t []){0x00}, 1, 0},
};

static esp_err_t panel_st7789v_init(esp_lcd_panel_t *panel) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG, "send command failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
            st7789v->madctl_val,
    }, 1), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {
            st7789v->colmod_val,
    }, 1), TAG, "send command failed");

    const st7789v_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    if (st7789v->init_cmds) {
        init_cmds = st7789v->init_cmds;
        init_cmds_size = st7789v->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(st7789v_lcd_init_cmd_t);
    }

    bool is_cmd_overwritten = false;
    for (int i = 0; i < init_cmds_size; i++) {
        // Check if the command has been used or conflicts with the internal
        switch (init_cmds[i].cmd) {
            case LCD_CMD_MADCTL:
                is_cmd_overwritten = true;
                st7789v->madctl_val = ((uint8_t *) init_cmds[i].data)[0];
                break;
            case LCD_CMD_COLMOD:
                is_cmd_overwritten = true;
                st7789v->colmod_val = ((uint8_t *) init_cmds[i].data)[0];
                break;
            default:
                is_cmd_overwritten = false;
                break;
        }

        if (is_cmd_overwritten) {
            ESP_LOGW(TAG, "The %02Xh command has been used and will be overwritten by external initialization sequence",
                     init_cmds[i].cmd);
        }

        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes),
                            TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
    }
    ESP_LOGD(TAG, "send init commands success");

    return ESP_OK;
}

static esp_err_t panel_st7789v_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                           const void *color_data) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = st7789v->io;

    x_start += st7789v->x_gap;
    x_end += st7789v->x_gap;
    y_start += st7789v->y_gap;
    y_end += st7789v->y_gap;

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
            (x_start >> 8) & 0xFF,
            x_start & 0xFF,
            ((x_end - 1) >> 8) & 0xFF,
            (x_end - 1) & 0xFF,
    }, 4), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
            (y_start >> 8) & 0xFF,
            y_start & 0xFF,
            ((y_end - 1) >> 8) & 0xFF,
            (y_end - 1) & 0xFF,
    }, 4), TAG, "send command failed");
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * st7789v->fb_bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_st7789v_invert_color(esp_lcd_panel_t *panel, bool invert_color_data) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_st7789v_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;
    if (mirror_x) {
        st7789v->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        st7789v->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        st7789v->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        st7789v->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
            st7789v->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_st7789v_swap_xy(esp_lcd_panel_t *panel, bool swap_axes) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;
    if (swap_axes) {
        st7789v->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        st7789v->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
            st7789v->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_st7789v_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    st7789v->x_gap = x_gap;
    st7789v->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_st7789v_disp_on_off(esp_lcd_panel_t *panel, bool on_off) {
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;
    int command = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}
