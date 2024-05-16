/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "lvgl.h"
#include "esp_lcd_st7789v.h"
#include "esp_lcd_touch_cst816s.h"
#include "ui/ui.h"

#define EXAMPLE_LCD_H_RES               (240)           /* LCD Horizontal resolution */
#define EXAMPLE_LCD_V_RES               (280)           /* LCD Vertical resolution */
#define EXAMPLE_LCD_BIT_PER_PIXEL       (16)            /* LCD bit per pixel */

#define EXAMPLE_LCD_HOST                SPI2_HOST
#define EXAMPLE_PIN_NUM_LCD_DC          (GPIO_NUM_47)    /* GPIO number for SPI DC*/
#define EXAMPLE_PIN_NUM_LCD_CS          (GPIO_NUM_38)   /* GPIO number for SPI CS*/
#define EXAMPLE_PIN_NUM_LCD_PCLK        (GPIO_NUM_39)   /* GPIO number for SPI SCL*/
#define EXAMPLE_PIN_NUM_LCD_DATA0       (GPIO_NUM_40)   /* GPIO number for SPI SDA*/
#define EXAMPLE_PIN_NUM_LCD_RST         (GPIO_NUM_41)   /* GPIO number for SPI RESET*/

#define I2C_MASTER_NUM                  0               /* I2C master i2c port number */
#define I2C_MASTER_SCL_IO               (GPIO_NUM_2)    /* GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO               (GPIO_NUM_1)    /* GPIO number used for I2C master data  */
#define EXAMPLE_LCD_TOUCH_RST           (-1)    /* GPIO number used for touch pad reset pin */
#define EXAMPLE_LCD_TOUCH_INT           (GPIO_NUM_42)    /* GPIO number used for touch pad interrupt pin */
#define I2C_MASTER_FREQ_HZ              (400 * 1000)      /*!< I2C master clock frequency */

#define EXAMPLE_LVGL_BUFFER_SIZE        EXAMPLE_LCD_H_RES * 50 * sizeof(lv_color_t)     /*!< LVGL buffer size */

#define EXAMPLE_LVGL_TICK_PERIOD_MS     2               /*!< LVGL tick period in ms */
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS  500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS  1
#define EXAMPLE_LVGL_TASK_STACK_SIZE    (5 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY      2

static char *TAG = "main";

static SemaphoreHandle_t touch_mux = NULL;
static SemaphoreHandle_t lvgl_mux = NULL;

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata,
                                            void *user_ctx) {
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *) user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    /* Copy a buffer's content to a specific area of the display */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    if (xSemaphoreTake(touch_mux, 0) == pdTRUE) {
        esp_lcd_touch_read_data(drv->user_data);
    }

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt,
                                                          1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void touch_callback(esp_lcd_touch_handle_t tp) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void example_increase_lvgl_tick(void *arg) {
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);

}

static bool example_lvgl_lock(int timeout_ms) {
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void) {
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg) {
    ESP_LOGI(TAG, "Starting LVGL task");
    ESP_LOGI(TAG, "Display LVGL UI");

    ui_init();

    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        /* Lock the mutex due to the LVGL APIs are not thread-safe */
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            /* Release the mutex */
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void app_main(void) {
    static lv_disp_draw_buf_t disp_buf;   /* Contains internal graphic buffer(s) called draw buffer(s) */
    static lv_disp_drv_t disp_drv;   /* Contains callback functions */

    touch_mux = xSemaphoreCreateBinary();   /* Define a mutex for the touch */
    assert(touch_mux);

    /* =============== Initialize SPI bus =============== */
    ESP_LOGI(TAG, "Initialize SPI bus");

    const spi_bus_config_t buscfg = ST7789V_PANEL_BUS_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                 EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES *
                                                                 sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* =============== Install panel IO =============== */
    ESP_LOGI(TAG, "Install panel IO");

    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = ST7789V_PANEL_IO_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,
                                                                                EXAMPLE_PIN_NUM_LCD_DC,
                                                                                example_notify_lvgl_flush_ready,
                                                                                &disp_drv);

    /* Attach the LCD to the SPI bus */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) EXAMPLE_LCD_HOST, &io_config, &io_handle));

    /* =============== Install st7789v panel driver =============== */
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
            .rgb_endian     = LCD_RGB_ENDIAN_RGB,
            .bits_per_pixel = EXAMPLE_LCD_BIT_PER_PIXEL,
    };
    ESP_LOGI(TAG, "Install ST7789V panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789v(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    /* =============== Initialize I2C bus =============== */
    ESP_LOGI(TAG, "Initialize I2C bus");

    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t i2c_conf = {
            .mode               = I2C_MODE_MASTER,
            .sda_io_num         = I2C_MASTER_SDA_IO,
            .scl_io_num         = I2C_MASTER_SCL_IO,
            .sda_pullup_en      = GPIO_PULLUP_ENABLE,
            .scl_pullup_en      = GPIO_PULLUP_ENABLE,
            .master.clk_speed   = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &i2c_conf);
    i2c_driver_install(i2c_master_port, i2c_conf.mode, 0, 0, 0);

    /* =============== Initialize touch component =============== */
    ESP_LOGI(TAG, "Initialize touch component");

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();

    esp_lcd_touch_config_t tp_cfg = {
            .x_max          = EXAMPLE_LCD_H_RES,
            .y_max          = EXAMPLE_LCD_V_RES,
            .rst_gpio_num   = EXAMPLE_LCD_TOUCH_RST,
            .int_gpio_num   = EXAMPLE_LCD_TOUCH_INT,
            .levels = {
                    .reset      = 0,
                    .interrupt  = 0,
            },
            .flags = {
                    .swap_xy    = 0,
                    .mirror_x   = 0,
                    .mirror_y   = 0,
            },
            .interrupt_callback = touch_callback,
    };

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t) I2C_MASTER_NUM, &tp_io_config, &tp_io_handle);
    esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);

    /* =============== Initialize LVGL =============== */
    ESP_LOGI(TAG, "Initialize LVGL");

    lv_init();
    /* Alloc draw buffers used by LVGL */
    /* It's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized */
    lv_color_t *buf1 = (lv_color_t *) heap_caps_malloc(EXAMPLE_LVGL_BUFFER_SIZE, MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *) heap_caps_malloc(EXAMPLE_LVGL_BUFFER_SIZE, MALLOC_CAP_DMA);
    assert(buf2);
    /* Initialize LVGL draw buffers */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 50);

    /* =============== Register display driver to LVGL =============== */
    ESP_LOGI(TAG, "Register display driver to LVGL");

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.offset_x = 0;
    disp_drv.offset_y = 20;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    /* =============== Register touch driver to LVGL =============== */
    ESP_LOGI(TAG, "Register touch driver to LVGL");

    static lv_indev_drv_t indev_drv;    /* Input device driver (Touch) */
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);

    /* =============== Register touch driver to LVGL =============== */
    ESP_LOGI(TAG, "Install LVGL tick timer");

    /* Tick interface for LVGL (using esp_timer to generate 2ms periodic event) */
    const esp_timer_create_args_t lvgl_tick_timer_args = {
            .callback   = &example_increase_lvgl_tick,
            .name       = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    /* =============== Create LVGL Task =============== */
    ESP_LOGI(TAG, "Run LVGL Task");

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
}
