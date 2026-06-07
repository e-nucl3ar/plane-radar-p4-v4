// =============================================================================
// display.c — MIPI-DSI display + LVGL 9 initialisation
//
// LVGL 9 display API:
//   lv_display_create() → lv_display_set_flush_cb() → lv_display_set_buffers()
//   (replaces LVGL 8's lv_disp_drv_t / lv_disp_draw_buf_t pattern)
// =============================================================================

#include "display.h"
#include "config.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_dpi.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lvgl.h"

static const char *TAG = "display";

// DPI timing for 4-inch DSI LCD (C) — 720×720
#define PANEL_DPI_CLK_MHZ        48
#define PANEL_HSYNC_BACK_PORCH   32
#define PANEL_HSYNC_PULSE_WIDTH  200
#define PANEL_HSYNC_FRONT_PORCH  120
#define PANEL_VSYNC_BACK_PORCH   4
#define PANEL_VSYNC_PULSE_WIDTH  16
#define PANEL_VSYNC_FRONT_PORCH  8

static SemaphoreHandle_t        s_lvgl_mux  = NULL;
static esp_lcd_panel_handle_t   s_panel     = NULL;
static lv_display_t            *s_disp      = NULL;
static esp_ldo_channel_handle_t s_ldo_mipi  = NULL;

// ---------------------------------------------------------------------------
// LVGL 9 flush callback
// ---------------------------------------------------------------------------
static void lvgl_flush_cb(lv_display_t *disp,
                          const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel =
        (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    esp_lcd_panel_draw_bitmap(panel,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              px_map);
    lv_display_flush_ready(disp);
}

// ---------------------------------------------------------------------------
// LVGL tick timer
// ---------------------------------------------------------------------------
static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// ---------------------------------------------------------------------------
// LVGL handler task
// ---------------------------------------------------------------------------
static void lvgl_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "LVGL handler on core %d", xPortGetCoreID());
    for (;;) {
        xSemaphoreTakeRecursive(s_lvgl_mux, portMAX_DELAY);
        uint32_t ms = lv_timer_handler();
        xSemaphoreGiveRecursive(s_lvgl_mux);
        vTaskDelay(pdMS_TO_TICKS(ms < 5 ? 5 : ms));
    }
}

// ---------------------------------------------------------------------------
// display_init
// ---------------------------------------------------------------------------
esp_err_t display_init(void)
{
    // 1. LDO for MIPI PHY
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &s_ldo_mipi),
                        TAG, "LDO acquire failed");

    // 2. DSI bus
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t dsi_bus_cfg = {
        .bus_id             = 0,
        .num_data_lanes     = DSI_LANE_NUM,
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = DSI_LANE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&dsi_bus_cfg, &dsi_bus),
                        TAG, "DSI bus failed");

    // 3. DBI command IO
    esp_lcd_panel_io_handle_t dbi_io = NULL;
    esp_lcd_dbi_io_config_t dbi_cfg = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,
        .lcd_param_bits  = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &dbi_io),
                        TAG, "DBI IO failed");

    // 4. DPI panel (waveshare/esp_lcd_dsi component)
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .virtual_channel    = 0,
        .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = PANEL_DPI_CLK_MHZ,
        .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .video_timing = {
            .h_size            = DISPLAY_WIDTH,
            .v_size            = DISPLAY_HEIGHT,
            .hsync_back_porch  = PANEL_HSYNC_BACK_PORCH,
            .hsync_pulse_width = PANEL_HSYNC_PULSE_WIDTH,
            .hsync_front_porch = PANEL_HSYNC_FRONT_PORCH,
            .vsync_back_porch  = PANEL_VSYNC_BACK_PORCH,
            .vsync_pulse_width = PANEL_VSYNC_PULSE_WIDTH,
            .vsync_front_porch = PANEL_VSYNC_FRONT_PORCH,
        },
        .flags.use_dma2d = true,
    };
    esp_lcd_panel_dev_config_t panel_dev_cfg = {
        .reset_gpio_num = GPIO_NUM_NC,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_dpi(dsi_bus, &dbi_io, &dpi_cfg, &panel_dev_cfg, &s_panel),
        TAG, "Panel create failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "Panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "Panel on failed");
    ESP_LOGI(TAG, "Panel online %d×%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);

    // 5. Backlight
    if (LCD_BL_PIN != GPIO_NUM_NC) {
        ledc_timer_config_t tmr = {
            .speed_mode      = LEDC_LOW_SPEED_MODE,
            .timer_num       = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz         = LCD_BL_FREQ_HZ,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        ledc_channel_config_t ch = {
            .gpio_num   = LCD_BL_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = LCD_BL_LEDC_CH,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 255,
        };
        ledc_timer_config(&tmr);
        ledc_channel_config(&ch);
    }

    // 6. LVGL 9 init
    lv_init();

    // Two draw buffers in PSRAM — 1/10 screen each
    size_t buf_px = DISPLAY_WIDTH * (DISPLAY_HEIGHT / 10);
    void *buf1 = heap_caps_malloc(buf_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    void *buf2 = heap_caps_malloc(buf_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!buf1 || !buf2) {
        ESP_LOGE(TAG, "LVGL draw buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    // LVGL 9: create display object, set resolution, flush callback, buffers
    s_disp = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_display_set_flush_cb(s_disp, lvgl_flush_cb);
    lv_display_set_user_data(s_disp, s_panel);
    lv_display_set_buffers(s_disp, buf1, buf2,
                           buf_px * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    // 7. Tick timer
    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb, .name = "lvgl_tick"
    };
    esp_timer_handle_t tick_timer;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer),
                        TAG, "Tick timer create failed");
    ESP_RETURN_ON_ERROR(
        esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000ULL),
        TAG, "Tick timer start failed");

    // 8. Mutex + handler task
    s_lvgl_mux = xSemaphoreCreateRecursiveMutex();
    configASSERT(s_lvgl_mux);
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL,
                            LVGL_TASK_PRIORITY, NULL, LVGL_TASK_CORE);

    ESP_LOGI(TAG, "Display fully initialised");
    return ESP_OK;
}

void display_lock(void)   { xSemaphoreTakeRecursive(s_lvgl_mux, portMAX_DELAY); }
void display_unlock(void) { xSemaphoreGiveRecursive(s_lvgl_mux); }
lv_disp_t *display_get(void) { return s_disp; }

void display_set_backlight(uint8_t pct)
{
    if (LCD_BL_PIN == GPIO_NUM_NC) return;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_BL_LEDC_CH, (pct * 255u) / 100u);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_BL_LEDC_CH);
}
