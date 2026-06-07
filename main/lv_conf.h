/**
 * lv_conf.h — LVGL 9 configuration for ESP32-P4 Plane Radar
 * Enable the Montserrat fonts we need; everything else is left at defaults.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* Color depth: 16 (RGB565) matches our DSI panel pixel format */
#define LV_COLOR_DEPTH 16

/* Enable the Montserrat font sizes used in radar_theme.h */
#define LV_FONT_MONTSERRAT_14 1   /* default, always on */
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_28 1

/* Use the custom tick provided by esp_timer in display.c */
#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "esp_timer.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR ((uint32_t)(esp_timer_get_time() / 1000ULL))

/* Memory: use stdlib malloc so LVGL internal allocs go to heap */
#define LV_MEM_CUSTOM 1
#define LV_MEM_CUSTOM_INCLUDE <stdlib.h>
#define LV_MEM_CUSTOM_ALLOC   malloc
#define LV_MEM_CUSTOM_FREE    free
#define LV_MEM_CUSTOM_REALLOC realloc

/* Enable canvas widget */
#define LV_USE_CANVAS 1

/* Enable arc, label, line widgets used in status screens */
#define LV_USE_LABEL  1
#define LV_USE_LINE   1
#define LV_USE_ARC    1

/* Default display refresh period (ms) */
#define LV_DEF_REFR_PERIOD 33

/* Log level */
#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF 1

#endif /* LV_CONF_H */
