// =============================================================================
// radar_display.c — ADS-B radar rendered with LVGL 9 on the 720×720 round screen
//
// LVGL 9 drawing API:
//   lv_canvas_init_layer(canvas, &layer)
//   lv_draw_rect / lv_draw_line / lv_draw_arc / lv_draw_label / lv_draw_triangle
//   lv_canvas_finish_layer(canvas, &layer)
//
// Buffer: allocated in PSRAM via heap_caps_malloc, set with lv_canvas_set_draw_buf
// =============================================================================

#include "radar_display.h"
#include "radar_theme.h"
#include "radar_range.h"
#include "config.h"
#include "display.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lvgl.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static const char *TAG = "radar";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD(d) ((d) * (float)(M_PI / 180.0))

// ---------------------------------------------------------------------------
// Canvas state
// ---------------------------------------------------------------------------
static lv_obj_t      *s_canvas  = NULL;
static lv_draw_buf_t *s_draw_buf = NULL;

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------
static void bearing_to_xy(float bearing_deg, float dist_km, float outer_km,
                           int *ox, int *oy)
{
    float r     = (dist_km / outer_km) * RADAR_OUTER_RADIUS;
    float angle = DEG2RAD(bearing_deg - 90.0f);
    *ox = (int)(RADAR_CENTER_X + r * cosf(angle));
    *oy = (int)(RADAR_CENTER_Y + r * sinf(angle));
}

static float haversine_km(float lat1, float lon1, float lat2, float lon2)
{
    float dlat = DEG2RAD(lat2 - lat1);
    float dlon = DEG2RAD(lon2 - lon1);
    float a = sinf(dlat/2)*sinf(dlat/2) +
              cosf(DEG2RAD(lat1))*cosf(DEG2RAD(lat2)) *
              sinf(dlon/2)*sinf(dlon/2);
    return 6371.0f * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

static float bearing_to(float lat1, float lon1, float lat2, float lon2)
{
    float dlon = DEG2RAD(lon2 - lon1);
    float y = sinf(dlon) * cosf(DEG2RAD(lat2));
    float x = cosf(DEG2RAD(lat1))*sinf(DEG2RAD(lat2)) -
              sinf(DEG2RAD(lat1))*cosf(DEG2RAD(lat2))*cosf(dlon);
    return fmodf(atan2f(y, x) * 180.0f / (float)M_PI + 360.0f, 360.0f);
}

// ---------------------------------------------------------------------------
// LVGL 9 drawing primitives — all take a layer pointer
// ---------------------------------------------------------------------------

static void draw_line_px(lv_layer_t *layer,
                         int x0, int y0, int x1, int y1,
                         lv_color_t color, int width)
{
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color   = color;
    dsc.width   = (int32_t)width;
    dsc.opa     = LV_OPA_COVER;
    dsc.p1.x    = (int32_t)x0;
    dsc.p1.y    = (int32_t)y0;
    dsc.p2.x    = (int32_t)x1;
    dsc.p2.y    = (int32_t)y1;
    lv_draw_line(layer, &dsc);
}

static void draw_circle_outline(lv_layer_t *layer,
                                int cx, int cy, int r,
                                lv_color_t color, int width)
{
    lv_draw_arc_dsc_t dsc;
    lv_draw_arc_dsc_init(&dsc);
    dsc.color        = color;
    dsc.width        = (int32_t)width;
    dsc.opa          = LV_OPA_COVER;
    dsc.center.x     = (int32_t)cx;
    dsc.center.y     = (int32_t)cy;
    dsc.radius       = (uint32_t)r;
    dsc.start_angle  = 0;
    dsc.end_angle    = 360;
    lv_draw_arc(layer, &dsc);
}

static void draw_filled_circle(lv_layer_t *layer,
                               int cx, int cy, int r, lv_color_t color)
{
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_color   = color;
    dsc.bg_opa     = LV_OPA_COVER;
    dsc.radius     = LV_RADIUS_CIRCLE;
    dsc.border_width = 0;
    lv_area_t area = { cx - r, cy - r, cx + r, cy + r };
    lv_draw_rect(layer, &dsc, &area);
}

static void draw_text(lv_layer_t *layer,
                      int x, int y, const char *text,
                      lv_color_t color, const lv_font_t *font,
                      lv_text_align_t align)
{
    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);
    dsc.color  = color;
    dsc.font   = font;
    dsc.align  = align;
    dsc.opa    = LV_OPA_COVER;
    dsc.text   = text;
    lv_area_t area = { x - 130, y - 20, x + 130, y + 44 };
    lv_draw_label(layer, &dsc, &area);
}

// Draw a filled triangle as aircraft heading indicator using lv_draw_triangle
static void draw_aircraft_triangle(lv_layer_t *layer,
                                   int cx, int cy, float heading_deg,
                                   int size, lv_color_t color)
{
    float h = DEG2RAD(heading_deg - 90.0f);

    lv_draw_triangle_dsc_t dsc;
    lv_draw_triangle_dsc_init(&dsc);
    dsc.bg_color = color;
    dsc.bg_opa   = LV_OPA_COVER;

    // Tip
    dsc.p[0].x = (int32_t)(cx + size * cosf(h));
    dsc.p[0].y = (int32_t)(cy + size * sinf(h));
    // Left base
    dsc.p[1].x = (int32_t)(cx + size * 0.6f * cosf(h + (float)(M_PI * 0.75)));
    dsc.p[1].y = (int32_t)(cy + size * 0.6f * sinf(h + (float)(M_PI * 0.75)));
    // Right base
    dsc.p[2].x = (int32_t)(cx + size * 0.6f * cosf(h - (float)(M_PI * 0.75)));
    dsc.p[2].y = (int32_t)(cy + size * 0.6f * sinf(h - (float)(M_PI * 0.75)));

    lv_draw_triangle(layer, &dsc);
}

// ---------------------------------------------------------------------------
// Grid
// ---------------------------------------------------------------------------
static void draw_grid(lv_layer_t *layer, const radar_location_t *loc)
{
    // Background
    lv_draw_rect_dsc_t bg;
    lv_draw_rect_dsc_init(&bg);
    bg.bg_color    = RADAR_BG_COLOR;
    bg.bg_opa      = LV_OPA_COVER;
    bg.border_width = 0;
    bg.radius      = 0;
    lv_area_t full = { 0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1 };
    lv_draw_rect(layer, &bg, &full);

    // Concentric rings
    for (int i = 1; i <= RADAR_RING_COUNT; i++) {
        int r = (RADAR_OUTER_RADIUS * i) / RADAR_RING_COUNT;
        draw_circle_outline(layer, RADAR_CENTER_X, RADAR_CENTER_Y, r,
                            RADAR_RING_COLOR, RADAR_RING_STROKE);
    }

    // Crosshairs
    draw_line_px(layer,
                 RADAR_CENTER_X, RADAR_CENTER_Y - RADAR_OUTER_RADIUS,
                 RADAR_CENTER_X, RADAR_CENTER_Y + RADAR_OUTER_RADIUS,
                 RADAR_CROSS_COLOR, RADAR_CROSS_STROKE);
    draw_line_px(layer,
                 RADAR_CENTER_X - RADAR_OUTER_RADIUS, RADAR_CENTER_Y,
                 RADAR_CENTER_X + RADAR_OUTER_RADIUS, RADAR_CENTER_Y,
                 RADAR_CROSS_COLOR, RADAR_CROSS_STROKE);

    // Compass labels
    draw_text(layer, RADAR_CENTER_X, RADAR_CENTER_Y - RADAR_OUTER_RADIUS - 30,
              "N", RADAR_LABEL_COLOR, RADAR_COMPASS_FONT, LV_TEXT_ALIGN_CENTER);
    draw_text(layer, RADAR_CENTER_X, RADAR_CENTER_Y + RADAR_OUTER_RADIUS - 10,
              "S", RADAR_LABEL_COLOR, RADAR_COMPASS_FONT, LV_TEXT_ALIGN_CENTER);
    draw_text(layer, RADAR_CENTER_X - RADAR_OUTER_RADIUS - 30, RADAR_CENTER_Y - 14,
              "W", RADAR_LABEL_COLOR, RADAR_COMPASS_FONT, LV_TEXT_ALIGN_RIGHT);
    draw_text(layer, RADAR_CENTER_X + RADAR_OUTER_RADIUS - 10, RADAR_CENTER_Y - 14,
              "E", RADAR_LABEL_COLOR, RADAR_COMPASS_FONT, LV_TEXT_ALIGN_LEFT);

    // Range label on ring 3
    uint8_t ridx = (loc->range_idx < RANGE_PRESET_COUNT) ? loc->range_idx : 1;
    int ring3_r = (RADAR_OUTER_RADIUS * 3) / RADAR_RING_COUNT;
    draw_text(layer, RADAR_CENTER_X + ring3_r + 6, RADAR_CENTER_Y - 16,
              loc->use_miles ? kRangePresets[ridx].label_mi
                             : kRangePresets[ridx].label_km,
              RADAR_RANGE_COLOR, RADAR_RANGE_FONT, LV_TEXT_ALIGN_LEFT);

    // Center dot
    draw_filled_circle(layer, RADAR_CENTER_X, RADAR_CENTER_Y,
                       RADAR_CENTER_DOT_R, RADAR_CENTER_COLOR);
}

// ---------------------------------------------------------------------------
// Aircraft
// ---------------------------------------------------------------------------
static void draw_aircraft(lv_layer_t *layer,
                          const aircraft_t *ac, const radar_location_t *loc)
{
    float outer_km = radar_range_fetch_km(loc->range_idx);
    float dist_km  = haversine_km(loc->lat, loc->lon, ac->lat, ac->lon);
    float bearing  = bearing_to(loc->lat, loc->lon, ac->lat, ac->lon);

    int ax, ay;
    bearing_to_xy(bearing, dist_km, outer_km, &ax, &ay);

    if (dist_km > outer_km) {
        // Rim dot at correct bearing
        float angle = DEG2RAD(bearing - 90.0f);
        int rx = (int)(RADAR_CENTER_X + (RADAR_OUTER_RADIUS - RIM_DOT_R - 2) * cosf(angle));
        int ry = (int)(RADAR_CENTER_Y + (RADAR_OUTER_RADIUS - RIM_DOT_R - 2) * sinf(angle));
        draw_filled_circle(layer, rx, ry, RIM_DOT_R, RIM_DOT_COLOR);
        return;
    }

    // Heading triangle
    draw_aircraft_triangle(layer, ax, ay, ac->heading_deg,
                           AIRCRAFT_TRIANGLE_SIZE, AIRCRAFT_ICON_COLOR);

    // Speed vector (5-min projection)
    if (ac->speed_kts > 20.0f) {
        float vec_km  = (ac->speed_kts / 1852.0f) * 5.0f;
        float scale   = (float)RADAR_OUTER_RADIUS / outer_km;
        int vx2 = ax + (int)(vec_km * sinf(DEG2RAD(ac->heading_deg)) * scale);
        int vy2 = ay - (int)(vec_km * cosf(DEG2RAD(ac->heading_deg)) * scale);
        // Clamp to outer radius
        float dx = vx2 - RADAR_CENTER_X, dy = vy2 - RADAR_CENTER_Y;
        float mag = sqrtf(dx*dx + dy*dy);
        if (mag > RADAR_OUTER_RADIUS) {
            vx2 = (int)(RADAR_CENTER_X + dx * RADAR_OUTER_RADIUS / mag);
            vy2 = (int)(RADAR_CENTER_Y + dy * RADAR_OUTER_RADIUS / mag);
        }
        draw_line_px(layer, ax, ay, vx2, vy2, AIRCRAFT_VECTOR_COLOR, 2);
    }

    // Tag: callsign / type
    int tag_x  = (ax < RADAR_CENTER_X) ? ax + 22 : ax - 22;
    lv_text_align_t talign = (ax < RADAR_CENTER_X) ? LV_TEXT_ALIGN_LEFT
                                                    : LV_TEXT_ALIGN_RIGHT;
    char line1[16];
    if (ac->callsign[0])
        snprintf(line1, sizeof(line1), "%s", ac->callsign);
    else if (ac->type[0])
        snprintf(line1, sizeof(line1), "%s", ac->type);
    else
        snprintf(line1, sizeof(line1), "?");

    draw_text(layer, tag_x, ay - 22, line1,
              AIRCRAFT_TAG_COLOR, RADAR_TAG_FONT, talign);

    if (ac->alt_ft != 0) {
        char alt_buf[12];
        if (ac->alt_ft >= 1000)
            snprintf(alt_buf, sizeof(alt_buf), "FL%03" PRId32, ac->alt_ft / 100);
        else
            snprintf(alt_buf, sizeof(alt_buf), "%" PRId32 "ft", ac->alt_ft);
        draw_text(layer, tag_x, ay + 2, alt_buf,
                  AIRCRAFT_TAG_COLOR, RADAR_TAG_FONT, talign);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void radar_display_init(void)
{
    // Allocate draw buffer in PSRAM
    size_t buf_bytes = (size_t)DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(lv_color_t);
    void *psram_buf = heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
    if (!psram_buf) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes in PSRAM for canvas",
                 (unsigned)buf_bytes);
        return;
    }

    // Create LVGL draw buffer descriptor
    s_draw_buf = lv_draw_buf_create(DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                    LV_COLOR_FORMAT_RGB565, LV_STRIDE_AUTO);
    if (!s_draw_buf) {
        ESP_LOGE(TAG, "lv_draw_buf_create failed");
        heap_caps_free(psram_buf);
        return;
    }
    // Point it at our PSRAM allocation
    s_draw_buf->data   = psram_buf;
    s_draw_buf->unaligned_data = psram_buf;

    // Create canvas widget
    s_canvas = lv_canvas_create(lv_screen_active());
    lv_canvas_set_draw_buf(s_canvas, s_draw_buf);
    lv_obj_set_pos(s_canvas, 0, 0);

    ESP_LOGI(TAG, "Radar canvas created (%d×%d, %u KB PSRAM)",
             DISPLAY_WIDTH, DISPLAY_HEIGHT, (unsigned)(buf_bytes / 1024));
}

void radar_display_show(void)
{
    display_lock();
    if (s_canvas) lv_scr_load(lv_obj_get_screen(s_canvas));
    display_unlock();
}

void radar_display_update(const aircraft_t *aircraft, int count,
                          const radar_location_t *loc)
{
    if (!s_canvas || !s_draw_buf) return;

    lv_layer_t layer;
    lv_canvas_init_layer(s_canvas, &layer);

    draw_grid(&layer, loc);

    int drawn = 0;
    for (int i = 0; i < count; i++) {
        if (!aircraft[i].valid) continue;
        draw_aircraft(&layer, &aircraft[i], loc);
        drawn++;
    }

    lv_canvas_finish_layer(s_canvas, &layer);
    lv_obj_invalidate(s_canvas);

    ESP_LOGD(TAG, "Rendered %d aircraft", drawn);
}
