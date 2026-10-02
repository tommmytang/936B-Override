/**
 * @file src/ui/page_dash.cpp
 *
 * The at-a-glance match screen: battery gauge on the left, what-will-run
 * banner and four live statistics on the right. Designed to be readable from
 * arm's length while the robot is on the field.
 */
#include <cmath>
#include <cstdio>

#include "internal.hpp"

namespace ui {
namespace internal {

using namespace theme;

namespace {

constexpr std::int32_t LEFT_W = 128;
constexpr std::int32_t RIGHT_X = LEFT_W + 6;
constexpr std::int32_t RIGHT_W = CONTENT_W - 12 - RIGHT_X;  // 272
constexpr std::int32_t BANNER_H = 40;
constexpr std::int32_t TILE_W = (RIGHT_W - 6) / 2;          // 133
constexpr std::int32_t TILE_H = 75;

lv_obj_t* g_arc = nullptr;
lv_obj_t* g_batt_pct = nullptr;
lv_obj_t* g_batt_detail = nullptr;
lv_obj_t* g_banner = nullptr;
lv_obj_t* g_banner_caption = nullptr;
lv_obj_t* g_banner_name = nullptr;
lv_obj_t* g_banner_ctrl = nullptr;
lv_obj_t* g_tile_value[4] = {};
lv_obj_t* g_tile_sub[4] = {};

/** Adds a small caption under a tile's value. */
lv_obj_t* tile_subtext(lv_obj_t* tile) {
    lv_obj_t* l = text(tile, "", f_micro(), TEXT_FAINT);
    lv_obj_align(l, LV_ALIGN_BOTTOM_RIGHT, 0, 1);
    return l;
}

}  // namespace

void build_dash(lv_obj_t* parent) {
    /* ---------------------------------------------------------- battery -- */
    lv_obj_t* left = card(parent, LEFT_W, CONTENT_H - 12);
    lv_obj_set_pos(left, 0, 0);

    lv_obj_t* cap = text(left, "BATTERY", f_micro(), TEXT_FAINT);
    lv_obj_align(cap, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_letter_space(cap, 1, LV_PART_MAIN);

    g_arc = lv_arc_create(left);
    lv_obj_set_size(g_arc, 96, 96);
    lv_obj_align(g_arc, LV_ALIGN_TOP_MID, 0, 16);
    lv_arc_set_rotation(g_arc, 135);
    lv_arc_set_bg_angles(g_arc, 0, 270);
    lv_arc_set_range(g_arc, 0, 100);
    lv_arc_set_value(g_arc, 0);
    lv_obj_remove_style(g_arc, nullptr, LV_PART_KNOB);
    lv_obj_remove_flag(g_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(g_arc, 9, LV_PART_MAIN);
    lv_obj_set_style_arc_color(g_arc, c(ACCENT_DEEP), LV_PART_MAIN);
    lv_obj_set_style_arc_width(g_arc, 9, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(g_arc, c(ACCENT_HI), LV_PART_INDICATOR);

    g_batt_pct = text(left, "--", f_huge(), TEXT);
    lv_obj_align(g_batt_pct, LV_ALIGN_TOP_MID, 0, 46);

    g_batt_detail = text(left, "-- V\n-- A\n-- " LV_SYMBOL_HOME, f_small(), TEXT_DIM);
    lv_obj_set_style_text_align(g_batt_detail, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_line_space(g_batt_detail, 3, LV_PART_MAIN);
    lv_obj_align(g_batt_detail, LV_ALIGN_BOTTOM_MID, 0, 0);

    /* ------------------------------------------------ what will run next -- */
    g_banner = card(parent, RIGHT_W, BANNER_H);
    lv_obj_set_pos(g_banner, RIGHT_X, 0);
    lv_obj_set_style_pad_all(g_banner, 5, LV_PART_MAIN);

    g_banner_caption = text(g_banner, "AUTON", f_micro(), TEXT_FAINT);
    lv_obj_align(g_banner_caption, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_letter_space(g_banner_caption, 1, LV_PART_MAIN);

    g_banner_name = text(g_banner, "None", f_title(), TEXT);
    lv_obj_align(g_banner_name, LV_ALIGN_BOTTOM_LEFT, 0, 1);

    g_banner_ctrl = text(g_banner, LV_SYMBOL_BLUETOOTH " --%", f_small(), TEXT_DIM);
    lv_obj_align(g_banner_ctrl, LV_ALIGN_RIGHT_MID, 0, 0);

    /* ------------------------------------------------------------ tiles -- */
    const char* captions[4] = {"HEADING", "POSITION", "HOTTEST", "CURRENT"};
    for (int i = 0; i < 4; ++i) {
        lv_obj_t* t = stat_tile(parent, TILE_W, TILE_H, captions[i], &g_tile_value[i]);
        lv_obj_set_pos(t, RIGHT_X + (i % 2) * (TILE_W + 6),
                       BANNER_H + 6 + (i / 2) * (TILE_H + 6));
        g_tile_sub[i] = tile_subtext(t);
    }
}

/*
 * NOTE: LVGL's lv_label_set_text_fmt() uses lv_snprintf, which is built
 * without floating point support -- "%.1f" renders the literal text "f" and,
 * worse, consumes no argument, so any conversion after it reads the wrong
 * vararg. Always format floats with std::snprintf and set the text plainly.
 */
void update_dash(const Snapshot& s) {
    char buf[64];

    /* ---- battery ---- */
    const int pct = static_cast<int>(s.battery_pct + 0.5);
    lv_arc_set_value(g_arc, pct);
    const std::uint32_t batt_tint =
        (pct >= 50) ? OK : (pct >= 25 ? WARN : DANGER);
    lv_obj_set_style_arc_color(g_arc, c(batt_tint), LV_PART_INDICATOR);
    lv_label_set_text_fmt(g_batt_pct, "%d", pct);
    lv_obj_set_style_text_color(g_batt_pct, c(batt_tint), LV_PART_MAIN);
    lv_obj_align(g_batt_pct, LV_ALIGN_TOP_MID, 0, 46);

    std::snprintf(buf, sizeof(buf), "%.1f V\n%.1f A\n%.0f C", s.battery_volts,
                  s.battery_current, s.battery_temp);
    lv_label_set_text(g_batt_detail, buf);

    /* ---- banner ---- */
    const AutonDef* a = selected_auton();
    const char* tab = selected_alliance() == Alliance::Red      ? "RED AUTON"
                      : selected_alliance() == Alliance::Blue   ? "BLUE AUTON"
                                                                : "SKILLS";
    lv_label_set_text(g_banner_caption, tab);
    lv_obj_set_style_text_color(g_banner_caption, c(alliance_color()), LV_PART_MAIN);
    lv_label_set_text(g_banner_name, a != nullptr ? a->name.c_str() : "None selected");
    lv_obj_set_style_text_color(g_banner_name, c(a != nullptr ? TEXT : TEXT_FAINT),
                                LV_PART_MAIN);
    lv_obj_set_style_border_color(g_banner, c(alliance_color()), LV_PART_MAIN);

    lv_label_set_text_fmt(g_banner_ctrl, LV_SYMBOL_BLUETOOTH " %d%%", s.controller_pct);
    lv_obj_set_style_text_color(g_banner_ctrl, c(s.controller_linked ? TEXT_DIM : DANGER),
                                LV_PART_MAIN);
    lv_obj_align(g_banner_ctrl, LV_ALIGN_RIGHT_MID, 0, 0);

    /* ---- heading ---- */
    if (s.pose_valid) {
        std::snprintf(buf, sizeof(buf), "%.0f", s.pose.theta);
        lv_label_set_text(g_tile_value[0], buf);
        lv_label_set_text(g_tile_sub[0], "deg");
    } else {
        lv_label_set_text(g_tile_value[0], "--");
        lv_label_set_text(g_tile_sub[0], "no odom");
    }
    lv_obj_align(g_tile_value[0], LV_ALIGN_BOTTOM_LEFT, 0, 1);

    /* ---- position ---- */
    if (s.pose_valid) {
        std::snprintf(buf, sizeof(buf), "%.0f, %.0f", s.pose.x, s.pose.y);
        lv_label_set_text(g_tile_value[1], buf);
        lv_label_set_text(g_tile_sub[1], "in");
    } else {
        lv_label_set_text(g_tile_value[1], "--");
        lv_label_set_text(g_tile_sub[1], "no odom");
    }
    lv_obj_align(g_tile_value[1], LV_ALIGN_BOTTOM_LEFT, 0, 1);

    /* ---- hottest motor ---- */
    if (s.hottest_temp > 0) {
        std::snprintf(buf, sizeof(buf), "%.0f C", s.hottest_temp);
        lv_label_set_text(g_tile_value[2], buf);
        lv_obj_set_style_text_color(g_tile_value[2],
                                    c(heat_color(s.hottest_temp, 50, 60)), LV_PART_MAIN);
        lv_label_set_text(g_tile_sub[2], s.hottest_label.c_str());
    } else {
        lv_label_set_text(g_tile_value[2], "--");
        lv_obj_set_style_text_color(g_tile_value[2], c(TEXT), LV_PART_MAIN);
        lv_label_set_text(g_tile_sub[2], "no motors");
    }
    lv_obj_align(g_tile_value[2], LV_ALIGN_BOTTOM_LEFT, 0, 1);

    /* ---- current draw ---- */
    std::snprintf(buf, sizeof(buf), "%.1f A", s.drive_current);
    lv_label_set_text(g_tile_value[3], buf);
    lv_obj_align(g_tile_value[3], LV_ALIGN_BOTTOM_LEFT, 0, 1);
    if (s.offline_count > 0) {
        lv_label_set_text_fmt(g_tile_sub[3], "%d offline", s.offline_count);
        lv_obj_set_style_text_color(g_tile_sub[3], c(DANGER), LV_PART_MAIN);
    } else {
        lv_label_set_text(g_tile_sub[3], "all ports ok");
        lv_obj_set_style_text_color(g_tile_sub[3], c(TEXT_FAINT), LV_PART_MAIN);
    }
    lv_obj_align(g_tile_sub[3], LV_ALIGN_BOTTOM_RIGHT, 0, 1);
}

}  // namespace internal
}  // namespace ui
