/**
 * @file src/ui/page_field.cpp
 *
 * A top-down VEX field with your robot drawn on it, live, plus a breadcrumb
 * trail of where it has been. This is the page you leave open while tuning an
 * autonomous -- seeing the path drift is far quicker than reading numbers.
 *
 * Coordinate convention matches LemLib: origin at the centre of the field,
 * +X to the right, +Y away from the driver station, heading in degrees with
 * 0 pointing along +Y and increasing clockwise.
 */
#include <cmath>
#include <cstdio>

#include "internal.hpp"

namespace ui {
namespace internal {

using namespace theme;

namespace {

constexpr std::int32_t BOX = 196;        //< outer size of the field widget
constexpr std::int32_t PLOT = BOX - 4;   //< drawable area inside the 2px border
constexpr double FIELD_IN = 144.0;       //< a VEX field is 12ft square
constexpr double SCALE = PLOT / FIELD_IN;
constexpr std::int32_t HALF = PLOT / 2;
constexpr std::int32_t ROBOT_PX = static_cast<std::int32_t>(18.0 * SCALE);  //< an 18" robot

constexpr std::int32_t RIGHT_X = BOX + 6;
constexpr std::int32_t RIGHT_W = CONTENT_W - 12 - RIGHT_X;  // 204
constexpr std::int32_t TILE_H = 41;

lv_obj_t* g_field = nullptr;
lv_obj_t* g_robot = nullptr;
lv_obj_t* g_nose = nullptr;
lv_obj_t* g_trail[TRAIL_POINTS] = {};
lv_obj_t* g_value[3] = {};
lv_obj_t* g_reset_btn = nullptr;
lv_obj_t* g_trail_btn = nullptr;
lv_obj_t* g_trail_label = nullptr;
lv_obj_t* g_status = nullptr;

std::size_t g_trail_head = 0;
std::size_t g_trail_used = 0;
bool g_trail_on = true;
std::uint32_t g_last_trail_ms = 0;

/** Field inches -> pixels inside the field widget's content area. */
inline std::int32_t px_x(double x) {
    return static_cast<std::int32_t>(HALF + x * SCALE + 0.5);
}
inline std::int32_t px_y(double y) {
    return static_cast<std::int32_t>(HALF - y * SCALE + 0.5);
}

void add_grid_line(lv_obj_t* parent, std::int32_t x, std::int32_t y, std::int32_t w,
                   std::int32_t h, std::uint32_t color) {
    lv_obj_t* l = blank(parent, w, h);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_style_bg_color(l, c(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(l, LV_OPA_COVER, LV_PART_MAIN);
}

void on_reset_click(lv_event_t*) {
    std::function<void()> action;
    {
        std::lock_guard<pros::Mutex> guard(data_lock());
        action = reset_pose_action();
    }
    g_trail_used = 0;
    g_trail_head = 0;
    for (auto& d : g_trail) lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);

    if (action) {
        action();
        ui::log("Odometry reset from the brain");
    } else {
        ui::warn("No reset handler - see ui::on_reset_pose()");
    }
}

void on_trail_click(lv_event_t*) {
    g_trail_on = !g_trail_on;
    if (!g_trail_on) {
        for (auto& d : g_trail) lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
        g_trail_used = 0;
        g_trail_head = 0;
    }
    lv_label_set_text(g_trail_label, g_trail_on ? "TRAIL ON" : "TRAIL OFF");
    lv_obj_set_style_text_color(g_trail_label, c(g_trail_on ? ACCENT_HI : TEXT_FAINT),
                                LV_PART_MAIN);
}

lv_obj_t* small_button(lv_obj_t* parent, std::int32_t w, std::int32_t h, const char* label,
                       lv_event_cb_t cb, lv_obj_t** out_label) {
    lv_obj_t* b = card(parent, w, h);
    lv_obj_set_style_pad_all(b, 0, LV_PART_MAIN);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* l = text(b, label, f_micro(), ACCENT_HI);
    lv_obj_center(l);
    lv_obj_set_style_text_letter_space(l, 1, LV_PART_MAIN);
    if (out_label != nullptr) *out_label = l;
    return b;
}

}  // namespace

void build_field(lv_obj_t* parent) {
    /* ------------------------------------------------------ the field -- */
    g_field = blank(parent, BOX, BOX);
    lv_obj_set_pos(g_field, 0, 3);
    lv_obj_set_style_bg_color(g_field, c(0x100C18), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_field, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_field, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(g_field, c(ACCENT), LV_PART_MAIN);
    lv_obj_set_style_radius(g_field, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_field, 0, LV_PART_MAIN);

    /* 24" foam tile grid, with the centre lines picked out */
    for (int i = 1; i < 6; ++i) {
        const std::int32_t at = static_cast<std::int32_t>(i * 24.0 * SCALE + 0.5);
        const std::uint32_t tint = (i == 3) ? ACCENT_MID : BORDER;
        add_grid_line(g_field, at, 0, 1, PLOT, tint);
        add_grid_line(g_field, 0, at, PLOT, 1, tint);
    }

    /* breadcrumbs first, so the robot always draws on top of them */
    for (auto& d : g_trail) {
        d = dot(g_field, 3, ACCENT_HI);
        lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
    }

    g_robot = blank(g_field, ROBOT_PX, ROBOT_PX);
    lv_obj_set_style_bg_color(g_robot, c(ACCENT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_robot, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_radius(g_robot, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_robot, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(g_robot, c(TEXT), LV_PART_MAIN);

    g_nose = dot(g_field, 7, TEXT);

    /* ------------------------------------------------------- readouts -- */
    const char* caps[3] = {"X  (in)", "Y  (in)", "HEADING  (deg)"};
    for (int i = 0; i < 3; ++i) {
        lv_obj_t* t = stat_tile(parent, RIGHT_W, TILE_H, caps[i], &g_value[i], f_large());
        lv_obj_set_pos(t, RIGHT_X, i * (TILE_H + 4));
        lv_obj_align(g_value[i], LV_ALIGN_BOTTOM_RIGHT, 0, 1);
    }

    const std::int32_t btn_y = 3 * (TILE_H + 4) + 2;
    g_reset_btn = small_button(parent, (RIGHT_W - 6) / 2, 30, LV_SYMBOL_REFRESH " RESET",
                               on_reset_click, nullptr);
    lv_obj_set_pos(g_reset_btn, RIGHT_X, btn_y);

    g_trail_btn = small_button(parent, (RIGHT_W - 6) / 2, 30, "TRAIL ON", on_trail_click,
                               &g_trail_label);
    lv_obj_set_pos(g_trail_btn, RIGHT_X + (RIGHT_W - 6) / 2 + 6, btn_y);

    g_status = text(parent, "", f_micro(), TEXT_FAINT);
    lv_obj_set_pos(g_status, RIGHT_X, btn_y + 34);
    lv_obj_set_width(g_status, RIGHT_W);
    lv_label_set_long_mode(g_status, LV_LABEL_LONG_WRAP);
}

/*
 * NOTE: LVGL's lv_label_set_text_fmt() uses lv_snprintf, which is built
 * without floating point support -- "%.1f" renders the literal text "f" and,
 * worse, consumes no argument, so any conversion after it reads the wrong
 * vararg. Always format floats with std::snprintf and set the text plainly.
 */
void update_field(const Snapshot& s) {
    if (!s.pose_valid) {
        lv_obj_add_flag(g_robot, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(g_nose, LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < 3; ++i) lv_label_set_text(g_value[i], "--");
        lv_label_set_text(g_status, "No odometry source. Call\nui::set_pose_source().");
        return;
    }
    lv_obj_remove_flag(g_robot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(g_nose, LV_OBJ_FLAG_HIDDEN);

    const std::int32_t cx = px_x(s.pose.x);
    const std::int32_t cy = px_y(s.pose.y);
    lv_obj_set_pos(g_robot, cx - ROBOT_PX / 2, cy - ROBOT_PX / 2);
    lv_obj_set_style_bg_color(g_robot, c(alliance_color()), LV_PART_MAIN);

    /* 0 deg faces +Y (up the screen); the angle grows clockwise */
    const double rad = s.pose.theta * M_PI / 180.0;
    const double reach = ROBOT_PX * 0.55;
    lv_obj_set_pos(g_nose, cx + static_cast<std::int32_t>(reach * std::sin(rad)) - 3,
                   cy - static_cast<std::int32_t>(reach * std::cos(rad)) - 3);

    /* drop a breadcrumb a few times a second, not every frame */
    const std::uint32_t now = pros::millis();
    if (g_trail_on && now - g_last_trail_ms >= 150) {
        g_last_trail_ms = now;
        lv_obj_t* d = g_trail[g_trail_head];
        lv_obj_set_pos(d, cx - 1, cy - 1);
        lv_obj_set_style_bg_color(d, c(alliance_color()), LV_PART_MAIN);
        lv_obj_remove_flag(d, LV_OBJ_FLAG_HIDDEN);
        g_trail_head = (g_trail_head + 1) % TRAIL_POINTS;
        if (g_trail_used < TRAIL_POINTS) ++g_trail_used;
    }
    /* fade the tail so the direction of travel reads at a glance */
    for (std::size_t i = 0; i < g_trail_used; ++i) {
        const std::size_t idx = (g_trail_head + TRAIL_POINTS - 1 - i) % TRAIL_POINTS;
        const std::int32_t opa = 230 - static_cast<std::int32_t>(i * (200 / TRAIL_POINTS));
        lv_obj_set_style_bg_opa(g_trail[idx], static_cast<lv_opa_t>(opa < 30 ? 30 : opa),
                                LV_PART_MAIN);
    }

    char buf[24];
    const double shown[3] = {s.pose.x, s.pose.y, s.pose.theta};
    for (int i = 0; i < 3; ++i) {
        std::snprintf(buf, sizeof(buf), "%.1f", shown[i]);
        lv_label_set_text(g_value[i], buf);
    }
    for (int i = 0; i < 3; ++i) lv_obj_align(g_value[i], LV_ALIGN_BOTTOM_RIGHT, 0, 1);

    const bool off_field =
        std::fabs(s.pose.x) > FIELD_IN / 2 || std::fabs(s.pose.y) > FIELD_IN / 2;
    if (s.pose_demo) {
        lv_label_set_text(g_status, LV_SYMBOL_EYE_OPEN " Demo path. No odometry\nwired up yet.");
        lv_obj_set_style_text_color(g_status, c(ACCENT_HI), LV_PART_MAIN);
    } else if (off_field) {
        lv_label_set_text(g_status, LV_SYMBOL_WARNING " Pose is outside the field");
        lv_obj_set_style_text_color(g_status, c(WARN), LV_PART_MAIN);
    } else {
        lv_label_set_text(g_status, "Grid squares are 24 in foam tiles.");
        lv_obj_set_style_text_color(g_status, c(TEXT_FAINT), LV_PART_MAIN);
    }
}

}  // namespace internal
}  // namespace ui
