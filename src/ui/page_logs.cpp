/**
 * @file src/ui/page_logs.cpp
 *
 * An on-screen console. `printf` goes to a terminal you probably are not
 * holding during a match; this you can read from the field.
 *
 * The list is rebuilt only when a line is actually added, which is rare, so
 * the cost of throwing the labels away and recreating them never lands on a
 * frame that matters.
 */
#include <cstdio>

#include "internal.hpp"

namespace ui {
namespace internal {

using namespace theme;

namespace {

constexpr std::int32_t INNER_W = CONTENT_W - 12;  // 406
constexpr std::int32_t HEAD_H = 22;
constexpr std::int32_t LIST_H = CONTENT_H - 12 - HEAD_H - 4;

lv_obj_t* g_list = nullptr;
lv_obj_t* g_count = nullptr;
lv_obj_t* g_follow_label = nullptr;

std::uint32_t g_seen_revision = 0xFFFFFFFFu;
bool g_follow = true;  //< keep the newest line in view

std::uint32_t level_color(LogLevel l) {
    switch (l) {
        case LogLevel::Warn: return WARN;
        case LogLevel::Error: return DANGER;
        default: return TEXT_DIM;
    }
}

const char* level_icon(LogLevel l) {
    switch (l) {
        case LogLevel::Warn: return LV_SYMBOL_WARNING;
        case LogLevel::Error: return LV_SYMBOL_CLOSE;
        default: return LV_SYMBOL_RIGHT;
    }
}

void on_clear_click(lv_event_t*) {
    {
        std::lock_guard<pros::Mutex> guard(data_lock());
        logs().clear();
    }
    ui::log("Console cleared");
}

void on_follow_click(lv_event_t*) {
    g_follow = !g_follow;
    lv_label_set_text(g_follow_label, g_follow ? "FOLLOW" : "PAUSED");
    lv_obj_set_style_text_color(g_follow_label, c(g_follow ? ACCENT_HI : TEXT_FAINT),
                                LV_PART_MAIN);
}

lv_obj_t* head_button(lv_obj_t* parent, std::int32_t x, std::int32_t w, const char* label,
                      lv_event_cb_t cb, lv_obj_t** out) {
    lv_obj_t* b = card(parent, w, HEAD_H - 2);
    lv_obj_align(b, LV_ALIGN_TOP_RIGHT, x, 0);
    lv_obj_set_style_pad_all(b, 0, LV_PART_MAIN);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* l = text(b, label, f_micro(), ACCENT_HI);
    lv_obj_center(l);
    if (out != nullptr) *out = l;
    return b;
}

}  // namespace

void build_logs(lv_obj_t* parent) {
    g_count = text(parent, "0 lines", f_small(), TEXT_DIM);
    lv_obj_set_pos(g_count, 2, 4);

    head_button(parent, -2, 58, LV_SYMBOL_TRASH " CLEAR", on_clear_click, nullptr);
    head_button(parent, -64, 58, "FOLLOW", on_follow_click, &g_follow_label);

    g_list = blank(parent, INNER_W, LIST_H);
    lv_obj_set_pos(g_list, 0, HEAD_H + 4);
    lv_obj_set_style_bg_color(g_list, c(SURFACE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_list, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(g_list, 6, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_list, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(g_list, c(BORDER), LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_list, 6, LV_PART_MAIN);
    lv_obj_add_flag(g_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(g_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(g_list, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_set_flex_flow(g_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_list, 2, LV_PART_MAIN);
}

void update_logs(const Snapshot&) {
    std::vector<LogLine> lines;
    std::uint32_t revision;
    {
        std::lock_guard<pros::Mutex> guard(data_lock());
        revision = log_revision();
        if (revision == g_seen_revision) return;  // nothing new; leave the DOM alone
        lines = logs();
    }
    g_seen_revision = revision;

    lv_obj_clean(g_list);
    char stamp[16];
    char buf[160];

    for (const LogLine& line : lines) {
        format_clock(stamp, sizeof(stamp), line.ms);
        std::snprintf(buf, sizeof(buf), "%s  %s  %s", stamp, level_icon(line.level),
                      line.text.c_str());
        lv_obj_t* l = text(g_list, buf, f_small(), level_color(line.level));
        lv_obj_set_width(l, INNER_W - 18);
        lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
    }

    lv_label_set_text_fmt(g_count, "%d lines", static_cast<int>(lines.size()));

    if (g_follow && !lines.empty()) {
        lv_obj_scroll_to_y(g_list, LV_COORD_MAX / 2, LV_ANIM_OFF);
    }
}

}  // namespace internal
}  // namespace ui
