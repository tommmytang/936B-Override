/**
 * @file src/ui/page_diag.cpp
 *
 * Every device you registered, in one scrolling list, with the two numbers
 * that actually predict a failed match: temperature and whether the port is
 * still answering. Check this before you queue, not after.
 */
#include <cmath>
#include <cstdio>

#include "internal.hpp"

namespace ui {
namespace internal {

using namespace theme;

namespace {

constexpr std::int32_t INNER_W = CONTENT_W - 12;  // 406
constexpr std::int32_t HEAD_H = 22;
constexpr std::int32_t LIST_H = CONTENT_H - 12 - HEAD_H - 4;
constexpr std::int32_t ROW_H = 36;

lv_obj_t* g_summary = nullptr;
lv_obj_t* g_health = nullptr;
lv_obj_t* g_list = nullptr;

struct Row {
    lv_obj_t* dot = nullptr;
    lv_obj_t* value = nullptr;
    lv_obj_t* bar = nullptr;
    lv_obj_t* sub = nullptr;
};
std::vector<Row> g_rows;

/** "port 1" / "ports 1,2,3" / "" for computed values. */
std::string describe_ports(const DeviceDef& d) {
    if (d.ports.empty()) return "computed";
    char buf[48];
    if (d.ports.size() == 1) {
        std::snprintf(buf, sizeof(buf), "port %d", std::abs(d.ports[0]));
        return buf;
    }
    std::string out = "ports ";
    for (std::size_t i = 0; i < d.ports.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%s%d", i ? "," : "", std::abs(d.ports[i]));
        out += buf;
    }
    return out;
}

bool is_motor(const DeviceDef& d) {
    return d.kind == DeviceKind::Motor || d.kind == DeviceKind::MotorGroup;
}

}  // namespace

void build_diag(lv_obj_t* parent) {
    g_summary = text(parent, "", f_small(), TEXT);
    lv_obj_set_pos(g_summary, 2, 4);

    g_health = text(parent, "", f_small(), OK);
    lv_obj_align(g_health, LV_ALIGN_TOP_RIGHT, -2, 4);

    g_list = blank(parent, INNER_W, LIST_H);
    lv_obj_set_pos(g_list, 0, HEAD_H + 4);
    lv_obj_add_flag(g_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(g_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(g_list, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_set_flex_flow(g_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_list, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_right(g_list, 4, LV_PART_MAIN);

    const std::vector<DeviceDef>& defs = devices();
    g_rows.resize(defs.size());

    for (std::size_t i = 0; i < defs.size(); ++i) {
        const DeviceDef& d = defs[i];
        lv_obj_t* row = card(g_list, INNER_W - 8, ROW_H);
        lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);

        g_rows[i].dot = dot(row, 7, TEXT_FAINT);
        lv_obj_align(g_rows[i].dot, LV_ALIGN_LEFT_MID, 8, 0);

        lv_obj_t* name = text(row, d.label.c_str(), f_small(), TEXT);
        lv_obj_align(name, LV_ALIGN_TOP_LEFT, 22, 3);

        g_rows[i].sub = text(row, describe_ports(d).c_str(), f_micro(), TEXT_FAINT);
        lv_obj_align(g_rows[i].sub, LV_ALIGN_BOTTOM_LEFT, 22, -3);

        g_rows[i].value = text(row, "--", f_small(), TEXT);
        lv_obj_align(g_rows[i].value, LV_ALIGN_RIGHT_MID, -8, 0);

        if (is_motor(d)) {
            /* A motor's temperature deserves a bar: 55 C is where the V5 starts
             * cutting power, so the eye should catch it before the number does. */
            g_rows[i].bar = meter(row, 92, 4);
            lv_obj_align(g_rows[i].bar, LV_ALIGN_BOTTOM_RIGHT, -8, -5);
            lv_obj_align(g_rows[i].value, LV_ALIGN_TOP_RIGHT, -8, 3);
        }
    }

    if (defs.empty()) {
        lv_obj_t* empty =
            text(g_list, "Nothing is being tracked yet.\n\n"
                         "ui::track_motor_group(\"Drive L\", {1, -2, 3});\n"
                         "ui::track_imu(\"IMU\", 10);\n"
                         "ui::track_value(\"Lift\", \"deg\", read_lift);",
                 f_small(), TEXT_FAINT);
        lv_obj_set_style_pad_top(empty, 24, LV_PART_MAIN);
        lv_obj_set_style_text_line_space(empty, 3, LV_PART_MAIN);
    }
}

/*
 * NOTE: LVGL's lv_label_set_text_fmt() uses lv_snprintf, which is built
 * without floating point support -- "%.1f" renders the literal text "f" and,
 * worse, consumes no argument, so any conversion after it reads the wrong
 * vararg. Always format floats with std::snprintf and set the text plainly.
 */
void update_diag(const Snapshot& s) {
    const std::vector<DeviceDef>& defs = devices();
    char buf[96];

    lv_label_set_text_fmt(g_summary, "%d tracked", static_cast<int>(defs.size()));

    if (s.offline_count > 0) {
        lv_label_set_text_fmt(g_health, LV_SYMBOL_WARNING " %d not responding",
                              s.offline_count);
        lv_obj_set_style_text_color(g_health, c(DANGER), LV_PART_MAIN);
    } else if (s.hottest_temp >= 50) {
        std::snprintf(buf, sizeof(buf), LV_SYMBOL_WARNING " %s at %.0f C",
                      s.hottest_label.c_str(), s.hottest_temp);
        lv_label_set_text(g_health, buf);
        lv_obj_set_style_text_color(g_health, c(WARN), LV_PART_MAIN);
    } else if (defs.empty()) {
        lv_label_set_text(g_health, "");
    } else {
        lv_label_set_text(g_health, LV_SYMBOL_OK " all healthy");
        lv_obj_set_style_text_color(g_health, c(OK), LV_PART_MAIN);
    }
    lv_obj_align(g_health, LV_ALIGN_TOP_RIGHT, -2, 4);

    const std::size_t n = std::min(g_rows.size(), s.devices.size());
    for (std::size_t i = 0; i < n; ++i) {
        const DeviceDef& d = defs[i];
        const DeviceSample& sample = s.devices[i];
        Row& row = g_rows[i];

        const bool bad = !sample.online || sample.bad_port != 0;
        std::uint32_t tint = OK;
        if (!sample.online) {
            tint = DANGER;
        } else if (sample.bad_port != 0) {
            tint = WARN;
        } else if (is_motor(d)) {
            tint = heat_color(sample.primary, 50, 60);
        }
        lv_obj_set_style_bg_color(row.dot, c(tint), LV_PART_MAIN);

        if (!sample.online) {
            lv_label_set_text(row.value, "OFFLINE");
            lv_obj_set_style_text_color(row.value, c(DANGER), LV_PART_MAIN);
        } else if (d.kind == DeviceKind::Flag) {
            lv_label_set_text(row.value, sample.flag ? "YES" : "NO");
            lv_obj_set_style_text_color(row.value, c(sample.flag ? OK : TEXT_DIM),
                                        LV_PART_MAIN);
        } else {
            std::snprintf(buf, sizeof(buf), "%.1f %s", sample.primary, d.unit.c_str());
            lv_label_set_text(row.value, buf);
            lv_obj_set_style_text_color(row.value, c(is_motor(d) ? tint : TEXT), LV_PART_MAIN);
        }

        if (row.bar != nullptr) {
            /* 20 C (cold) to 70 C (shut down) mapped across the bar */
            const double pct = (sample.primary - 20.0) / 50.0 * 100.0;
            lv_bar_set_value(row.bar, static_cast<std::int32_t>(std::fmin(std::fmax(pct, 0.0), 100.0)),
                             LV_ANIM_OFF);
            lv_obj_set_style_bg_color(row.bar, c(tint), LV_PART_INDICATOR);
        }

        if (sample.bad_port != 0) {
            lv_label_set_text_fmt(row.sub, "port %d unplugged", sample.bad_port);
            lv_obj_set_style_text_color(row.sub, c(WARN), LV_PART_MAIN);
        } else if (bad) {
            lv_obj_set_style_text_color(row.sub, c(DANGER), LV_PART_MAIN);
        } else {
            lv_obj_set_style_text_color(row.sub, c(TEXT_FAINT), LV_PART_MAIN);
        }
        lv_obj_align(row.sub, LV_ALIGN_BOTTOM_LEFT, 22, -3);
        lv_obj_align(row.value, row.bar != nullptr ? LV_ALIGN_TOP_RIGHT : LV_ALIGN_RIGHT_MID,
                     -8, row.bar != nullptr ? 3 : 0);
    }
}

}  // namespace internal
}  // namespace ui
