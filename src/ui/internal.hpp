/**
 * @file src/ui/internal.hpp
 *
 * Shared plumbing between the UI's translation units. Nothing here is part of
 * the public API -- see include/ui/ui.hpp for that.
 *
 * THREADING MODEL
 * ---------------
 * Two actors touch this state:
 *
 *   1. A PROS task ("ui-sample") polls the robot every SAMPLE_MS and writes
 *      into `snapshot()`, holding `data_lock()`.
 *   2. An LVGL timer, which runs inside PROS's display daemon, copies the
 *      snapshot and pushes it into widgets.
 *
 * That split matters: LVGL is not thread safe, so *every* LVGL call in this
 * UI happens on the display daemon. Device reads, which can block on the VDML
 * mutex, happen on the sampler task so they can never stall rendering.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "liblvgl/lvgl.h"
#include "pros/rtos.hpp"
#include "ui/theme.hpp"
#include "ui/ui.hpp"

namespace ui {
namespace internal {

inline constexpr std::uint32_t SAMPLE_MS = 50;   //< robot polling period
inline constexpr std::uint32_t RENDER_MS = 50;   //< widget refresh period
inline constexpr std::size_t MAX_LOG_LINES = 80; //< log console ring buffer
inline constexpr std::size_t TRAIL_POINTS = 40;  //< field map breadcrumbs

/* ======================================================= registered things */

enum class DeviceKind : std::uint8_t {
    Motor, MotorGroup, Imu, Rotation, Distance, Optical, Value, Flag
};

/** Something the user asked us to watch. Filled in before `start()`. */
struct DeviceDef {
    DeviceKind kind = DeviceKind::Value;
    std::string label;
    std::string unit;
    std::vector<std::int8_t> ports;
    std::function<double()> read_value;
    std::function<bool()> read_flag;
};

/** One entry on the autonomous selector. */
struct AutonDef {
    Alliance alliance = Alliance::Red;
    std::string name;
    std::string description;
    AutonFn run;
};

/* =============================================================== sampling */

/** The most recent reading of one tracked device. */
struct DeviceSample {
    bool online = false;
    double primary = 0;    //< temperature (C) for motors, heading for an IMU
    double secondary = 0;  //< current draw (A) for motors
    bool flag = false;
    std::int8_t bad_port = 0;  //< first unplugged port of a group, 0 if healthy
};

enum class Mode : std::uint8_t { Disabled, Autonomous, Driver };

/** Everything the pages draw, sampled once per SAMPLE_MS. */
struct Snapshot {
    double battery_pct = 0;
    double battery_volts = 0;
    double battery_current = 0;
    double battery_temp = 0;

    int controller_pct = 0;
    bool controller_linked = false;
    bool competition_linked = false;
    bool field_control = false;
    bool sd_installed = false;

    Mode mode = Mode::Disabled;
    std::uint32_t mode_ms = 0;  //< time since the current mode began

    Pose pose;
    bool pose_valid = false;
    bool pose_demo = false;  //< the position shown is synthetic, not measured

    std::vector<DeviceSample> devices;

    /* rolled-up health, so the dashboard doesn't re-scan every frame */
    double hottest_temp = 0;
    std::string hottest_label;
    int offline_count = 0;
    double drive_current = 0;
};

/* ==================================================================== logs */

enum class LogLevel : std::uint8_t { Info, Warn, Error };

struct LogLine {
    LogLevel level = LogLevel::Info;
    std::uint32_t ms = 0;
    std::string text;
};

/* ================================================================== state */

/** Guards everything below. Held briefly -- never call LVGL while holding it. */
pros::Mutex& data_lock();

std::vector<DeviceDef>& devices();
std::vector<AutonDef>& autons();
std::vector<LogLine>& logs();
Snapshot& snapshot();

/** Set by the log writers so the console only rebuilds when something changed. */
std::uint32_t log_revision();

std::function<Pose()>& pose_source();
std::function<void()>& reset_pose_action();
std::string& team_name();
bool demo_mode();

Alliance& selected_alliance();
int& selected_index();  //< index into autons(), or -1
/** Index of the selected routine restricted to the visible alliance tab. */
const AutonDef* selected_auton();

/* ======================================================== SD card storage */

/** Reads the saved alliance + routine name. Silent no-op without an SD card. */
void storage_load();
/** Persists the current selection. Called on every tap, debounced by content. */
void storage_save();
/** True once we have confirmed a card is present and writable. */
bool storage_available();

/* ============================================================== rendering */

/** One page in the nav rail. */
struct Page {
    const char* icon;   //< an LV_SYMBOL_* string
    const char* title;
    lv_obj_t* root = nullptr;
    void (*build)(lv_obj_t* parent) = nullptr;
    void (*update)(const Snapshot& s) = nullptr;
};

Page* pages();
int page_count();
int active_page();

/** Rebuilds the nav rail highlight and shows/hides page roots. */
void set_active_page(int index);

/** Re-tints accent-coloured chrome after the alliance changes. */
void refresh_alliance_tint();

/* -- page builders, one per file ---------------------------------------- */

void build_auton(lv_obj_t* parent);
void update_auton(const Snapshot& s);
void retint_auton();

void build_dash(lv_obj_t* parent);
void update_dash(const Snapshot& s);

void build_field(lv_obj_t* parent);
void update_field(const Snapshot& s);

void build_diag(lv_obj_t* parent);
void update_diag(const Snapshot& s);

void build_logs(lv_obj_t* parent);
void update_logs(const Snapshot& s);

/* ================================================== small widget helpers */

/** A flat container with no default styling, padding or scrollbars. */
lv_obj_t* blank(lv_obj_t* parent, std::int32_t w, std::int32_t h);

/** A rounded surface panel. */
lv_obj_t* card(lv_obj_t* parent, std::int32_t w, std::int32_t h);

/** A text label. Pass nullptr for `font` to inherit the parent's. */
lv_obj_t* text(lv_obj_t* parent, const char* str, const lv_font_t* font,
               std::uint32_t color);

/**
 * A titled statistic box: caption on top, big value beneath.
 * Returns the card; `out_value` receives the label you update each frame.
 */
lv_obj_t* stat_tile(lv_obj_t* parent, std::int32_t w, std::int32_t h,
                    const char* caption, lv_obj_t** out_value,
                    const lv_font_t* value_font = nullptr);

/** A thin horizontal meter, 0-100. */
lv_obj_t* meter(lv_obj_t* parent, std::int32_t w, std::int32_t h);

/** A small filled circle used as a status light. */
lv_obj_t* dot(lv_obj_t* parent, std::int32_t size, std::uint32_t color);

/** The accent colour for the current alliance -- red, blue, or team purple. */
std::uint32_t alliance_color();

/** Formats milliseconds as M:SS. */
void format_clock(char* out, std::size_t n, std::uint32_t ms);

}  // namespace internal
}  // namespace ui
