/**
 * @file src/ui/core.cpp
 *
 * State, sampling and the application shell (nav rail + status strip).
 * See internal.hpp for the threading model; it is the thing to understand
 * before changing anything in this file.
 */
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "internal.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"

namespace ui {
namespace internal {

using namespace theme;

/* ========================================================================
 * State
 * ==================================================================== */

namespace {

pros::Mutex g_lock;
std::vector<DeviceDef> g_devices;
std::vector<AutonDef> g_autons;
std::vector<LogLine> g_logs;
Snapshot g_snap;
std::uint32_t g_log_rev = 0;
std::uint32_t g_pose_rev = 0;  //< bumped when set_pose_source() is called

std::function<Pose()> g_pose_source;
std::function<void()> g_reset_action;
std::string g_team = "936B";

bool g_demo = false;
bool g_demo_pinned = false;  //< true once set_demo_mode() was called explicitly
bool g_started = false;

Alliance g_alliance = Alliance::Red;
int g_selected = -1;
int g_active_page = 0;

/* chrome widgets, all owned by the display daemon */
lv_obj_t* g_rail_btn[5] = {};
lv_obj_t* g_rail_icon[5] = {};
lv_obj_t* g_rail_text[5] = {};
lv_obj_t* g_rail_mark[5] = {};
lv_obj_t* g_brand = nullptr;
lv_obj_t* g_title = nullptr;
lv_obj_t* g_mode_pill = nullptr;
lv_obj_t* g_mode_text = nullptr;
lv_obj_t* g_batt_text = nullptr;
lv_obj_t* g_link_dot = nullptr;
lv_obj_t* g_sd_text = nullptr;

Page g_pages[5] = {
    {LV_SYMBOL_PLAY, "AUTON", nullptr, build_auton, update_auton},
    {LV_SYMBOL_HOME, "DASH", nullptr, build_dash, update_dash},
    {LV_SYMBOL_GPS, "FIELD", nullptr, build_field, update_field},
    {LV_SYMBOL_SETTINGS, "DIAG", nullptr, build_diag, update_diag},
    {LV_SYMBOL_LIST, "LOG", nullptr, build_logs, update_logs},
};

}  // namespace

pros::Mutex& data_lock() { return g_lock; }
std::vector<DeviceDef>& devices() { return g_devices; }
std::vector<AutonDef>& autons() { return g_autons; }
std::vector<LogLine>& logs() { return g_logs; }
Snapshot& snapshot() { return g_snap; }
std::uint32_t log_revision() { return g_log_rev; }
std::function<Pose()>& pose_source() { return g_pose_source; }
std::function<void()>& reset_pose_action() { return g_reset_action; }
std::string& team_name() { return g_team; }
bool demo_mode() { return g_demo; }
Alliance& selected_alliance() { return g_alliance; }
int& selected_index() { return g_selected; }
Page* pages() { return g_pages; }
int page_count() { return 5; }
int active_page() { return g_active_page; }

const AutonDef* selected_auton() {
    if (g_selected < 0 || g_selected >= static_cast<int>(g_autons.size())) return nullptr;
    return &g_autons[g_selected];
}

/* ========================================================================
 * Sampling -- runs on the "ui-sample" task, never touches LVGL
 * ==================================================================== */

namespace {

/**
 * Fills in the readings nothing is wired up to yet, so the layout is visible
 * on a bare brain. Battery, controller and competition state are always real
 * -- a dashboard that invents a battery percentage is worse than no dashboard.
 */
void fill_demo(Snapshot& s) {
    const double t = pros::millis() / 1000.0;
    s.pose.x = 36 * std::sin(t / 6.0);
    s.pose.y = 30 * std::sin(t / 4.0);
    s.pose.theta = std::fmod(t * 24.0, 360.0);
    s.pose_valid = true;
    s.pose_demo = true;
    s.drive_current = 4.2 + 2.0 * std::sin(t / 3.0);
    s.hottest_temp = 42 + 9 * std::sin(t / 11.0);
    s.hottest_label = "demo";
}

/** Reads one registered device. Returns a fully populated sample. */
DeviceSample read_device(const DeviceDef& d) {
    DeviceSample out;
    switch (d.kind) {
        case DeviceKind::Motor:
        case DeviceKind::MotorGroup: {
            double hottest = 0;
            double current = 0;
            bool any_online = false;
            for (std::int8_t port : d.ports) {
                pros::Motor m(port);
                if (!m.is_installed()) {
                    if (out.bad_port == 0) out.bad_port = static_cast<std::int8_t>(std::abs(port));
                    continue;
                }
                any_online = true;
                const double temp = m.get_temperature();
                if (std::isfinite(temp) && temp > hottest) hottest = temp;
                const std::int32_t amps = m.get_current_draw();
                if (amps != PROS_ERR) current += amps / 1000.0;
            }
            out.online = any_online;
            out.primary = hottest;
            out.secondary = current;
            break;
        }
        case DeviceKind::Imu: {
            pros::Imu imu(static_cast<std::uint8_t>(d.ports.front()));
            out.online = imu.is_installed();
            const double h = imu.get_heading();
            out.primary = std::isfinite(h) ? h : 0;
            out.flag = out.online && imu.is_calibrating();
            break;
        }
        case DeviceKind::Rotation: {
            pros::Rotation r(d.ports.front());
            out.online = r.is_installed();
            const std::int32_t a = r.get_angle();
            out.primary = (a == PROS_ERR) ? 0 : a / 100.0;
            break;
        }
        case DeviceKind::Distance: {
            pros::Distance dist(static_cast<std::uint8_t>(d.ports.front()));
            out.online = dist.is_installed();
            const std::int32_t mm = dist.get_distance();
            out.primary = (mm == PROS_ERR) ? 0 : mm;
            break;
        }
        case DeviceKind::Optical: {
            pros::Optical o(static_cast<std::uint8_t>(d.ports.front()));
            out.online = o.is_installed();
            const double hue = o.get_hue();
            out.primary = std::isfinite(hue) ? hue : 0;
            break;
        }
        case DeviceKind::Value:
            out.online = true;
            if (d.read_value) out.primary = d.read_value();
            break;
        case DeviceKind::Flag:
            out.online = true;
            if (d.read_flag) out.flag = d.read_flag();
            break;
    }
    return out;
}

void sampler(void*) {
    static pros::Controller master(pros::E_CONTROLLER_MASTER);

    Mode last_mode = Mode::Disabled;
    std::uint32_t mode_start = pros::millis();
    std::uint32_t next = pros::millis();

    /* Device definitions are frozen once start() runs, so this local copy of
     * the list never goes stale and lets us read devices without the lock. */
    std::vector<DeviceDef> defs;
    {
        std::lock_guard<pros::Mutex> guard(g_lock);
        defs = g_devices;
    }

    std::vector<DeviceSample> samples(defs.size());
    std::function<Pose()> pose_fn;
    std::uint32_t pose_rev = 0xFFFFFFFFu;
    Snapshot local;

    while (true) {
        local.battery_pct = pros::battery::get_capacity();
        local.battery_volts = pros::battery::get_voltage() / 1000.0;
        local.battery_current = pros::battery::get_current() / 1000.0;
        local.battery_temp = pros::battery::get_temperature();
        local.controller_pct = master.get_battery_capacity();
        local.controller_linked = master.is_connected() == 1;
        local.competition_linked = pros::competition::is_connected() != 0;
        local.field_control = pros::competition::is_field_control() != 0;
        local.sd_installed = pros::usd::is_installed() != 0;

        Mode mode = Mode::Driver;
        if (pros::competition::is_disabled()) {
            mode = Mode::Disabled;
        } else if (pros::competition::is_autonomous()) {
            mode = Mode::Autonomous;
        }
        if (mode != last_mode) {
            last_mode = mode;
            mode_start = pros::millis();
        }
        local.mode = mode;
        local.mode_ms = pros::millis() - mode_start;

        double hottest = 0;
        const char* hottest_name = "--";
        int offline = 0;
        double drive_amps = 0;

        for (std::size_t i = 0; i < defs.size(); ++i) {
            samples[i] = read_device(defs[i]);
            const DeviceDef& d = defs[i];
            const bool is_motor =
                d.kind == DeviceKind::Motor || d.kind == DeviceKind::MotorGroup;
            if (is_motor) {
                drive_amps += samples[i].secondary;
                if (samples[i].primary > hottest) {
                    hottest = samples[i].primary;
                    hottest_name = d.label.c_str();
                }
            }
            if (!samples[i].online || samples[i].bad_port != 0) ++offline;
        }

        local.devices = samples;
        local.hottest_temp = hottest;
        local.hottest_label = hottest_name;
        local.offline_count = offline;
        local.drive_current = drive_amps;

        {
            std::lock_guard<pros::Mutex> guard(g_lock);
            if (pose_rev != g_pose_rev) {
                pose_rev = g_pose_rev;
                pose_fn = g_pose_source;
            }
        }
        local.pose_demo = false;
        if (pose_fn) {
            local.pose = pose_fn();
            local.pose_valid = true;
        } else {
            local.pose_valid = false;
        }

        if (g_demo) fill_demo(local);

        {
            std::lock_guard<pros::Mutex> guard(g_lock);
            g_snap = local;
        }

        next += SAMPLE_MS;
        pros::Task::delay_until(&next, SAMPLE_MS);
    }
}

}  // namespace

/* ========================================================================
 * Shell -- nav rail and status strip
 * ==================================================================== */

namespace {

void paint_rail_item(int i, bool on) {
    const std::uint32_t fg = on ? TEXT : TEXT_FAINT;
    lv_obj_set_style_bg_opa(g_rail_btn[i], on ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_rail_btn[i], c(ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_rail_icon[i], c(on ? ACCENT_HI : TEXT_FAINT), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_rail_text[i], c(fg), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_rail_mark[i], on ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
}

void on_rail_click(lv_event_t* e) {
    set_active_page(static_cast<int>(reinterpret_cast<std::intptr_t>(lv_event_get_user_data(e))));
}

void build_rail(lv_obj_t* scr) {
    lv_obj_t* rail = blank(scr, RAIL_W, SCREEN_H);
    lv_obj_set_pos(rail, 0, 0);
    lv_obj_set_style_bg_color(rail, c(SURFACE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(rail, LV_OPA_COVER, LV_PART_MAIN);

    /* team badge, aligned with the status strip next to it */
    lv_obj_t* badge = blank(rail, RAIL_W, TOPBAR_H);
    lv_obj_set_pos(badge, 0, 0);
    lv_obj_set_style_bg_color(badge, c(ACCENT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, LV_PART_MAIN);
    g_brand = text(badge, g_team.c_str(), f_small(), TEXT);
    lv_obj_center(g_brand);

    const std::int32_t item_h = 42;
    for (int i = 0; i < page_count(); ++i) {
        lv_obj_t* b = blank(rail, RAIL_W, item_h);
        lv_obj_set_pos(b, 0, TOPBAR_H + 2 + i * item_h);
        lv_obj_set_style_radius(b, 0, LV_PART_MAIN);
        lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b, on_rail_click, LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(static_cast<std::intptr_t>(i)));

        lv_obj_t* mark = blank(b, 3, item_h - 10);
        lv_obj_align(mark, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_set_style_bg_color(mark, c(ACCENT_HI), LV_PART_MAIN);
        lv_obj_set_style_radius(mark, 2, LV_PART_MAIN);

        lv_obj_t* icon = text(b, g_pages[i].icon, f_title(), TEXT_FAINT);
        lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 4);

        lv_obj_t* label = text(b, g_pages[i].title, f_micro(), TEXT_FAINT);
        lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_obj_set_style_text_letter_space(label, 1, LV_PART_MAIN);

        g_rail_btn[i] = b;
        g_rail_icon[i] = icon;
        g_rail_text[i] = label;
        g_rail_mark[i] = mark;
        paint_rail_item(i, i == 0);
    }
}

void build_topbar(lv_obj_t* scr) {
    lv_obj_t* bar = blank(scr, CONTENT_W, TOPBAR_H);
    lv_obj_set_pos(bar, CONTENT_X, 0);
    lv_obj_set_style_bg_color(bar, c(SURFACE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);

    g_title = text(bar, g_pages[0].title, f_small(), TEXT_DIM);
    lv_obj_align(g_title, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_set_style_text_letter_space(g_title, 2, LV_PART_MAIN);

    /* right-hand cluster, laid out from the right edge inward */
    g_batt_text = text(bar, "--%", f_small(), TEXT);
    lv_obj_align(g_batt_text, LV_ALIGN_RIGHT_MID, -8, 0);

    g_link_dot = dot(bar, 7, TEXT_FAINT);
    lv_obj_align(g_link_dot, LV_ALIGN_RIGHT_MID, -52, 0);

    g_sd_text = text(bar, LV_SYMBOL_SD_CARD, f_small(), TEXT_FAINT);
    lv_obj_align(g_sd_text, LV_ALIGN_RIGHT_MID, -68, 0);

    g_mode_pill = blank(bar, 96, 17);
    lv_obj_align(g_mode_pill, LV_ALIGN_RIGHT_MID, -90, 0);
    lv_obj_set_style_bg_opa(g_mode_pill, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_mode_pill, c(ACCENT_DEEP), LV_PART_MAIN);
    lv_obj_set_style_radius(g_mode_pill, 8, LV_PART_MAIN);
    g_mode_text = text(g_mode_pill, "DISABLED", f_micro(), TEXT_DIM);
    lv_obj_center(g_mode_text);
}

void update_topbar(const Snapshot& s) {
    char buf[40];

    lv_label_set_text_fmt(g_batt_text, "%d%%", static_cast<int>(s.battery_pct + 0.5));
    lv_obj_set_style_text_color(
        g_batt_text, c(heat_color(100 - s.battery_pct, 50.0, 75.0)), LV_PART_MAIN);
    lv_obj_align(g_batt_text, LV_ALIGN_RIGHT_MID, -8, 0);

    lv_obj_set_style_bg_color(g_link_dot, c(s.controller_linked ? OK : DANGER), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_sd_text, c(s.sd_installed ? TEXT_DIM : TEXT_FAINT),
                                LV_PART_MAIN);

    const char* name = "DRIVER";
    std::uint32_t tint = ACCENT;
    if (s.mode == Mode::Disabled) {
        name = "DISABLED";
        tint = ACCENT_DEEP;
    } else if (s.mode == Mode::Autonomous) {
        name = "AUTON";
        tint = alliance_color();
    }
    char clock[12];
    format_clock(clock, sizeof(clock), s.mode_ms);
    std::snprintf(buf, sizeof(buf), "%s  %s", name, clock);
    lv_label_set_text(g_mode_text, buf);
    lv_obj_set_style_bg_color(g_mode_pill, c(tint), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_mode_text, c(s.mode == Mode::Disabled ? TEXT_DIM : TEXT),
                                LV_PART_MAIN);
}

/** Runs inside PROS's display daemon -- the only place LVGL is touched. */
void render_cb(lv_timer_t*) {
    static Snapshot s;  //< reused so the per-frame copy does not re-allocate
    {
        std::lock_guard<pros::Mutex> guard(g_lock);
        s = g_snap;
    }
    update_topbar(s);
    if (g_pages[g_active_page].update != nullptr) g_pages[g_active_page].update(s);
}

/** One-shot: constructs the whole interface, then hands over to render_cb. */
void build_cb(lv_timer_t* t) {
    lv_obj_t* scr = lv_screen_active();
    lv_obj_remove_style_all(scr);
    lv_obj_set_style_bg_color(scr, c(BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    build_rail(scr);
    build_topbar(scr);

    for (int i = 0; i < page_count(); ++i) {
        lv_obj_t* root = blank(scr, CONTENT_W, CONTENT_H);
        lv_obj_set_pos(root, CONTENT_X, CONTENT_Y);
        lv_obj_set_style_pad_all(root, 6, LV_PART_MAIN);
        g_pages[i].root = root;
        if (g_pages[i].build != nullptr) g_pages[i].build(root);
        if (i != 0) lv_obj_add_flag(root, LV_OBJ_FLAG_HIDDEN);
    }

    lv_timer_create(render_cb, RENDER_MS, nullptr);
    lv_timer_delete(t);
}

}  // namespace

void set_active_page(int index) {
    if (index < 0 || index >= page_count()) return;
    for (int i = 0; i < page_count(); ++i) {
        if (g_pages[i].root == nullptr) continue;
        if (i == index) {
            lv_obj_remove_flag(g_pages[i].root, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(g_pages[i].root, LV_OBJ_FLAG_HIDDEN);
        }
        paint_rail_item(i, i == index);
    }
    g_active_page = index;
    lv_label_set_text(g_title, g_pages[index].title);
}

void refresh_alliance_tint() { retint_auton(); }

}  // namespace internal

/* ========================================================================
 * Public API
 * ==================================================================== */

using namespace internal;

namespace {
/** Registration implies the user has real hardware, so demo data steps aside. */
void saw_real_data() {
    if (!g_demo_pinned) g_demo = false;
}

/**
 * The device and routine lists are read without the lock by the sampler and by
 * the pages, which is only sound because nothing appends to them once start()
 * has run. Late calls are refused rather than raced on.
 */
bool registration_closed(const char* what) {
    if (!g_started) return false;
    ui::warn("Ignored %s registered after ui::start()", what);
    return true;
}
}  // namespace

void add_auton(Alliance a, const char* name, const char* description, AutonFn run) {
    if (registration_closed("auton")) return;
    std::lock_guard<pros::Mutex> guard(data_lock());
    autons().push_back({a, name ? name : "Unnamed", description ? description : "", std::move(run)});
    if (selected_index() < 0 && a == selected_alliance()) {
        selected_index() = static_cast<int>(autons().size()) - 1;
    }
}

void add_auton(Alliance a, const char* name, AutonFn run) {
    add_auton(a, name, "", std::move(run));
}

Alliance alliance() { return selected_alliance(); }

const char* auton_name() {
    const AutonDef* a = selected_auton();
    return a != nullptr ? a->name.c_str() : "None";
}

bool has_auton() {
    const AutonDef* a = selected_auton();
    return a != nullptr && static_cast<bool>(a->run);
}

void run_auton() {
    AutonFn fn;
    std::string name;
    {
        std::lock_guard<pros::Mutex> guard(data_lock());
        const AutonDef* a = selected_auton();
        if (a != nullptr) {
            fn = a->run;
            name = a->name;
        }
    }
    if (!fn) {
        ui::error("autonomous(): no routine selected");
        return;
    }
    ui::log("Running \"%s\"", name.c_str());
    fn();
    ui::log("\"%s\" finished", name.c_str());
}

void select_auton(Alliance a, const char* name) {
    std::lock_guard<pros::Mutex> guard(data_lock());
    selected_alliance() = a;
    for (std::size_t i = 0; i < autons().size(); ++i) {
        if (autons()[i].alliance == a && autons()[i].name == name) {
            selected_index() = static_cast<int>(i);
            return;
        }
    }
}

void track_motor(const char* label, std::int8_t port) {
    if (registration_closed("motor")) return;
    saw_real_data();
    std::lock_guard<pros::Mutex> guard(data_lock());
    DeviceDef d;
    d.kind = DeviceKind::Motor;
    d.label = label;
    d.unit = "C";
    d.ports = {port};
    devices().push_back(std::move(d));
}

void track_motor_group(const char* label, std::initializer_list<std::int8_t> ports) {
    if (registration_closed("motor group")) return;
    saw_real_data();
    std::lock_guard<pros::Mutex> guard(data_lock());
    DeviceDef d;
    d.kind = DeviceKind::MotorGroup;
    d.label = label;
    d.unit = "C";
    d.ports = ports;
    devices().push_back(std::move(d));
}

namespace {
void add_sensor(DeviceKind kind, const char* label, const char* unit, std::int8_t port) {
    if (registration_closed("sensor")) return;
    saw_real_data();
    std::lock_guard<pros::Mutex> guard(data_lock());
    DeviceDef d;
    d.kind = kind;
    d.label = label;
    d.unit = unit;
    d.ports = {port};
    devices().push_back(std::move(d));
}
}  // namespace

void track_imu(const char* label, std::uint8_t port) {
    add_sensor(DeviceKind::Imu, label, "deg", static_cast<std::int8_t>(port));
}
void track_rotation(const char* label, std::int8_t port) {
    add_sensor(DeviceKind::Rotation, label, "deg", port);
}
void track_distance(const char* label, std::uint8_t port) {
    add_sensor(DeviceKind::Distance, label, "mm", static_cast<std::int8_t>(port));
}
void track_optical(const char* label, std::uint8_t port) {
    add_sensor(DeviceKind::Optical, label, "hue", static_cast<std::int8_t>(port));
}

void track_value(const char* label, const char* unit, std::function<double()> read) {
    if (registration_closed("value")) return;
    saw_real_data();
    std::lock_guard<pros::Mutex> guard(data_lock());
    DeviceDef d;
    d.kind = DeviceKind::Value;
    d.label = label;
    d.unit = unit ? unit : "";
    d.read_value = std::move(read);
    devices().push_back(std::move(d));
}

void track_flag(const char* label, std::function<bool()> read) {
    if (registration_closed("flag")) return;
    saw_real_data();
    std::lock_guard<pros::Mutex> guard(data_lock());
    DeviceDef d;
    d.kind = DeviceKind::Flag;
    d.label = label;
    d.read_flag = std::move(read);
    devices().push_back(std::move(d));
}

void set_pose_source(std::function<Pose()> read) {
    saw_real_data();
    std::lock_guard<pros::Mutex> guard(data_lock());
    pose_source() = std::move(read);
    ++g_pose_rev;
}

void on_reset_pose(std::function<void()> action) {
    std::lock_guard<pros::Mutex> guard(data_lock());
    reset_pose_action() = std::move(action);
}

namespace {
void push_log(LogLevel level, const char* fmt, va_list args) {
    char buf[128];
    std::vsnprintf(buf, sizeof(buf), fmt, args);

    std::lock_guard<pros::Mutex> guard(data_lock());
    logs().push_back({level, pros::millis(), buf});
    if (logs().size() > MAX_LOG_LINES) logs().erase(logs().begin());
    ++g_log_rev;
}
}  // namespace

void log(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    push_log(LogLevel::Info, fmt, args);
    va_end(args);
}

void warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    push_log(LogLevel::Warn, fmt, args);
    va_end(args);
}

void error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    push_log(LogLevel::Error, fmt, args);
    va_end(args);
}

void set_team(const char* name) {
    std::lock_guard<pros::Mutex> guard(data_lock());
    team_name() = name ? name : "";
}

void set_demo_mode(bool enabled) {
    g_demo = enabled;
    g_demo_pinned = true;
}

void show_page(int index) { set_active_page(index); }

void start() {
    if (g_started) return;
    g_started = true;

    /* With nothing registered there is nothing to show, so animate stand-ins. */
    if (!g_demo_pinned && devices().empty() && !pose_source()) g_demo = true;

    storage_load();
    ui::log("%s brain UI ready", team_name().c_str());
    if (!storage_available()) {
        ui::warn("No SD card - auton choice will not be remembered");
    }
    if (g_demo) {
        ui::warn("Demo data on - nothing tracked yet");
    }

    pros::Task(sampler, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "ui-sample");

    /* Build on the display daemon rather than here, so that every LVGL call in
     * this program happens on one thread. lv_timer callbacks run there. */
    lv_timer_t* boot = lv_timer_create(build_cb, 10, nullptr);
    lv_timer_set_repeat_count(boot, 1);
}

}  // namespace ui
