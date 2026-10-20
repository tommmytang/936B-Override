/** UI state, the device sampler, and the shell (nav rail + status strip). */
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "internal.hpp"
#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"

namespace ui {
namespace internal {

using namespace theme;

/* ---- state ---------------------------------------------------------------- */

namespace {

std::vector<DeviceDef> g_devices;
std::vector<AutonDef> g_autons;
std::vector<LogLine> g_logs;
Snapshot g_snap;
std::uint32_t g_log_rev = 0;
std::uint32_t g_pose_rev = 0;  //< bumped by set_pose_source()

std::function<Pose()> g_pose_source;
std::function<void()> g_reset_action;
constexpr const char* TEAM = "936B";
bool g_started = false;

Alliance g_alliance = Alliance::Red;
int g_selected = -1;
int g_active_page = 0;

lv_obj_t* g_rail_btn[5] = {};
lv_obj_t* g_rail_icon[5] = {};
lv_obj_t* g_rail_text[5] = {};
lv_obj_t* g_rail_mark[5] = {};
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

std::vector<DeviceDef>& devices() { return g_devices; }
std::vector<AutonDef>& autons() { return g_autons; }
std::vector<LogLine>& logs() { return g_logs; }
Snapshot& snapshot() { return g_snap; }
std::uint32_t log_revision() { return g_log_rev; }
std::function<Pose()>& pose_source() { return g_pose_source; }
std::function<void()>& reset_pose_action() { return g_reset_action; }
Alliance& selected_alliance() { return g_alliance; }
int& selected_index() { return g_selected; }
Page* pages() { return g_pages; }
int page_count() { return 5; }
int active_page() { return g_active_page; }

const AutonDef* selected_auton() {
    if (g_selected < 0 || g_selected >= static_cast<int>(g_autons.size())) return nullptr;
    return &g_autons[g_selected];
}

/* ---- sampler: "ui-sample" task, never touches LVGL ------------------------ */

namespace {

DeviceSample read_device(const DeviceDef& d) {
    DeviceSample out;
    switch (d.kind) {
        case DeviceKind::MotorGroup: {
            for (std::int8_t port : d.ports) {
                pros::Motor m(port);
                if (!m.is_installed()) {
                    if (out.bad_port == 0) out.bad_port = static_cast<std::int8_t>(std::abs(port));
                    continue;
                }
                out.online = true;
                const double temp = m.get_temperature();
                if (std::isfinite(temp) && temp > out.primary) out.primary = temp;
                const std::int32_t amps = m.get_current_draw();
                if (amps != PROS_ERR) out.secondary += amps / 1000.0;
            }
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
        case DeviceKind::Distance: {
            pros::Distance dist(static_cast<std::uint8_t>(d.ports.front()));
            out.online = dist.is_installed();
            const std::int32_t mm = dist.get_distance();
            out.primary = (mm == PROS_ERR) ? 0 : mm;
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
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    Mode last_mode = Mode::Disabled;
    std::uint32_t mode_start = pros::millis();
    std::uint32_t next = pros::millis();

    // Registration is closed once start() runs, so this copy never goes stale.
    std::vector<DeviceDef> defs;
    {
        Guard guard;
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
        if (pros::competition::is_disabled()) mode = Mode::Disabled;
        else if (pros::competition::is_autonomous()) mode = Mode::Autonomous;
        if (mode != last_mode) {
            last_mode = mode;
            mode_start = pros::millis();
        }
        local.mode = mode;
        local.mode_ms = pros::millis() - mode_start;

        double hottest = 0;
        const char* hottest_name = "--";
        int offline = 0;
        double amps = 0;
        for (std::size_t i = 0; i < defs.size(); ++i) {
            samples[i] = read_device(defs[i]);
            if (defs[i].kind == DeviceKind::MotorGroup) {
                amps += samples[i].secondary;
                if (samples[i].primary > hottest) {
                    hottest = samples[i].primary;
                    hottest_name = defs[i].label.c_str();
                }
            }
            if (!samples[i].online || samples[i].bad_port != 0) ++offline;
        }
        local.devices = samples;
        local.hottest_temp = hottest;
        local.hottest_label = hottest_name;
        local.offline_count = offline;
        local.drive_current = amps;

        {
            Guard guard;
            if (pose_rev != g_pose_rev) {
                pose_rev = g_pose_rev;
                pose_fn = g_pose_source;
            }
        }
        local.pose_valid = static_cast<bool>(pose_fn);
        if (pose_fn) local.pose = pose_fn();

        {
            Guard guard;
            g_snap = local;
        }
        pros::Task::delay_until(&next, SAMPLE_MS);
    }
}

}  // namespace

/* ---- shell: nav rail and status strip (display daemon only) -------------- */

namespace {

void paint_rail_item(int i, bool on) {
    lv_obj_set_style_bg_opa(g_rail_btn[i], on ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_rail_btn[i], c(ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_rail_icon[i], c(on ? ACCENT_HI : TEXT_FAINT), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_rail_text[i], c(on ? TEXT : TEXT_FAINT), LV_PART_MAIN);
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

    lv_obj_t* badge = blank(rail, RAIL_W, TOPBAR_H);
    lv_obj_set_pos(badge, 0, 0);
    lv_obj_set_style_bg_color(badge, c(ACCENT), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_center(text(badge, TEAM, f_small(), TEXT));

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
    char buf[40];
    format_clock(clock, sizeof(clock), s.mode_ms);
    std::snprintf(buf, sizeof(buf), "%s  %s", name, clock);
    lv_label_set_text(g_mode_text, buf);
    lv_obj_set_style_bg_color(g_mode_pill, c(tint), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_mode_text, c(s.mode == Mode::Disabled ? TEXT_DIM : TEXT),
                                LV_PART_MAIN);
}

void render_cb(lv_timer_t*) {
    static Snapshot s;  //< reused so the copy doesn't re-allocate every frame
    {
        Guard guard;
        s = g_snap;
    }
    update_topbar(s);
    if (g_pages[g_active_page].update != nullptr) g_pages[g_active_page].update(s);
}

/** One-shot: builds the interface on the daemon, then hands over to render_cb. */
void build_cb(lv_timer_t*) {
    lv_obj_t* scr = lv_obj_create(nullptr);
    lv_screen_load(scr);
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
}

}  // namespace

void set_active_page(int index) {
    if (index < 0 || index >= page_count()) return;
    for (int i = 0; i < page_count(); ++i) {
        if (g_pages[i].root == nullptr) continue;
        if (i == index) lv_obj_remove_flag(g_pages[i].root, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_pages[i].root, LV_OBJ_FLAG_HIDDEN);
        paint_rail_item(i, i == index);
    }
    g_active_page = index;
    lv_label_set_text(g_title, g_pages[index].title);
}

void refresh_alliance_tint() { retint_auton(); }

}  // namespace internal

/* ---- public API ----------------------------------------------------------- */

using namespace internal;

namespace {
/** The lists are read unlocked once start() has run, so refuse late additions. */
bool registration_closed(const char* what) {
    if (!g_started) return false;
    ui::warn("Ignored %s registered after ui::start()", what);
    return true;
}

void add_device(DeviceDef d) {
    if (registration_closed(d.label.c_str())) return;
    Guard guard;
    g_devices.push_back(std::move(d));
}

void push_log(LogLevel level, const char* fmt, va_list args) {
    char buf[128];
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    LogLine line{level, pros::millis(), buf};
    Guard guard;
    g_logs.push_back(std::move(line));
    if (g_logs.size() > MAX_LOG_LINES) g_logs.erase(g_logs.begin());
    ++g_log_rev;
}
}  // namespace

void add_auton(Alliance a, const char* name, const char* description, AutonFn run) {
    if (registration_closed("auton")) return;
    Guard guard;
    g_autons.push_back({a, name ? name : "Unnamed", description ? description : "", std::move(run)});
    if (g_selected < 0 && a == g_alliance) g_selected = static_cast<int>(g_autons.size()) - 1;
}

Alliance alliance() { return g_alliance; }

const char* auton_name() {
    const AutonDef* a = selected_auton();
    return a != nullptr ? a->name.c_str() : "None";
}

bool started() { return g_started; }

bool has_auton() {
    const AutonDef* a = selected_auton();
    return a != nullptr && static_cast<bool>(a->run);
}

void run_auton() {
    AutonFn fn;
    std::string name;
    {
        Guard guard;
        if (const AutonDef* a = selected_auton()) {
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

void track_motor_group(const char* label, std::initializer_list<std::int8_t> ports) {
    DeviceDef d;
    d.kind = DeviceKind::MotorGroup;
    d.label = label;
    d.unit = "C";
    d.ports = ports;
    add_device(std::move(d));
}

void track_imu(const char* label, std::uint8_t port) {
    DeviceDef d;
    d.kind = DeviceKind::Imu;
    d.label = label;
    d.unit = "deg";
    d.ports = {static_cast<std::int8_t>(port)};
    add_device(std::move(d));
}

void track_distance(const char* label, std::uint8_t port) {
    DeviceDef d;
    d.kind = DeviceKind::Distance;
    d.label = label;
    d.unit = "mm";
    d.ports = {static_cast<std::int8_t>(port)};
    add_device(std::move(d));
}

void track_value(const char* label, const char* unit, std::function<double()> read) {
    DeviceDef d;
    d.kind = DeviceKind::Value;
    d.label = label;
    d.unit = unit ? unit : "";
    d.read_value = std::move(read);
    add_device(std::move(d));
}

void track_flag(const char* label, std::function<bool()> read) {
    DeviceDef d;
    d.kind = DeviceKind::Flag;
    d.label = label;
    d.read_flag = std::move(read);
    add_device(std::move(d));
}

void set_pose_source(std::function<Pose()> read) {
    Guard guard;
    g_pose_source = std::move(read);
    ++g_pose_rev;
}

void on_reset_pose(std::function<void()> action) {
    Guard guard;
    g_reset_action = std::move(action);
}

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

void start() {
    if (g_started) return;
    g_started = true;

    storage_load();
    ui::log("%s brain UI ready", TEAM);
    if (!storage_available()) ui::warn("No SD card - auton choice will not be remembered");

    pros::Task(sampler, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "ui-sample");

    // Everything else runs as lv_timers on the display daemon. Creating the
    // first one is the only LVGL call made from another task, so make it only
    // while the daemon is parked in its delay (Blocked) with the scheduler
    // suspended -- then it cannot be mid-lv_timer_handler().
    pros::task_t daemon = pros::c::task_get_by_name("Display Daemon (PROS)");
    // It runs at priority 3 and busy robot tasks starve it; safe to raise now
    // that nothing else touches LVGL.
    if (daemon != nullptr) pros::c::task_set_priority(daemon, TASK_PRIORITY_DEFAULT);
    while (true) {
        rtos_suspend_all();
        const bool parked = daemon == nullptr ||
                            pros::c::task_get_state(daemon) == pros::E_TASK_STATE_BLOCKED;
        if (parked) lv_timer_set_repeat_count(lv_timer_create(build_cb, 10, nullptr), 1);
        rtos_resume_all();
        if (parked) break;
        pros::delay(1);
    }
}

}  // namespace ui
