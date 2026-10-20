#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "internal.hpp"
#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {

// opcontrol restarts between periods; a >500 ms gap between ticks marks a new one.
std::atomic<std::uint32_t> g_period_start{0};
std::uint32_t g_last_step_ms = 0;
std::atomic<bool> g_r2_held{false};
std::atomic<bool> g_aux_on{false};
int g_aux_rpm = 0;
bool g_claw_was_closed = false;
std::uint32_t g_x_down_at = 0;
bool g_x_fired = false;

int aux_full_rpm() {
    switch (aux_mg.get_gearing()) {
        case pros::v5::MotorGears::red: return 100;
        case pros::v5::MotorGears::blue: return 600;
        default: return 200;
    }
}

std::uint32_t match_remaining_ms() {
    const std::uint32_t start = g_period_start.load();
    if (start == 0) return cfg::DRIVER_PERIOD_MS;
    const std::uint32_t elapsed = pros::millis() - start;
    return elapsed >= cfg::DRIVER_PERIOD_MS ? 0 : cfg::DRIVER_PERIOD_MS - elapsed;
}

/** True once, after the button has been held HOLD_TO_CONFIRM_MS. */
bool held_long_enough(bool pressed, std::uint32_t& down_at, bool& fired) {
    if (!pressed) {
        down_at = 0;
        fired = false;
        return false;
    }
    if (down_at == 0) down_at = pros::millis();
    if (!fired && pros::millis() - down_at >= cfg::HOLD_TO_CONFIRM_MS) {
        fired = true;
        return true;
    }
    return false;
}

struct Hottest {
    const char* name = "";
    double temp = -1.0;
};

Hottest hottest_motor() {
    Hottest h;
    auto check = [&h](const char* name, const pros::MotorGroup& mg) {
        for (const double t : mg.get_temperature_all()) {
            if (std::isfinite(t) && t > h.temp) h = {name, t};
        }
    };
    check("Drive", left_mg);
    check("Drive", right_mg);
    check("Lift", lift_mg);
    check("Aux", aux_mg);
    return h;
}

/** Writes one full 15-column row so no stale text is left behind. */
void put_line(std::uint8_t line, const char* fmt, ...) {
    char text[32];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    char row[16];
    std::snprintf(row, sizeof(row), "%-15.15s", text);
    master.set_text(line, 0, row);
}

/* Controller screen, one row per 100 ms (it drops faster writes):
 *   1:23   87% D52C     time left | brain battery | hottest motor
 *   GRIP auto AX on     claw | auto-clamp | aux
 *   Grabs 12 Lf 45      or HOT / CREEP / selected auton before driver */
void controller_screen_task(void*) {
    Hottest hot;
    int line = 0;
    while (true) {
        if (line == 0) {
            hot = hottest_motor();
            const unsigned long secs = match_remaining_ms() / 1000;
            const double battery = pros::battery::get_capacity();
            char batt[16];
            char temp[16];
            if (std::isfinite(battery)) std::snprintf(batt, sizeof(batt), "%3.0f%%", battery);
            else std::snprintf(batt, sizeof(batt), " --%%");
            if (hot.temp >= 0) std::snprintf(temp, sizeof(temp), "%c%2.0fC", hot.name[0], hot.temp);
            else std::snprintf(temp, sizeof(temp), " --C");
            put_line(0, "%lu:%02lu  %s %s", secs / 60, secs % 60, batt, temp);
        } else if (line == 1) {
            const char* clamp = !claw::armed() ? "man " : distance_present ? "auto" : "noDS";
            put_line(1, "%s %s AX %s", claw::closed() ? "GRIP" : "open", clamp,
                     g_aux_on ? "on" : "--");
        } else if (hot.temp >= cfg::HOT_MOTOR_C) {
            put_line(2, "HOT %s %2.0fC", hot.name, hot.temp);
        } else if (g_r2_held.load()) {
            put_line(2, "CREEP");
        } else if (g_period_start.load() == 0 && ui::started()) {
            put_line(2, "Auto %s", ui::auton_name());
        } else {
            put_line(2, "Grabs %2d Lf%3.0f", claw::grabs(), lift::height());
        }
        line = (line + 1) % 3;
        pros::delay(100);
    }
}

}  // namespace

void init() {
    hardware_init();
    lift_init();
    claw_init();

    g_aux_rpm = aux_full_rpm() * cfg::AUX_SPEED_PCT / 100;

    ui::track_motor_group("Drive L", {cfg::DRIVE_L1, cfg::DRIVE_L2, cfg::DRIVE_L3});
    ui::track_motor_group("Drive R", {cfg::DRIVE_R1, cfg::DRIVE_R2, cfg::DRIVE_R3});
    ui::track_motor_group("Lift", {cfg::LIFT_1, cfg::LIFT_2});
    ui::track_motor_group("Aux", {cfg::AUX_1, cfg::AUX_2});
    if (imu_present) ui::track_imu("IMU", cfg::IMU_PORT);
    if (distance_present) ui::track_distance("Claw", cfg::DISTANCE_PORT);

    ui::track_value("Lift", "deg", [] { return lift::height(); });
    ui::track_value("Claw gap", "mm", [] { return static_cast<double>(claw::distance_mm()); });
    ui::track_value("Claw size", "", [] {
        return distance_present ? static_cast<double>(claw_distance.get_object_size()) : -1.0;
    });
    ui::track_value("Closing", "mm/s", [] { return static_cast<double>(claw::closing_speed()); });
    ui::track_flag("Claw", [] { return claw::closed(); });
    ui::track_flag("Auto-clamp", [] { return claw::armed(); });
    ui::track_flag("Seated", [] { return claw::cone_seated(); });

    pros::Task(controller_screen_task, nullptr, "ctrl-screen");
}

void driver_step() {
    const std::uint32_t now = pros::millis();
    if (now - g_last_step_ms > 500) {
        g_period_start.store(now);
        g_aux_on = false;
        g_claw_was_closed = claw::closed();  // a grab from auton is not a new grab
        ui::log("Driver period started");
    }
    g_last_step_ms = now;

    const bool creep = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    g_r2_held.store(creep);
    drive::arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                  master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), creep);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) lift::manual(cfg::LIFT_MAX_POWER);
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) lift::manual(-cfg::LIFT_MAX_POWER);
    else lift::manual(0);

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        g_aux_on = !g_aux_on;
        ui::log("Aux %s", g_aux_on ? "on" : "off");
    }
    if (g_aux_on) aux_mg.move_velocity(g_aux_rpm);
    else aux_mg.move(0);  // coast down

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) claw::toggle();

    // Lift after grab: watch the claw, not A, so auto-clamp grabs count too.
    const bool claw_closed = claw::closed();
    if (claw_closed && !g_claw_was_closed && lift::height() <= cfg::GRAB_LIFT_BELOW_DEG) {
        lift::raise_by(cfg::GRAB_LIFT_RAISE_DEG, cfg::GRAB_LIFT_DELAY_MS);
        ui::log("Grab at lift %.0f -- raising %.0f", lift::height(), cfg::GRAB_LIFT_RAISE_DEG);
    }
    g_claw_was_closed = claw_closed;

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        claw::set_armed(!claw::armed());
        master.rumble(claw::armed() ? "." : "-");
    }

    if (held_long_enough(master.get_digital(pros::E_CONTROLLER_DIGITAL_X), g_x_down_at,
                         g_x_fired)) {
        lift::tare();
        master.rumble("..");
    }
}

}  // namespace robot
