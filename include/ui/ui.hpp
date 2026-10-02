/**
 * @file ui/ui.hpp
 *
 * The 936B brain screen interface.
 * ---------------------------------------------------------------------------
 * A touch UI for the V5 brain with five pages: an autonomous selector that
 * remembers your pick across reboots, a live match dashboard, an odometry
 * field map, a device diagnostics list, and an on-screen log console.
 *
 * QUICK START -- this is the whole integration:
 *
 *     #include "ui/ui.hpp"
 *
 *     void initialize() {
 *         ui::add_auton(ui::Alliance::Red,  "Rush",  "Grab mid, score 2", red_rush);
 *         ui::add_auton(ui::Alliance::Blue, "Safe",  "Score preload",     blue_safe);
 *         ui::add_auton(ui::Alliance::Skills, "Skills", "60s run",        skills);
 *
 *         ui::track_motor_group("Drive L", {1, -2, 3});
 *         ui::track_motor_group("Drive R", {-4, 5, -6});
 *         ui::track_imu("IMU", 10);
 *
 *         ui::start();          // <- always last
 *     }
 *
 *     void autonomous() { ui::run_auton(); }
 *
 * Every registration call is optional. Call `ui::start()` on its own and you
 * still get a working dashboard, diagnostics and log console.
 *
 * THREADING: all of these functions are safe to call from any task. The UI
 * itself is rendered by PROS's display daemon; nothing here blocks on it.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <initializer_list>
#include <string>
#include <vector>

namespace ui {

/* =========================================================================
 * Autonomous selector
 * ========================================================================= */

/** Which side of the field -- also used to tint the whole UI. */
enum class Alliance : std::uint8_t { Red = 0, Blue = 1, Skills = 2 };

/** A field position, in the same units your odometry reports. */
struct Pose {
    double x = 0;      //< inches, +X toward the right of the field
    double y = 0;      //< inches, +Y away from the driver station
    double theta = 0;  //< degrees, 0 = +Y, increasing clockwise (LemLib style)
};

using AutonFn = std::function<void()>;

/**
 * Registers a routine on the autonomous selector.
 *
 * @param alliance    which tab the routine appears under
 * @param name        short label, shown large (keep it under ~14 characters)
 * @param description one line of detail, shown beneath the name
 * @param run         the routine itself; called by `run_auton()`
 *
 * Call this as many times as you like, before `start()`. The first routine
 * registered for an alliance is selected by default, unless a saved choice
 * from a previous run is found on the SD card.
 */
void add_auton(Alliance alliance, const char* name, const char* description, AutonFn run);

/** Registers a routine with no description. */
void add_auton(Alliance alliance, const char* name, AutonFn run);

/** The alliance currently selected on screen. */
Alliance alliance();

/** Name of the selected routine, or "None" if nothing is registered. */
const char* auton_name();

/** True if a routine is selected and ready to run. */
bool has_auton();

/**
 * Runs the selected routine. Put this in `autonomous()` and never touch it
 * again -- switching routines is done on the brain, not in code.
 */
void run_auton();

/** Selects a routine from code (useful for testing from `opcontrol`). */
void select_auton(Alliance alliance, const char* name);

/* =========================================================================
 * Telemetry -- what the dashboard and diagnostics pages display
 * ========================================================================= */

/**
 * Watches one motor. Appears on the diagnostics page with live temperature,
 * current draw and a plugged-in check.
 *
 * @param label human name, e.g. "Intake"
 * @param port  1-21; use a negative port for a reversed motor, exactly as you
 *              would when constructing a `pros::Motor`
 */
void track_motor(const char* label, std::int8_t port);

/**
 * Watches a group of motors as a single row. The row reports the hottest
 * motor in the group and flags any port that has dropped offline.
 */
void track_motor_group(const char* label, std::initializer_list<std::int8_t> ports);

/** Watches an IMU. Feeds the heading readout on the dashboard. */
void track_imu(const char* label, std::uint8_t port);

/** Watches a rotation sensor. */
void track_rotation(const char* label, std::int8_t port);

/** Watches a distance sensor. */
void track_distance(const char* label, std::uint8_t port);

/** Watches an optical sensor. */
void track_optical(const char* label, std::uint8_t port);

/**
 * Watches any number you can compute. Shows up on the diagnostics page.
 *
 *     ui::track_value("Lift", "deg", [] { return lift.get_position(); });
 *
 * @param read called roughly 20x a second from a background task -- keep it
 *             cheap and never block inside it
 */
void track_value(const char* label, const char* unit, std::function<double()> read);

/**
 * Watches a yes/no condition, drawn as a green or red dot.
 *
 *     ui::track_flag("Clamp", [] { return clamp.get_value(); });
 */
void track_flag(const char* label, std::function<bool()> read);

/**
 * Feeds the field map and the dashboard's position readout.
 *
 * With LemLib:
 *
 *     ui::set_pose_source([] {
 *         lemlib::Pose p = chassis.getPose();
 *         return ui::Pose{p.x, p.y, p.theta};
 *     });
 */
void set_pose_source(std::function<Pose()> read);

/** Wires up the field map's "Reset" button, e.g. to zero your odometry. */
void on_reset_pose(std::function<void()> action);

/* =========================================================================
 * Log console
 * ========================================================================= */

void log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void warn(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void error(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/* =========================================================================
 * Lifecycle and appearance
 * ========================================================================= */

/** Team name shown in the status strip. Defaults to "936B". */
void set_team(const char* name);

/**
 * When nothing has been registered, the UI animates plausible fake numbers so
 * you can see the layout without a robot attached. It turns itself off as soon
 * as you register anything real; call this to force it either way.
 */
void set_demo_mode(bool enabled);

/**
 * Builds the interface and starts updating it. Call once, at the end of
 * `initialize()`, after all your registration calls.
 *
 * Do NOT also call `pros::lcd::initialize()` -- the two fight over the screen.
 */
void start();

/** Opens a page from code. 0=Auton 1=Dash 2=Field 3=Diag 4=Log. */
void show_page(int index);

}  // namespace ui
