/**
 * Brain screen UI: auton selector (remembered on SD), dashboard, field map,
 * diagnostics and log console. Register everything in initialize(), then call
 * start() last. All functions are safe to call from any task.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <initializer_list>

namespace ui {

enum class Alliance : std::uint8_t { Red = 0, Blue = 1, Skills = 2 };

/** LemLib convention: inches, degrees, 0 = +Y, clockwise. */
struct Pose {
    double x = 0;
    double y = 0;
    double theta = 0;
};

using AutonFn = std::function<void()>;

/* --- Auton selector -------------------------------------------------------- */
void add_auton(Alliance alliance, const char* name, const char* description, AutonFn run);
Alliance alliance();
/** Selected routine's name, or "None". Only valid once started() is true. */
const char* auton_name();
bool has_auton();
bool started();
/** Runs the selected routine. Call from autonomous(). */
void run_auton();
/** True while the AUTON page's TEST button is running a routine (in opcontrol). */
bool auton_test_running();

/* --- Telemetry (diagnostics page). read callbacks run ~20x/s; keep them cheap. */
void track_motor_group(const char* label, std::initializer_list<std::int8_t> ports);
void track_imu(const char* label, std::uint8_t port);
void track_distance(const char* label, std::uint8_t port);
void track_value(const char* label, const char* unit, std::function<double()> read);
void track_flag(const char* label, std::function<bool()> read);
void set_pose_source(std::function<Pose()> read);
/** Wires up the field page's Reset button. */
void on_reset_pose(std::function<void()> action);

/* --- Log console ----------------------------------------------------------- */
void log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void warn(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void error(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/** Builds the UI. Call once, last in initialize(). Don't use pros::lcd as well. */
void start();

}  // namespace ui
