/**
 * @file src/robot/internal.hpp
 *
 * Shared between the robot/*.cpp files. Not part of the public API --
 * include robot/robot.hpp instead.
 */
#pragma once

#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "robot/config.hpp"
#include "robot/robot.hpp"

namespace robot::internal {

/* The devices themselves. Defined in hardware.cpp. */
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::MotorGroup lift_mg;
extern pros::adi::DigitalOut claw_solenoid;
extern pros::Imu imu;
extern pros::Distance claw_distance;
extern pros::Rotation lift_rotation;

/* Probed once during init(), so the control loops never pay for a failed
 * device read every tick. */
extern bool imu_present;
extern bool distance_present;
extern bool rotation_present;

void hardware_init();
void lift_init();
void claw_init();
void drive_init();
void stack_init();

/* The active controller, for rumble feedback. Null until opcontrol runs, so
 * every use must null-check. Re-pointed each tick by driver_step(). */
extern pros::Controller* g_controller;

/* Preset persistence -- /usd/936b_lift.cfg. Silent no-op without a card. */
void presets_load(double* out, int count);
void presets_save(const double* in, int count);

/** Clamps v into [lo, hi]. */
inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline int clampi(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/** Moves `current` toward `target` by at most `max_step`. */
inline int slew(int current, int target, int max_step) {
    const int delta = target - current;
    if (delta > max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

}  // namespace robot::internal
