// Shared by the robot module's .cpp files. Use robot/robot.hpp from outside.
#pragma once

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "robot/config.hpp"
#include "robot/robot.hpp"

namespace robot::internal {

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::MotorGroup lift_mg;
extern pros::MotorGroup aux_mg;
extern pros::adi::DigitalOut claw_solenoid;
extern pros::Imu imu;
extern pros::Distance claw_distance;
// Global, not a local in opcontrol(): PROS kills that task on every mode change.
extern pros::Controller master;

extern bool imu_present;
extern bool distance_present;

void hardware_init();
void lift_init();
void claw_init();

inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace robot::internal
