/**
 * @file src/robot/hardware.cpp
 *
 * The device objects, and the one-time probe that decides which optional
 * sensors are actually plugged in.
 *
 * A missing sensor must never stop the robot: if the IMU is unplugged you
 * lose heading hold and keep the drive, if the Distance sensor is unplugged
 * you lose auto-clamp and keep the manual claw. Nothing here throws and
 * nothing here blocks on a device that is not there.
 */
#include "internal.hpp"

#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot::internal {

namespace cfg = robot::config;

pros::MotorGroup left_mg({cfg::DRIVE_L1, cfg::DRIVE_L2, cfg::DRIVE_L3});
pros::MotorGroup right_mg({cfg::DRIVE_R1, cfg::DRIVE_R2, cfg::DRIVE_R3});
pros::MotorGroup lift_mg({cfg::LIFT_1, cfg::LIFT_2});
pros::MotorGroup aux_mg({cfg::AUX_1, cfg::AUX_2});

pros::adi::DigitalOut claw_solenoid(cfg::CLAW_ADI_PORT, false);

pros::Imu imu(cfg::IMU_PORT);
pros::Distance claw_distance(cfg::DISTANCE_PORT);

bool imu_present = false;
bool distance_present = false;

pros::Controller* g_controller = nullptr;

}  // namespace robot::internal

/* The LemLib chassis. Defined here, after the motors and IMU, so the devices
 * it points at are guaranteed to be constructed first. */
namespace robot {

namespace {
namespace cfg = robot::config;
using namespace robot::internal;

lemlib::Drivetrain drivetrain(&left_mg, &right_mg, cfg::CHASSIS_TRACK_WIDTH,
                              cfg::CHASSIS_WHEEL_DIAMETER, cfg::CHASSIS_WHEEL_RPM,
                              cfg::CHASSIS_HORIZONTAL_DRIFT);

// No tracking wheels: LemLib falls back to the drive motor encoders.
lemlib::OdomSensors odom_sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// kP, kI, kD, windupRange, smallError, smallErrorTimeout, largeError,
// largeErrorTimeout, slew. The last six are LemLib's defaults: leave them.
lemlib::ControllerSettings linear(cfg::LINEAR_KP, cfg::LINEAR_KI, cfg::LINEAR_KD,
                                  3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular(cfg::ANGULAR_KP, cfg::ANGULAR_KI, cfg::ANGULAR_KD,
                                   3, 1, 100, 3, 500, 0);
}  // namespace

lemlib::Chassis chassis(drivetrain, linear, angular, odom_sensors);

}  // namespace robot

namespace robot::internal {

void hardware_init() {
    // Hold position instead of coasting, so the mechanism stays where the
    // driver leaves it rather than sagging under its own weight.
    // set_brake_mode_all, not set_brake_mode -- the latter defaults to index 0
    // and would set the first port only, leaving the second coasting.
    lift_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    // Coast the drive. Brake-holding the drive makes creep mode fight itself:
    // every time the stick crosses the deadband the wheels lock, which is the
    // opposite of the fine control creep mode exists to provide.
    left_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // Coast the aux pair so it spins down freely when toggled off.
    aux_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    distance_present = claw_distance.is_installed();
    imu_present = imu.is_installed();

    if (distance_present) {
        ui::log("Auto-clamp: distance sensor on port %d", cfg::DISTANCE_PORT);
    } else {
        ui::warn("No distance sensor on %d -- auto-clamp off", cfg::DISTANCE_PORT);
    }

    if (imu_present) {
        ui::log("IMU calibrating on port %d", cfg::IMU_PORT);
    } else {
        ui::warn("No IMU on %d -- heading hold off, auton turns will drift",
                 cfg::IMU_PORT);
    }

    // Calibrates the IMU, zeroes the drive encoders and starts LemLib's
    // odometry task. BLOCKS for ~2-3 s while the IMU calibrates, so keep the
    // robot still at power-on. With no IMU it gives up quickly and tracks
    // heading from the wheels instead.
    chassis.calibrate();
}

}  // namespace robot::internal
