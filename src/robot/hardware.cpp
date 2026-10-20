#include "internal.hpp"
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
pros::Controller master(pros::E_CONTROLLER_MASTER);

bool imu_present = false;
bool distance_present = false;

}  // namespace robot::internal

// Same file as the devices, so they are constructed before the chassis.
namespace robot {
namespace {
namespace cfg = robot::config;
using namespace robot::internal;

lemlib::Drivetrain drivetrain(&left_mg, &right_mg, cfg::CHASSIS_TRACK_WIDTH,
                              cfg::CHASSIS_WHEEL_DIAMETER, cfg::CHASSIS_WHEEL_RPM,
                              cfg::CHASSIS_HORIZONTAL_DRIFT);
lemlib::OdomSensors odom_sensors(nullptr, nullptr, nullptr, nullptr, &imu);
lemlib::ControllerSettings linear(cfg::LINEAR_KP, cfg::LINEAR_KI, cfg::LINEAR_KD,
                                  3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular(cfg::ANGULAR_KP, cfg::ANGULAR_KI, cfg::ANGULAR_KD,
                                   3, 1, 100, 3, 500, 0);
}  // namespace

lemlib::Chassis chassis(drivetrain, linear, angular, odom_sensors);
}  // namespace robot

namespace robot::internal {

void hardware_init() {
    // set_brake_mode_all: set_brake_mode() only sets the first motor.
    lift_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    left_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    aux_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    distance_present = claw_distance.is_installed();
    imu_present = imu.is_installed();
    if (!distance_present) ui::warn("No distance sensor on %d -- auto-clamp off", cfg::DISTANCE_PORT);
    if (!imu_present) ui::warn("No IMU on %d -- auton turns will drift", cfg::IMU_PORT);

    chassis.calibrate();  // blocks ~3 s while the IMU calibrates
}

}  // namespace robot::internal
