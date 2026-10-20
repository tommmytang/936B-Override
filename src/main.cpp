#include "main.h"

#include <cmath>

#include "lemlib/api.hpp"
#include "robot/config.hpp"
#include "robot/robot.hpp"
#include "ui/ui.hpp"

// LemLib: inches and degrees, heading 0 = +Y, clockwise. Motions are async --
// waitUntilDone() before anything that needs the robot to have arrived.
// Paused autons: paused/autons_paused.cpp (not compiled).

using robot::chassis;

/** Logs a step and the current pose to the LOG page. */
void step(const char* what) {
    const lemlib::Pose p = chassis.getPose();
    ui::log("%s  (%.1f, %.1f, %.0f deg)%s", what, p.x, p.y, p.theta,
            chassis.isInMotion() ? "  [still moving]" : "");
}

/** Finishes the running drive, slowing as a cone gets close and stopping when
 *  auto-clamp grabs it. Returns true if it grabbed. */
bool approach_and_grab(int max_power, std::uint32_t timeout_ms) {
    constexpr int SLOW_FROM_MM = 300;
    constexpr int MIN_POWER = 25;
    const int seat_mm = robot::config::CLAMP_SEAT_MM;
    const std::uint32_t start = pros::millis();

    while (chassis.isInMotion() && !robot::claw::closed()) {
        const int mm = robot::claw::distance_mm();
        if (mm > 0 && mm <= SLOW_FROM_MM) {
            chassis.cancelMotion();
            break;
        }
        pros::delay(10);
    }

    while (!robot::claw::closed() && pros::millis() - start < timeout_ms) {
        const int mm = robot::claw::distance_mm();
        if (mm <= 0 || mm > SLOW_FROM_MM) break;
        double ramp = static_cast<double>(mm - seat_mm) / (SLOW_FROM_MM - seat_mm);
        ramp = std::fmin(std::fmax(ramp, 0.0), 1.0);
        chassis.arcade(MIN_POWER + static_cast<int>((max_power - MIN_POWER) * ramp), 0, true);
        pros::delay(10);
    }

    chassis.arcade(0, 0, true);
    if (!robot::claw::closed()) ui::warn("approach_and_grab: no grab");
    return robot::claw::closed();
}

/** Lift to `target` arm degrees, or until timeout. Up power is capped at 102. */
void lift_to(double target, std::uint32_t timeout_ms, int up_power = 80) {
    const bool up = target > robot::lift::height();
    const std::uint32_t start = pros::millis();
    robot::lift::manual(up ? up_power : -60);
    while (pros::millis() - start < timeout_ms &&
           (up ? robot::lift::height() < target : robot::lift::height() > target)) {
        pros::delay(10);
    }
    robot::lift::manual(0);
    if (pros::millis() - start >= timeout_ms) {
        ui::warn("lift_to %.1f timed out at %.1f", target, robot::lift::height());
    }
}

/** Drive straight `inches` (negative = back) at fixed power, by odometry.
 *  LemLib barely moves on short distances. */
void drive_straight(double inches, std::uint32_t timeout_ms) {
    constexpr int POWER = 45;
    // waitUntilDone() hangs if no motion has ever run.
    if (chassis.isInMotion()) chassis.waitUntilDone();
    const lemlib::Pose start = chassis.getPose();
    const std::uint32_t t0 = pros::millis();
    const int power = inches >= 0 ? POWER : -POWER;
    while (pros::millis() - t0 < timeout_ms) {
        const lemlib::Pose now = chassis.getPose();
        if (std::hypot(now.x - start.x, now.y - start.y) >= std::fabs(inches)) break;
        chassis.arcade(power, 0, true);
        pros::delay(10);
    }
    chassis.arcade(0, 0, true);
}

/* One toggle flip from the Left start pose (facing 180, toggle behind):
 * forward, lift to 88, ram back, stay wedged, reset pose, lift up 15.
 * second = true: skip the lift to 88 and stay wedged longer before lifting. */
void do_toggle(bool second = false) {
    constexpr double TOGGLE_LIFT = 88;     // 90 too high, 86 did not flip
    constexpr double RUN_UP_CM = 10;
    constexpr int RAM_POWER = 127;         // 110 was not enough
    constexpr int TOGGLE_LIFT_POWER = 127;
    constexpr std::uint32_t RAM_MS = 400;
    constexpr double SAFETY_RAISE = 15;
    const std::uint32_t WEDGE_MS = second ? 800 : 400;
    const std::uint32_t SETTLE_MS = second ? 400 : 150;

    step("T1 run-up forward");
    drive_straight(RUN_UP_CM / 2.54, 1000);

    if (!second) {
        step("T2 lift to toggle height");
        lift_to(TOGGLE_LIFT, 2500, TOGGLE_LIFT_POWER);
    }

    step("T3 ram back into toggle");
    chassis.arcade(-RAM_POWER, 0, true);
    pros::delay(RAM_MS);

    step("T3b re-seat against toggle");
    chassis.arcade(-40, 0, true);
    pros::delay(WEDGE_MS);
    chassis.arcade(0, 0, true);
    pros::delay(SETTLE_MS);

    // The ram slips the wheels; against the wall we're back at the start pose.
    chassis.setPose(0, 64.065, 180);

    step("T4 lift up more");
    lift_to(robot::lift::height() + SAFETY_RAISE, 1000, TOGGLE_LIFT_POWER);
}

/* Left side: `toggles` flips (red needs 2 from yellow, blue 1), then pin 1 on
 * the first goal. Same coordinates for both -- not mirrored for blue. */
void left_one_pin(int toggles) {
    chassis.setPose(0, 64.065, 180);
    for (int i = 0; i < toggles; ++i) do_toggle(i > 0);

    step("1 drive out");
    chassis.moveToPoint(0, 46.308, 2000, {.maxSpeed = 50});
    pros::delay(1000);
    step("2 turn to 90");
    chassis.turnToHeading(90, 1000);
    pros::delay(1000);

    step("4 drive to goal");
    chassis.moveToPoint(16.523, 51.972, 2000, {.maxSpeed = 50});
    chassis.waitUntilDone();

    step("4b push into goal");  // the goal squares the robot up
    chassis.arcade(50, 0, true);
    pros::delay(500);
    chassis.arcade(0, 0, true);

    step("5 back up 4.5 cm");
    drive_straight(-4.5 / 2.54, 1000);
    step("5 lower lift");
    lift_to(30, 2500);
    robot::lift::manual(-60);
    pros::delay(400);  // longer = lower
    robot::lift::manual(0);
    step("5 release");
    robot::claw::open();

    step("6 back off goal");
    drive_straight(-15.0 / 2.54, 1000);
    step("done");
}

void red_left() { left_one_pin(2); }
void blue_left() { left_one_pin(1); }

void red_rush() {
    // From path.jerryio. Reverse legs back up instead of turning around.
    constexpr float FAST = 107;
    constexpr float SLOW = 60;

    chassis.setPose(-61.074, -0.291, 90);
    chassis.moveToPoint(-46.898, -0.477, 1500, {.maxSpeed = FAST});

    chassis.turnToPoint(-46.898, -15.585, 1000);
    chassis.moveToPoint(-46.898, -15.585, 1500, {.maxSpeed = FAST});

    chassis.moveToPoint(-46.898, -7.752, 1200, {.forwards = false, .maxSpeed = FAST});

    chassis.turnToPoint(-29.645, -19.094, 1000);
    chassis.moveToPoint(-29.645, -19.094, 2000, {.maxSpeed = FAST});

    chassis.turnToPoint(-38.155, -21.385, 1000, {.forwards = false});
    chassis.moveToPoint(-38.155, -21.385, 1200, {.forwards = false, .maxSpeed = FAST});

    chassis.turnToPoint(-31.936, -21.385, 1000);
    chassis.moveToPoint(-31.936, -21.385, 1200, {.maxSpeed = FAST});

    chassis.turnToPoint(-43.393, -40.534, 1000, {.forwards = false});
    chassis.moveToPoint(-43.393, -40.534, 2000, {.forwards = false, .maxSpeed = FAST});

    chassis.moveToPose(-51.412, -31.205, 208, 2500, {.forwards = false, .maxSpeed = SLOW});
    chassis.waitUntilDone();
}

void red_right() {
    chassis.setPose(61.282, -0.151, 270);
    step("1 drive out");
    chassis.moveToPoint(44.696, -0.151, 2000, {.maxSpeed = 50});
    pros::delay(1000);
    step("2 turn to 0");
    chassis.turnToHeading(0, 1000);
    pros::delay(1000);

    step("3 lift up");
    robot::lift::manual(80);
    pros::delay(300);  // longer = higher
    robot::lift::manual(0);

    step("4 drive to goal");
    chassis.moveToPoint(43.574, 13, 2000, {.maxSpeed = 50});
    chassis.waitUntilDone();

    step("5 drop + release");
    robot::lift::manual(-60);
    pros::delay(400);
    robot::lift::manual(0);
    robot::claw::open();

    constexpr double BACK_UP_IN = 15.0 / 2.54;
    step("6 back up");
    const lemlib::Pose here = chassis.getPose(true);  // radians
    chassis.moveToPoint(here.x - BACK_UP_IN * std::sin(here.theta),
                        here.y - BACK_UP_IN * std::cos(here.theta), 1200,
                        {.forwards = false, .maxSpeed = 50});

    // Aim at the cone (28.7, 18.6), swung EXTRA_TURN_DEG further left.
    constexpr double EXTRA_TURN_DEG = 8;
    chassis.waitUntilDone();
    const lemlib::Pose from = chassis.getPose();
    const double dx = 28.7 - from.x;
    const double dy = 18.6 - from.y;
    const double reach = std::hypot(dx, dy);
    const double aim = std::atan2(dx, dy) - EXTRA_TURN_DEG * M_PI / 180.0;
    const double aim_x = from.x + reach * std::sin(aim);
    const double aim_y = from.y + reach * std::cos(aim);

    constexpr int GRAB_SPEED = 35;
    step("7 aim at cone");
    chassis.turnToPoint(aim_x, aim_y, 1000);
    chassis.moveToPoint(aim_x, aim_y, 2500, {.maxSpeed = GRAB_SPEED});
    step("8 slow grab");
    approach_and_grab(GRAB_SPEED, 4000);
    pros::delay(500);

    // Stack on (47, 24); stop short so the claw, not the centre, is over it.
    constexpr double STACK_X = 47;
    constexpr double STACK_Y = 24;
    constexpr double STOP_SHORT_IN = 8.0 / 2.54 + 9.0;

    step("9 lift for stack");
    robot::lift::manual(80);
    pros::delay(450);
    robot::lift::manual(0);

    step("10 turn to stack");
    chassis.turnToPoint(STACK_X, STACK_Y, 2500, {.maxSpeed = 60});  // slow: lift is up
    chassis.waitUntilDone();

    step("11 drive to stack");
    const lemlib::Pose at = chassis.getPose();
    const double sx = STACK_X - at.x;
    const double sy = STACK_Y - at.y;
    const double gap = std::hypot(sx, sy);
    if (gap > STOP_SHORT_IN) {
        const double k = (gap - STOP_SHORT_IN) / gap;
        chassis.moveToPoint(at.x + sx * k, at.y + sy * k, 2500, {.maxSpeed = 50});
        chassis.waitUntilDone();
    }

    step("12 drop + release");
    robot::lift::manual(-60);
    pros::delay(400);
    robot::lift::manual(0);
    robot::claw::open();
    step("done");
}

/* One toggle flip, then forward and lift down. */
void toggle_test() {
    chassis.setPose(0, 64.065, 180);
    do_toggle();
    step("5 forward, away from toggle");
    drive_straight(10.0 / 2.54, 1000);
    step("6 lower lift");
    lift_to(0, 2500);
    step("done");
}

void do_nothing() { ui::warn("No-op auton selected"); }

/* PID tuning: run, watch the LOG page, change gains in config.hpp, repeat. */
void log_final_pose() {
    chassis.waitUntilDone();
    const lemlib::Pose p = chassis.getPose();
    ui::log("ended at x=%.1f y=%.1f theta=%.1f", p.x, p.y, p.theta);
}

void tune_turn() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 100000);
    log_final_pose();  // want theta = 90
}

void tune_drive() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 100000);
    log_final_pose();  // want y = 24, x and theta near 0
}

/* --- Competition entry points ------------------------------------------- */

void initialize() {
    robot::init();  // zeroes the lift (have it down), calibrates the IMU (keep still)

    using ui::Alliance;
    ui::add_auton(Alliance::Red, "Rush", "Contest the middle, score 2", red_rush);
    ui::add_auton(Alliance::Red, "Right", "Preload only, stay home", red_right);
    ui::add_auton(Alliance::Red, "Left", "2 toggles + 1 pin", red_left);
    ui::add_auton(Alliance::Red, "Toggle test", "1 toggle, then lift down", toggle_test);
    ui::add_auton(Alliance::Red, "Nothing", "Sit still", do_nothing);
    ui::add_auton(Alliance::Blue, "Left", "1 toggle + 1 pin", blue_left);
    ui::add_auton(Alliance::Blue, "Nothing", "Sit still", do_nothing);
    ui::add_auton(Alliance::Skills, "Tune turn", "Turn 90 deg, tune ANGULAR_K*", tune_turn);
    ui::add_auton(Alliance::Skills, "Tune drive", "Drive 24 in, tune LINEAR_K*", tune_drive);

    ui::set_pose_source([] {
        const lemlib::Pose p = chassis.getPose();
        return ui::Pose{p.x, p.y, p.theta};
    });
    ui::on_reset_pose([] { chassis.setPose(0, 0, 0); });

    ui::start();  // always last
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    ui::run_auton();
}

void opcontrol() {
    chassis.cancelAllMotions();  // a leftover auton motion would fight the driver
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    bool testing = false;
    while (true) {
        // The AUTON page's TEST button runs a routine; stay off the motors meanwhile.
        const bool test_now = ui::auton_test_running();
        if (test_now != testing) {
            testing = test_now;
            chassis.setBrakeMode(testing ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
        }
        if (!testing) robot::driver_step();
        pros::delay(10);
    }
}
