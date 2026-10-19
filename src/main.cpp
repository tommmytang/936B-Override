#include "main.h"

#include <cmath>

#include "lemlib/api.hpp"
#include "robot/config.hpp"
#include "robot/robot.hpp"
#include "ui/ui.hpp"

/* ---------------------------------------------------------------------------
 * Hardware and subsystem control now live in src/robot/.
 *
 *   include/robot/config.hpp   ports, button bindings, speeds, gains, presets
 *   src/robot/drive.cpp        creep mode, heading hold, lift-aware scaling
 *   src/robot/lift.cpp         DR4B hold, cone-count presets, stall guard
 *   src/robot/claw.cpp         piston + distance-sensor auto-clamp
 *
 * If you want to change a number, change it in config.hpp. You should not
 * need to touch this file again.
 * ------------------------------------------------------------------------ */

/* ---------------------------------------------------------------------------
 * Autonomous routines
 *
 * Each of these shows up on the brain's AUTON page. Add, rename or delete them
 * freely -- the selector builds itself from whatever is registered below.
 *
 * LemLib cheat sheet. Units are inches and degrees. setPose() says where the
 * robot starts; +Y is straight ahead from there, theta increases clockwise.
 *
 *   chassis.setPose(x, y, theta);                 where the robot starts
 *   chassis.moveToPoint(x, y, timeout_ms);        drive to a point
 *   chassis.moveToPoint(x, y, 1500, {.forwards = false});  ...in reverse
 *   chassis.moveToPose(x, y, theta, timeout_ms);  arrive facing theta (curves)
 *   chassis.turnToHeading(theta, timeout_ms);     turn on the spot
 *   chassis.turnToPoint(x, y, timeout_ms);        face a point
 *   chassis.waitUntilDone();                      block until the motion ends
 *   chassis.waitUntil(10);                        ...or until 10" into it
 *
 * Motions are ASYNC: each one starts and returns immediately. A second motion
 * waits for the first on its own, but anything else -- the claw, the lift --
 * runs straight away. That is how you raise the lift while driving. Put a
 * waitUntilDone() before anything that needs the robot to have arrived.
 *
 * The timeout is a safety net, not a duration: set it a bit longer than the
 * move takes, so a robot stuck on a field element gives up and carries on.
 * ------------------------------------------------------------------------ */

using robot::chassis;

/** Logs a routine step and where the robot thinks it is. If a routine stalls,
 *  the LOG page shows the last step that started, and when. */
void step(const char* what) {
    const lemlib::Pose p = chassis.getPose();
    ui::log("%s  (%.1f, %.1f, %.0f deg)%s", what, p.x, p.y, p.theta,
            chassis.isInMotion() ? "  [still moving]" : "");
}

/**
 * Finish the drive that is already running -- slowing down as a cone gets
 * close, and stopping the moment auto-clamp grabs it.
 *
 * LemLib cannot change a motion's speed halfway through, so once the claw
 * sensor sees something within SLOW_FROM_MM this cancels the motion and
 * takes over, driving straight at a power that ramps from max_power down to
 * MIN_POWER as the gap closes to CLAMP_SEAT_MM. If no cone ever shows up, the
 * original drive simply finishes.
 *
 * Needs auto-clamp armed and the claw open. Returns true if it grabbed.
 */
bool approach_and_grab(int max_power, std::uint32_t timeout_ms) {
    constexpr int SLOW_FROM_MM = 300;  // start slowing down here
    constexpr int MIN_POWER = 25;      // slowest creep that still moves the robot
    const int seat_mm = robot::config::CLAMP_SEAT_MM;
    const std::uint32_t start = pros::millis();

    // Let LemLib drive until the cone shows up in the claw sensor.
    while (chassis.isInMotion() && !robot::claw::closed()) {
        const int mm = robot::claw::distance_mm();
        if (mm > 0 && mm <= SLOW_FROM_MM) {
            chassis.cancelMotion();
            break;
        }
        pros::delay(10);
    }

    // Creep in, slower the closer it gets, until the claw grabs.
    while (!robot::claw::closed() && pros::millis() - start < timeout_ms) {
        const int mm = robot::claw::distance_mm();
        if (mm <= 0 || mm > SLOW_FROM_MM) break;  // nothing ahead (any more)

        double ramp = static_cast<double>(mm - seat_mm) / (SLOW_FROM_MM - seat_mm);
        ramp = std::fmin(std::fmax(ramp, 0.0), 1.0);
        const int power = MIN_POWER + static_cast<int>((max_power - MIN_POWER) * ramp);
        chassis.arcade(power, 0, true);
        pros::delay(10);
    }

    chassis.arcade(0, 0, true);
    if (!robot::claw::closed()) ui::warn("approach_and_grab: no grab");
    return robot::claw::closed();
}

void red_rush() {
    ui::log("red_rush: driving out");

    // From path.jerryio (V5RC 2027 Override field, GPS coordinates), corner
    // points only: each straight leg is "turn to face it, then drive". The
    // path does not say which way the robot faces, so legs marked REVERSE
    // back up instead of turning around -- picked to keep turns small.
    constexpr float FAST = 107;  // the path's own speed
    constexpr float SLOW = 60;   // the hook at the end

    chassis.setPose(-61.074, -0.291, 90);

    // 1. out from the start
    chassis.moveToPoint(-46.898, -0.477, 1500, {.maxSpeed = FAST});

    // 2. down the field
    chassis.turnToPoint(-46.898, -15.585, 1000);
    chassis.moveToPoint(-46.898, -15.585, 1500, {.maxSpeed = FAST});

    // 3. back up the same line                                    REVERSE
    chassis.moveToPoint(-46.898, -7.752, 1200, {.forwards = false, .maxSpeed = FAST});

    // 4. diagonal across
    chassis.turnToPoint(-29.645, -19.094, 1000);
    chassis.moveToPoint(-29.645, -19.094, 2000, {.maxSpeed = FAST});

    // 5. back off                                                 REVERSE
    chassis.turnToPoint(-38.155, -21.385, 1000, {.forwards = false});
    chassis.moveToPoint(-38.155, -21.385, 1200, {.forwards = false, .maxSpeed = FAST});

    // 6. forward again
    chassis.turnToPoint(-31.936, -21.385, 1000);
    chassis.moveToPoint(-31.936, -21.385, 1200, {.maxSpeed = FAST});

    // 7. long diagonal                                            REVERSE
    chassis.turnToPoint(-43.393, -40.534, 1000, {.forwards = false});
    chassis.moveToPoint(-43.393, -40.534, 2000, {.forwards = false, .maxSpeed = FAST});

    // 8. the hook: curve round to finish at (-51.4, -31.2)         REVERSE
    // moveToPose arrives at the same point and angle as the drawn curve but
    // takes its own route there, so the swing will not match it exactly.
    chassis.moveToPose(-51.412, -31.205, 208, 2500, {.forwards = false, .maxSpeed = SLOW});
    chassis.waitUntilDone();
}

void red_safe() {
    ui::log("red_safe: scoring preload");
    chassis.setPose(61.282, -0.151, 270);
    step("1 drive out");
    chassis.moveToPoint(44.696, -0.151, 2000, {.maxSpeed=50});
    pros::delay(1000);
    step("2 turn to 0");
    chassis.turnToHeading(0, 1000);
    pros::delay(1000);

    // Lift a little before the last drive. Timed, because lift::move_to()
    // needs ENABLE_LIFT_HOLD. Longer delay = higher.
    step("3 lift up");
    robot::lift::manual(80);
    pros::delay(300);
    robot::lift::manual(0);  // the lift motors brake-hold it there

    step("4 drive to goal");
    chassis.moveToPoint(43.574, 13, 2000, {.maxSpeed=50});
    chassis.waitUntilDone();  // arrive before dropping

    // Drop the lift back down, then let go.
    step("5 drop + release");
    robot::lift::manual(-60);
    pros::delay(400);
    robot::lift::manual(0);
    robot::claw::open();

    // Back up 15 cm, straight back from wherever it actually stopped, to
    // clear the goal. Heading 0 is +Y and grows clockwise, so "forward" is
    // (sin, cos) of the heading.
    constexpr double BACK_UP_IN = 15.0 / 2.54;  // cm -> inches
    step("6 back up");
    const lemlib::Pose here = chassis.getPose(true);  // true = radians
    chassis.moveToPoint(here.x - BACK_UP_IN * std::sin(here.theta),
                        here.y - BACK_UP_IN * std::cos(here.theta), 1200,
                        {.forwards = false, .maxSpeed = 50});

    // Then aim at (28.7, 18.6) -- swung EXTRA_TURN_DEG further left -- and
    // drive at the cone, slowing as it gets close and stopping as soon as
    // the claw grabs it. Positive turns further left, negative further right.
    constexpr double EXTRA_TURN_DEG = 8;
    chassis.waitUntilDone();  // finish backing up before measuring from here
    const lemlib::Pose from = chassis.getPose();
    const double dx = 28.7 - from.x;
    const double dy = 18.6 - from.y;
    const double reach = std::hypot(dx, dy);
    const double aim = std::atan2(dx, dy) - EXTRA_TURN_DEG * M_PI / 180.0;
    const double aim_x = from.x + reach * std::sin(aim);
    const double aim_y = from.y + reach * std::cos(aim);

    // Slower than the rest of the routine: this is the grab. Lower = gentler.
    constexpr int GRAB_SPEED = 35;
    step("7 aim at cone");
    chassis.turnToPoint(aim_x, aim_y, 1000);
    chassis.moveToPoint(aim_x, aim_y, 2500, {.maxSpeed = GRAB_SPEED});
    step("8 slow grab");
    approach_and_grab(GRAB_SPEED, 4000);
    pros::delay(500);  // let the claw settle on the cone before moving off

    // Stack it: turn round to face the cone at (47, 24), lift, drive up to
    // it, then lower and let go.
    //
    // Positions are the robot's CENTRE, and the claw sits ahead of it, so
    // stop well short: 5 cm, plus ~9 in for centre-to-claw. Bigger number =
    // shorter drive.
    constexpr double STACK_X = 47;
    constexpr double STACK_Y = 24;
    constexpr double STOP_SHORT_IN = 8.0 / 2.54 + 9.0;

    // Lift first, so the held cone is already up when the robot swings round.
    step("9 lift for stack");
    robot::lift::manual(80);
    pros::delay(450);  // longer = higher; the held cone has to clear the other one
    robot::lift::manual(0);

    // Turn slowly: a raised lift with a cone on it tips and swings if the
    // robot spins hard. Lower maxSpeed = gentler (out of 127).
    step("10 turn to stack");
    chassis.turnToPoint(STACK_X, STACK_Y, 2500, {.maxSpeed = 60});
    chassis.waitUntilDone();

    step("11 drive to stack");
    const lemlib::Pose at = chassis.getPose();
    const double sx = STACK_X - at.x;
    const double sy = STACK_Y - at.y;
    const double gap = std::hypot(sx, sy);
    if (gap > STOP_SHORT_IN) {
        const double k = (gap - STOP_SHORT_IN) / gap;  // fraction of the way there
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

void blue_rush() {
    ui::log("blue_rush: driving out");
    // TODO: your routine
    pros::delay(500);
}

void blue_safe() {
    ui::log("blue_safe: scoring preload");
    // TODO: your routine
    pros::delay(500);
}

void skills() {
    ui::log("skills: starting 60s run");
    // TODO: your routine
    pros::delay(500);
}

void do_nothing() { ui::warn("No-op auton selected"); }

/* PID tuning. Run these from the AUTON page, watch the robot, change the
 * gains in config.hpp, re-upload, repeat. The huge timeout is deliberate: it
 * lets you watch the controller settle (or oscillate forever) instead of
 * cutting it off. Final pose goes to the LOG page. */
void log_final_pose() {
    chassis.waitUntilDone();
    const lemlib::Pose p = chassis.getPose();
    ui::log("ended at x=%.1f y=%.1f theta=%.1f", p.x, p.y, p.theta);
}

void tune_turn() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 100000);
    log_final_pose();  // want theta = 90.0
}

void tune_drive() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 100000);
    log_final_pose();  // want y = 24.0, x and theta near 0
}

/* ---------------------------------------------------------------------------
 * Competition entry points
 * ------------------------------------------------------------------------ */

void initialize() {
    // Devices, control tasks and all the DIAG page rows. Zeroes the lift
    // against wherever the arms are sitting, so have them down on the hard
    // stop at power-on. Also calibrates the LemLib chassis, which blocks for
    // ~2-3 s while the IMU settles -- keep the robot still.
    robot::init();

    // --- what shows up on the AUTON page -------------------------------
    // Three rows fit on screen per tab; a fourth needs scrolling.
    ui::add_auton(ui::Alliance::Red, "Rush", "Contest the middle, score 2", red_rush);
    ui::add_auton(ui::Alliance::Red, "Safe", "Preload only, stay home", red_safe);
    ui::add_auton(ui::Alliance::Red, "Nothing", "Sit still", do_nothing);
    ui::add_auton(ui::Alliance::Blue, "Rush", "Contest the middle, score 2", blue_rush);
    ui::add_auton(ui::Alliance::Blue, "Safe", "Preload only, stay home", blue_safe);
    ui::add_auton(ui::Alliance::Blue, "Nothing", "Sit still", do_nothing);
    ui::add_auton(ui::Alliance::Skills, "Skills", "Full 60 second run", skills);
    ui::add_auton(ui::Alliance::Skills, "Tune turn", "Turn 90 deg, tune ANGULAR_K*", tune_turn);
    ui::add_auton(ui::Alliance::Skills, "Tune drive", "Drive 24 in, tune LINEAR_K*", tune_drive);

    // --- what shows up on the FIELD page -------------------------------
    ui::set_pose_source([] {
        const lemlib::Pose p = chassis.getPose();
        return ui::Pose{p.x, p.y, p.theta};
    });
    ui::on_reset_pose([] { chassis.setPose(0, 0, 0); });

    pros::lcd::initialize();

    // LVGL is pumped by PROS's own "Display Daemon (PROS)" task, started at
    // boot by display_initialize(). It is the ONLY thing that may call
    // lv_timer_handler(): LVGL is not thread-safe, and a second pump task
    // racing it corrupted LVGL's state and crashed the brain with a prefetch
    // abort in lv-pump. Do not add another pump.
    //
    // It runs at priority 3, below every default-priority task, so a busy
    // robot can starve it and leave the screen blank. Lift it to default.
    if (pros::task_t daemon = pros::c::task_get_by_name("Display Daemon (PROS)")) {
        pros::c::task_set_priority(daemon, TASK_PRIORITY_DEFAULT);
    } else {
        ui::warn("PROS display daemon not found -- screen may not update");
    }

    ui::start();  // always last
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Hold, not coast: the robot should stop where LemLib puts it, not roll
    // on past the target.
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // Whatever is selected on the brain. Nothing to change here, ever.
    ui::run_auton();
}

void opcontrol() {
    // An async motion runs in its own task and survives the end of the auton
    // period. Kill it, or it fights the driver for the drive motors.
    chassis.cancelAllMotions();
    // Back to coast for driver control -- see hardware_init() for why.
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    // Bindings live in include/robot/config.hpp. In short:
    //   L1/R1 lift      R2 creep      L2 aux on/off  A claw
    //   D-pad up/down cone count   right auto-clamp   left stow
    //   hold X re-zero lift        hold Y save this height as the preset
    bool testing = false;
    while (true) {
        // TEST on the AUTON page runs a routine right here in opcontrol. Keep
        // off the motors until it finishes -- driver_step() writes 0 to the
        // drive every tick with the sticks centred, which would fight LemLib
        // and make every tuning run meaningless. Brake like a real auton.
        const bool test_now = ui::auton_test_running();
        if (test_now != testing) {
            testing = test_now;
            chassis.setBrakeMode(testing ? pros::E_MOTOR_BRAKE_HOLD
                                         : pros::E_MOTOR_BRAKE_COAST);
        }
        if (testing) {
            pros::delay(10);
            continue;
        }

        robot::driver_step(master);
        pros::delay(10);
    }
}
