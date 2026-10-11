#include "main.h"

#include "lemlib/api.hpp"
#include "ui/ui.hpp"

/* ---------------------------------------------------------------------------
 * Hardware
 * ------------------------------------------------------------------------ */

pros::MotorGroup left_mg({-1, -2, -3});    // forward ports 1 & 3, reversed port 2
pros::MotorGroup right_mg({11, 14, 15});  // forward port 5, reversed ports 4 & 6
pros::MotorGroup aux_mg({9, -10});  // port 9 forward, port 10 reversed

// Pneumatic piston on ADI (3-wire) port A/1. false = solenoid de-energised =
// piston contracted, which is how it comes up on boot.
pros::adi::DigitalOut piston(1, false);
bool piston_extended = false;

/* ---------------------------------------------------------------------------
 * Autonomous routines
 *
 * Each of these shows up on the brain's AUTON page. Add, rename or delete them
 * freely -- the selector builds itself from whatever is registered below.
 * ------------------------------------------------------------------------ */

void red_rush() {
    ui::log("red_rush: driving out");
    // TODO: your routine
    pros::delay(500);
}

void red_safe() {
    ui::log("red_safe: scoring preload");
    // TODO: your routine
    pros::delay(500);
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

/* ---------------------------------------------------------------------------
 * Competition entry points
 * ------------------------------------------------------------------------ */

void initialize() {
    // --- what shows up on the AUTON page -------------------------------
    ui::add_auton(ui::Alliance::Red, "Rush", "Contest the middle, score 2", red_rush);
    ui::add_auton(ui::Alliance::Red, "Safe", "Preload only, stay home", red_safe);
    ui::add_auton(ui::Alliance::Blue, "Rush", "Contest the middle, score 2", blue_rush);
    ui::add_auton(ui::Alliance::Blue, "Safe", "Preload only, stay home", blue_safe);
    ui::add_auton(ui::Alliance::Skills, "Skills", "Full 60 second run", skills);
    ui::add_auton(ui::Alliance::Skills, "Nothing", "Sit still", do_nothing);

    // --- what shows up on the DIAG page --------------------------------
    ui::track_motor_group("Drive L", {-1, -2, -3});
    ui::track_motor_group("Drive R", {11, 14, 15});
    ui::track_flag("Piston", [] { return piston_extended; });

    // Uncomment and correct the ports as you add hardware:
    // ui::track_imu("IMU", 10);
    // ui::track_motor("Intake", 7);
    // ui::track_rotation("Odom X", 11);
    // ui::track_value("Lift", "deg", [] { return lift.get_position(); });
    // ui::track_flag("Clamp", [] { return clamp.get_value() != 0; });

    // --- what shows up on the FIELD page -------------------------------
    // Once a LemLib chassis exists, point the map at it:
    //
    //   ui::set_pose_source([] {
    //       lemlib::Pose p = chassis.getPose();
    //       return ui::Pose{p.x, p.y, p.theta};
    //   });
    //   ui::on_reset_pose([] { chassis.setPose(0, 0, 0); });

    ui::start();  // always last
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Whatever is selected on the brain. Nothing to change here, ever.
    ui::run_auton();
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        // Left joystick (up/down): drives both sides together, forward/backward.
        // Right joystick (left/right): drives the sides in opposite directions to turn.
        const int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        left_mg.move(forward + turn);
        right_mg.move(forward - turn);

        // Top-left button drives aux_mg forward, top-right drives it backward.
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            aux_mg.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            aux_mg.move(-127);
        } else {
            aux_mg.move(0);
        }

        // Button A toggles the piston: contracted -> expanded -> contracted.
        // get_digital_new_press only fires on the rising edge, so holding A down
        // does not flap the solenoid.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            piston_extended = !piston_extended;
            piston.set_value(piston_extended);
        }

        pros::delay(20);
    }
}
