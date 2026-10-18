/**
 * @file robot/robot.hpp
 *
 * Subsystem control for the 936B robot: drive, DR4B lift and pneumatic claw.
 * ---------------------------------------------------------------------------
 * Integration is three calls:
 *
 *     void initialize() {
 *         robot::init();              // devices, tasks, telemetry
 *         ui::start();
 *     }
 *
 *     void opcontrol() {
 *         pros::Controller master(pros::E_CONTROLLER_MASTER);
 *         while (true) {
 *             robot::driver_step(master);
 *             pros::delay(10);
 *         }
 *     }
 *
 * Everything tunable is in robot/config.hpp.
 *
 * NOTHING HERE BLOCKS. The lift and claw run their own tasks; driver_step()
 * reads buttons and returns immediately. A macro can never hold the driver
 * hostage, and any manual input cancels an in-progress move.
 */
#pragma once

#include <cstdint>

#include "lemlib/api.hpp"
#include "pros/misc.hpp"

namespace robot {

/** The LemLib chassis, for autonomous. Shares the drive motors and IMU with
 *  driver control. Gains and dimensions live in config.hpp.
 *
 *  Do not call chassis motion functions during opcontrol: driver control
 *  writes to the same motors every tick and the two will fight. */
extern lemlib::Chassis chassis;

/** Constructs devices, starts the control tasks, registers UI telemetry and
 *  zeroes the lift. Call once from initialize(), before ui::start().
 *
 *  The lift tares against wherever it is sitting at boot, so put the arms down
 *  on their hard stop before powering on. */
void init();

/** One iteration of driver control. Call from opcontrol() every ~10 ms. */
void driver_step(pros::Controller& master);

/* =========================================================================
 * Drive
 * ========================================================================= */
namespace drive {

/** Arcade drive with creep scaling, lift-aware scaling, slew limiting and
 *  IMU heading hold all applied. Inputs are raw controller counts. */
void arcade(int forward, int turn, bool creep);

/** Stops both sides and drops any latched heading. */
void stop();

/** True while the IMU is present and finished calibrating. */
bool imu_ready();

/** Heading the drive is currently holding, in continuous degrees. */
double heading();

/** Forgets the latched heading; it re-latches on the next straight stretch. */
void release_heading();

/** Tilt magnitude in degrees from the configured anti-tip axis, or 0 with no
 *  IMU. Track this on the DIAG page to verify you picked the right axis. */
double tilt();

/** True while anti-tip is actively clamping the drive. */
bool tipping();

}  // namespace drive

/* =========================================================================
 * Lift -- all heights are ARM DEGREES above the hard stop
 * ========================================================================= */
namespace lift {

/** Current height. */
double height();

/** 0.0 at the hard stop, 1.0 at LIFT_MAX_DEG. Drives the drive scaling. */
double height_fraction();

/** Hand control to the driver for this tick. Cancels any preset move.
 *  @param power -127..127; 0 means "hold wherever you are". */
void manual(int power);

/** Start a non-blocking move to an absolute height. */
void move_to(double arm_degrees);

/** True once a move_to() has settled inside tolerance. */
bool at_target();

/** True while the driver has hold of the lift. The stack macro watches this
 *  and aborts, so manual input always wins. */
bool is_manual();

/** True if the lift was commanded but is not moving -- gears skipping or
 *  something jammed. Clears when the command changes. */
bool stalled();

/** Declares the current position to be zero. Use with the arms resting on
 *  the hard stop. */
void tare();

/* --- cone counter ---------------------------------------------------- */

/** How many cones are already on the stack you are building. 0..MAX_CONES-1. */
int cone_count();
void set_cone_count(int n);
void bump_cone_count(int delta);

/** Height registered for the current cone count. */
double preset_height();

/** Send the lift to the preset for the current cone count. */
void go_to_preset();

/** Save the lift's current height as the preset for the current cone count
 *  and write it to the SD card. This is how you tune heights at a comp
 *  without recompiling. */
void teach_preset();

}  // namespace lift

/* =========================================================================
 * Stack macro -- raise, dunk, release on load transfer, clear
 * ========================================================================= */
namespace stack {

/** The B button. Context-aware:
 *   - not yet at the preset  -> raise to it, then stop and wait
 *   - at the preset          -> dunk, release and clear
 *   - already running        -> abort
 *  Never blocks; the sequence runs in its own task. */
void begin();

/** Stops the sequence wherever it is and hands the lift back. Called
 *  automatically on any manual lift input and by anti-tip. */
void abort();

/** True while a sequence is running. */
bool active();

/** Short label for the current step, for the controller screen. */
const char* step();

}  // namespace stack

/* =========================================================================
 * Claw
 * ========================================================================= */
namespace claw {

bool closed();
void close();
void open();
void toggle();

/** Whether auto-clamp is watching for a cone. */
bool armed();
void set_armed(bool on);

/** Last distance reading in mm, or -1 when no sensor / nothing in range. */
int distance_mm();

/** True when the distance sensor currently sees a seated cone. */
bool cone_seated();

/** How fast the gap in front of the claw is shrinking, mm/s. Positive means
 *  something is getting nearer; 0 with nothing in range. */
int closing_speed();

}  // namespace claw

}  // namespace robot
