/**
 * @file robot/config.hpp
 *
 * Every number you will ever want to change lives in this file.
 * ---------------------------------------------------------------------------
 * Ports, button bindings, speeds, PID gains, preset heights, sensor
 * thresholds. Nothing else in src/robot/ hard-codes a tunable value, so you
 * should never need to go hunting through the .cpp files at a competition.
 *
 * Anything marked  <<< SET THIS  depends on how your robot is actually wired
 * or geared and will be wrong until you measure it.
 */
#pragma once

#include <cstdint>

namespace robot::config {

/* =========================================================================
 * FEATURE FLAGS -- bring-up, one macro at a time
 *
 * EVERYTHING IS OFF BY DEFAULT. With all of these false the robot behaves
 * exactly like the original main.cpp did: arcade drive, L1/R1 lift straight
 * through to the motors on brake-hold, A toggles the claw. Nothing else runs.
 *
 * Turn ON exactly one, rebuild, upload, test it, then leave it on and move to
 * the next. The LOG page prints the enabled set at every boot, so you can
 * always see what is actually live.
 *
 * Suggested order -- each one depends on the ones above it being trustworthy:
 *
 *   0. (all false)      confirm drive, lift and claw still work at all
 *   1. LIFT_SENSING     the "Lift" row on the DIAG page is live from boot in
 *                       every mode. Tune LIFT_MOTOR_DEG_PER_ARM_DEG until it
 *                       matches a protractor on the real arm, THEN set this
 *                       flag -- it unlocks the four features built on that
 *                       number. Do this one FIRST; a wrong ratio makes the
 *                       limits, scaling, presets and anti-tip all misbehave
 *                       in ways that look like unrelated bugs.
 *   2. CREEP            hold R2, confirm the robot gets slow and controllable
 *   3. DRIVE_SLEW       confirm acceleration is smoother, not sluggish
 *   4. LIFT_SCALING     raise the lift, confirm the drive backs off
 *   5. HEADING_HOLD     drive straight hands-off-turn-stick down a field tile
 *   6. LIFT_HOLD        PD hold. Confirm the arm neither sags nor buzzes --
 *                       this is the one that needs LIFT_KF tuned
 *   7. LIFT_LIMITS      confirm the arm stops at both ends of travel
 *   8. PRESETS          teach a couple of heights, confirm L2 returns to them
 *   9. AUTO_CLAMP       confirm the claw fires on a cone and not on air
 *  10. ANTITIP          tilt the robot by hand with the lift up
 *  11. STACK_MACRO      run it on blocks with an EMPTY claw first
 *  12. MATCH_ALERTS     harmless, enable whenever
 * ========================================================================= */

inline constexpr bool ENABLE_LIFT_SENSING = false;  // "the height readout is correct"
inline constexpr bool ENABLE_CREEP = true;
inline constexpr bool ENABLE_DRIVE_SLEW = false;
inline constexpr bool ENABLE_LIFT_SCALING = false;
inline constexpr bool ENABLE_HEADING_HOLD = false;
inline constexpr bool ENABLE_LIFT_HOLD = false;     // false = raw passthrough, like the original
inline constexpr bool ENABLE_LIFT_LIMITS = false;   // soft stops at LIFT_MIN_DEG / LIFT_MAX_DEG
inline constexpr bool ENABLE_PRESETS = false;
inline constexpr bool ENABLE_AUTO_CLAMP = true;
inline constexpr bool ENABLE_ANTITIP = false;
inline constexpr bool ENABLE_STACK_MACRO = false;
inline constexpr bool ENABLE_MATCH_ALERTS = false;

/* Keep this on. It only ever REDUCES power, and it is protecting the gears
 * your own main.cpp comment says were already skipping teeth. */
inline constexpr bool ENABLE_STALL_GUARD = true;

/* Invalid combinations caught at build time rather than on the field. */
static_assert(!ENABLE_PRESETS || ENABLE_LIFT_HOLD,
              "ENABLE_PRESETS needs ENABLE_LIFT_HOLD: without position control there is "
              "nothing to hold a preset height.");
static_assert(!ENABLE_STACK_MACRO || ENABLE_PRESETS,
              "ENABLE_STACK_MACRO needs ENABLE_PRESETS: the macro raises to the preset for "
              "the current cone count.");
static_assert(!ENABLE_LIFT_LIMITS || ENABLE_LIFT_SENSING,
              "ENABLE_LIFT_LIMITS needs ENABLE_LIFT_SENSING: soft stops built on an "
              "unverified height will trap the arm.");
static_assert(!ENABLE_LIFT_SCALING || ENABLE_LIFT_SENSING,
              "ENABLE_LIFT_SCALING needs ENABLE_LIFT_SENSING: it scales the drive by a "
              "height you have not confirmed yet.");
static_assert(!ENABLE_ANTITIP || ENABLE_LIFT_SENSING,
              "ENABLE_ANTITIP needs ENABLE_LIFT_SENSING: it only arms above a lift height.");

/* =========================================================================
 * PORTS
 * ========================================================================= */

/* Drive -- negative port = reversed motor, same as pros::Motor. */
inline constexpr std::int8_t DRIVE_L1 = -1, DRIVE_L2 = -2, DRIVE_L3 = -3;
inline constexpr std::int8_t DRIVE_R1 = 6, DRIVE_R2 = 7, DRIVE_R3 = 8;

/* DR4B lift. */
inline constexpr std::int8_t LIFT_1 = 9, LIFT_2 = -10;

/* Claw solenoid on an ADI (3-wire) port. 1 = 'A', 2 = 'B', ... 8 = 'H'. */
inline constexpr std::uint8_t CLAW_ADI_PORT = 1;

/* Which way the piston grips. false = the piston CONTRACTS to close the claw,
 * which is how 936B is plumbed: the claw is closed at power-on, with the
 * solenoid off. Flip this if the claw ever opens when it should grab. */
inline constexpr bool CLAW_EXTEND_TO_CLOSE = false;

/* Sensors. All three are optional at runtime: if a port is empty the feature
 * that needs it turns itself off and the rest of the robot keeps working. */
inline constexpr std::uint8_t IMU_PORT = 11;       // <<< SET THIS
inline constexpr std::uint8_t DISTANCE_PORT = 5;   // in the claw throat
inline constexpr std::uint8_t LIFT_ROT_PORT = 13;  // <<< SET THIS -- on the DR4B pivot

/* ------------------------------------------------------------------------
 * Which sensor measures lift height.
 *
 * false -> the lift motors' built-in encoders. Works today, zero extra parts,
 *          but the count desynchronises permanently the moment the gears skip
 *          (see AUX/LIFT_MAX_POWER below -- yours have skipped before).
 * true  -> a Rotation sensor on the arm pivot. Measures the arm itself, so
 *          gear skip and motor swaps cannot corrupt it.
 *
 * Presets are stored in ARM DEGREES either way, so flipping this flag does
 * not invalidate heights you have already tuned -- you only need the matching
 * ratio constant below to be right.
 * --------------------------------------------------------------------- */
inline constexpr bool LIFT_USE_ROTATION = false;

/* =========================================================================
 * CONTROLLER BINDINGS
 *
 * Unchanged from before: L1/R1 raise and lower, A toggles the claw.
 * ========================================================================= */
//  LEFT STICK Y    drive forward / back
//  RIGHT STICK X   turn
//
//  L1              raise lift (manual -- always cancels any macro)
//  R1              lower lift (manual -- always cancels any macro)
//  L2   (tap)      send lift to the preset for the current cone count
//  R2   (hold)     creep mode -- precision speed
//
//  A    (tap)      toggle claw open/closed
//  B    (tap)      STACK. 1st press raises to the preset, 2nd press dunks,
//                  releases on contact and lifts clear. Pressed again while
//                  running, it aborts.
//  X    (hold 1s)  re-zero the lift at its hard stop
//  Y    (hold 1s)  teach: save current height as this cone count's preset
//
//  D-UP   (tap)    cone count +1
//  D-DOWN (tap)    cone count -1
//  D-LEFT (tap)    stow lift to travel height
//  D-RIGHT (tap)   arm / disarm auto-clamp

inline constexpr std::uint32_t HOLD_TO_CONFIRM_MS = 1000;

/* =========================================================================
 * DRIVE
 * ========================================================================= */

inline constexpr int JOY_DEADBAND = 8;        // raw stick counts ignored around centre
inline constexpr int DRIVE_MAX_POWER = 127;

/* Creep mode (hold R2). Forward is scaled less aggressively than turning,
 * because an over-eager turn is what throws a narrow claw off line. */
inline constexpr double CREEP_FORWARD_SCALE = 0.35;
inline constexpr double CREEP_TURN_SCALE = 0.25;

/* Lift-aware scaling. At the bottom of travel the drive runs at full command;
 * at the top it is multiplied by LIFT_MIN_DRIVE_SCALE, ramped linearly in
 * between. Tipping and stack-knocking come mostly from acceleration, so the
 * slew limit tightens with height too. */
inline constexpr double LIFT_MIN_DRIVE_SCALE = 0.45;
inline constexpr int DRIVE_SLEW_LOW = 18;   // max power change per 10 ms, lift down
inline constexpr int DRIVE_SLEW_HIGH = 7;   // max power change per 10 ms, lift up

/* Heading hold (needs the IMU). When you let go of the turn stick the robot
 * latches its current heading and corrects drift while you drive forward, so
 * a line-up made at distance survives the approach. */
inline constexpr double HEADING_KP = 2.6;
inline constexpr double HEADING_KD = 14.0;
inline constexpr int HEADING_MAX_CORRECTION = 35;   // power units
inline constexpr std::uint32_t HEADING_SETTLE_MS = 180;  // wait after a turn before latching
inline constexpr int HEADING_MIN_FORWARD = 12;      // don't correct while standing still

/* =========================================================================
 * AUTONOMOUS -- LemLib chassis
 *
 * No tracking wheels: LemLib estimates position from the drive motor
 * encoders plus the IMU. Fine for autons, but a robot that gets pushed
 * sideways will lose track of where it is.
 * ========================================================================= */

inline constexpr float CHASSIS_TRACK_WIDTH = 11.38; // 28.9 cm -- inches, centre of left
                                                    //     wheels to centre of right wheels
inline constexpr float CHASSIS_WHEEL_DIAMETER = 3.25;  // new 3.25" omnis
inline constexpr float CHASSIS_WHEEL_RPM = 360;        // blue cartridge, 36:60
inline constexpr float CHASSIS_HORIZONTAL_DRIFT = 2;   // 2 = all omnis, 8 = traction centre wheels

/* PID gains. Tune with the "Tune" autons on the brain's AUTON page:
 *
 *   1. kD = 0. Raise kP until the robot oscillates around the target.
 *   2. Raise kD until the oscillation stops.
 *   3. Repeat: a bit more kP, a bit more kD, until it is fast and settles.
 *   Leave kI at 0 unless it consistently stops just short.
 *
 * Tune ANGULAR first: a robot that cannot turn straight also cannot drive
 * straight. */
inline constexpr float LINEAR_KP = 10, LINEAR_KI = 0, LINEAR_KD = 30;
inline constexpr float ANGULAR_KP = 3, ANGULAR_KI = 0, ANGULAR_KD = 18;

/* =========================================================================
 * LIFT
 * ========================================================================= */

/* Your own note in main.cpp: the gears skipped teeth at 127. Every lift
 * command -- manual, hold and preset alike -- is clamped to this. */
inline constexpr int LIFT_MAX_POWER = 102;          // 127 * 0.80
inline constexpr int LIFT_SLEW = 10;                // max power change per 10 ms

/* Gearing between the sensor and the arm. Height is tracked in ARM DEGREES. */
inline constexpr double LIFT_MOTOR_DEG_PER_ARM_DEG = 7.0;  // <<< SET THIS (e.g. 84:12 = 7.0)
inline constexpr double LIFT_ROT_DEG_PER_ARM_DEG = 1.0;    // 1.0 if sensor is on the pivot

/* Hold / move gains, in arm degrees. */
inline constexpr double LIFT_KP = 3.2;
inline constexpr double LIFT_KD = 9.0;
inline constexpr double LIFT_KF = 8.0;      // constant anti-sag term; raise until it stops drooping
inline constexpr double LIFT_TOLERANCE = 1.5;   // arm degrees considered "arrived"
inline constexpr int LIFT_SETTLE_TICKS = 8;     // ticks inside tolerance before a preset finishes

/* Travel limits in arm degrees, measured from the hard stop (= 0). */
inline constexpr double LIFT_MIN_DEG = 0.0;
inline constexpr double LIFT_MAX_DEG = 95.0;    // <<< SET THIS

/* Stall protection. Commanded hard but not moving means the gears are
 * skipping or something is jammed; back off rather than chew the teeth. */
inline constexpr int LIFT_STALL_POWER = 45;         // only watch above this command
inline constexpr double LIFT_STALL_VELOCITY = 5.0;  // rpm below which it counts as stopped
inline constexpr std::uint32_t LIFT_STALL_MS = 350;

/* =========================================================================
 * STACK PRESETS -- arm degrees, indexed by how many cones are already stacked
 *
 * These are placeholders. Do not try to calculate them: drive the lift to the
 * right height by hand, then hold Y to save it to the current cone count.
 * Saved values live on the SD card and survive reboots and re-flashes.
 * ========================================================================= */
inline constexpr int MAX_CONES = 8;
inline constexpr double STACK_HEIGHTS[MAX_CONES] = {
    6.0,   // 0 cones down -- placing onto the floor
    14.0,  // 1
    22.0,  // 2
    30.0,  // 3
    38.0,  // 4
    46.0,  // 5
    54.0,  // 6
    62.0,  // 7
};

inline constexpr double GROUND_HEIGHT = 0.0;
inline constexpr double TRAVEL_HEIGHT = 12.0;   // stowed, safe to drive fast

/* Bump the cone counter automatically when the claw is opened above this
 * height -- i.e. when you have just placed one. Set false if you would rather
 * drive the counter yourself with the D-pad. */
inline constexpr bool AUTO_INCREMENT_ON_RELEASE = true;
inline constexpr double RELEASE_COUNTS_ABOVE_DEG = 10.0;

/* =========================================================================
 * AUTO-CLAMP (Distance sensor in the claw throat)
 * ========================================================================= */

/* A cone is "seated" when the sensor reads inside this window. The V5 Distance
 * sensor is unreliable below ~20 mm, so mount it at the back of the jaw
 * looking outward -- a gripped cone should sit near the middle of this band,
 * never pressed against the lens. */
inline constexpr int CLAMP_MIN_MM = 25;
inline constexpr int CLAMP_SEAT_MM = 80;        // grab at 80 mm or closer
inline constexpr int CLAMP_MIN_CONFIDENCE = 40;  // 0-63; rejects noisy reads
inline constexpr int CLAMP_MIN_OBJECT_SIZE = 50; // rejects specks and far walls

/* Grab ONLY when the sensor's size estimate reads -1. On this robot the size
 * reads ~70 while a cone approaches and flips to -1 once it is in the jaw, so
 * -1 doubles as a "seated" signal. The distance window above still applies.
 * false = the size-based check above instead. */
inline constexpr bool CLAMP_ONLY_ON_UNKNOWN_SIZE = true;
inline constexpr int CLAMP_DEBOUNCE_TICKS = 2;   // consecutive good reads (10 ms each)
inline constexpr std::uint32_t CLAMP_COOLDOWN_MS = 600;  // after a grab, before re-arming

/* ---- fast approaches ----------------------------------------------------
 * From "cone in range" to "jaw shut" takes ~150 ms, mostly the piston. At
 * full speed the robot covers ~23 cm in that time, so the cone used to blow
 * straight through the 80 mm window and get knocked over. Two fixes: */

/* 1. Fire early. Predict where the cone will be CLAMP_LEAD_MS from now, from
 *    how fast the distance is shrinking, and grab when THAT is inside the
 *    window. Approaching slowly, the prediction is just the reading.
 *      claw shuts in front of the cone / pushes it  -> lower this
 *      cone still hits the back before the jaw shuts -> raise this
 *    The LOG line "grabbed at N mm, closing at V mm/s" helps judge it.
 *    0 turns prediction off. */
inline constexpr int CLAMP_LEAD_MS = 120;
inline constexpr int CLAMP_LEAD_MAX_MM = 200;   // never fire on a prediction from farther out

/* 2. Auto-slow. While auto-clamp is armed, the claw is open and something is
 *    within APPROACH_SLOW_MM, forward drive power is capped. Reversing and
 *    turning are untouched. Assumes the claw faces the FRONT of the robot.
 *    Driver control only: autons set their own speed with {.maxSpeed = ...}. */
inline constexpr bool ENABLE_APPROACH_SLOWDOWN = true;
inline constexpr int APPROACH_SLOW_MM = 300;
inline constexpr int APPROACH_MAX_POWER = 60;   // out of 127

/* Auto-clamp starts armed at the beginning of opcontrol. */
inline constexpr bool CLAMP_ARMED_AT_START = true;


/* =========================================================================
 * STACK MACRO -- the dunk
 *
 * Press B once to raise to the preset for the current cone count. Drive over
 * the stack. Press B again to dunk, release and clear.
 *
 * The release fires on LOAD TRANSFER, not on a timer: while the lift lowers
 * at a constant rate it is carrying the cone's weight, and the instant the
 * cone's base lands on the stack that weight goes away and the lift motors'
 * torque drops. That moment is the right time to open the claw, and it finds
 * itself at whatever height the stack actually is.
 *
 * READ THIS BEFORE TRUSTING IT: a cone is light compared to a DR4B, so the
 * torque drop is small and V5 torque readings are noisy. Watch the "Lift Nm"
 * row on the DIAG page during a slow descent with a cone and see how big the
 * step actually is, then set DUNK_TORQUE_DROP_NM to about half of it. If the
 * signal is buried in noise on your robot, set DUNK_USE_TORQUE_TRIGGER false
 * and the macro falls back to the fixed-depth release, which is less adaptive
 * but perfectly usable.
 * ========================================================================= */

inline constexpr double DUNK_CLEARANCE_DEG = 6.0;      // raise this far above the preset
inline constexpr double DUNK_READY_TOLERANCE_DEG = 4.0; // "close enough to dunk from"
inline constexpr double DUNK_DESCENT_DEG_PER_SEC = 18.0; // slow enough to react to
inline constexpr double DUNK_MAX_DEPTH_DEG = 12.0;     // floor guard: release by here regardless

inline constexpr bool DUNK_USE_TORQUE_TRIGGER = true;
inline constexpr int DUNK_TORQUE_BASELINE_TICKS = 10;  // 100 ms of "carrying it" before judging
inline constexpr double DUNK_TORQUE_DROP_NM = 0.08;    // <<< MEASURE THIS on the DIAG page
inline constexpr int DUNK_TORQUE_CONFIRM_TICKS = 3;

/* Pistons are not instant. Retracting before the claw has physically opened
 * is the classic way to drag a finished stack over. */
inline constexpr std::uint32_t DUNK_PISTON_MS = 120;
inline constexpr double DUNK_CLEAR_DEG = 10.0;         // lift the jaw clear before backing off
inline constexpr std::uint32_t DUNK_TIMEOUT_MS = 2500; // nothing happened -> abort, do NOT drop

/* =========================================================================
 * ANTI-TIP
 *
 * Only armed with the lift up, because a DR4B at the bottom cannot tip you.
 * The magnitude check uses fabs(), so the IMU's pitch SIGN does not matter --
 * but which AXIS does: if your IMU is mounted on its side, what you want is
 * get_roll(). Track "Tilt" on the DIAG page, tip the robot by hand, and make
 * sure the number moves. If it does not, flip ANTITIP_USE_ROLL.
 * ========================================================================= */

inline constexpr bool ANTITIP_USE_ROLL = false;        // <<< VERIFY on the DIAG page
inline constexpr double ANTITIP_ARMED_ABOVE_FRACTION = 0.25;  // lift height 0..1
inline constexpr double ANTITIP_WARN_DEG = 8.0;
inline constexpr double ANTITIP_CRITICAL_DEG = 14.0;
inline constexpr double ANTITIP_DRIVE_SCALE = 0.30;    // drive authority while tipping
inline constexpr int ANTITIP_SLEW = 4;                 // and almost no acceleration

/* At the critical angle, put the lift down. This is the only thing that
 * actually changes the centre of mass, and it will drop a held cone -- which
 * is a cheaper outcome than going over. */
inline constexpr bool ANTITIP_AUTO_STOW = true;

/* =========================================================================
 * MATCH TIMER HAPTICS
 *
 * Timed from the start of opcontrol. Three distinguishable patterns so the
 * driver can tell them apart without looking.
 * ========================================================================= */

inline constexpr std::uint32_t DRIVER_PERIOD_MS = 105000;  // 1:45 head-to-head; 60000 for skills

inline constexpr std::uint32_t MATCH_ALERT_MS[3] = {35000, 15000, 5000};
inline constexpr const char* MATCH_ALERT_PATTERN[3] = {"-", "--", "..."};

}  // namespace robot::config
