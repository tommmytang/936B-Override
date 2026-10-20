/** Every tunable number: ports, speeds, gains, sensor thresholds. */
#pragma once

#include <cstdint>

namespace robot::config {

/* --- Ports (negative = reversed) ----------------------------------------- */
inline constexpr std::int8_t DRIVE_L1 = -1, DRIVE_L2 = -2, DRIVE_L3 = -3;
inline constexpr std::int8_t DRIVE_R1 = 6, DRIVE_R2 = 7, DRIVE_R3 = 8;
inline constexpr std::int8_t LIFT_1 = 9, LIFT_2 = -10;
inline constexpr std::int8_t AUX_1 = -11, AUX_2 = 12;  // spin opposite ways
inline constexpr std::uint8_t CLAW_ADI_PORT = 1;       // 'A'
inline constexpr std::uint8_t IMU_PORT = 4;
inline constexpr std::uint8_t DISTANCE_PORT = 5;       // in the claw throat

/* Controller
 *   Left Y / Right X  arcade drive       R2 (hold)   creep
 *   L1 / R1           lift up / down     L2 (tap)    aux motors on/off
 *   A                 claw toggle        D-right     arm/disarm auto-clamp
 *   X (hold 1 s)      re-zero lift at its hard stop */
inline constexpr std::uint32_t HOLD_TO_CONFIRM_MS = 1000;

/* --- Drive --------------------------------------------------------------- */
inline constexpr int JOY_DEADBAND = 8;
inline constexpr double CREEP_FORWARD_SCALE = 0.35;
inline constexpr double CREEP_TURN_SCALE = 0.25;

/* --- Aux pair: speed-controlled, percent of full speed -------------------- */
inline constexpr int AUX_SPEED_PCT = 25;

/* --- LemLib chassis (no tracking wheels) --------------------------------- */
inline constexpr float CHASSIS_TRACK_WIDTH = 11.38;    // inches
inline constexpr float CHASSIS_WHEEL_DIAMETER = 3.25;
inline constexpr float CHASSIS_WHEEL_RPM = 360;
inline constexpr float CHASSIS_HORIZONTAL_DRIFT = 2;   // 2 = all omnis
// Tune with the "Tune turn" / "Tune drive" autons. Angular first.
inline constexpr float LINEAR_KP = 10, LINEAR_KI = 0, LINEAR_KD = 30;
inline constexpr float ANGULAR_KP = 3, ANGULAR_KI = 0, ANGULAR_KD = 18;

/* --- Lift ---------------------------------------------------------------- */
inline constexpr int LIFT_MAX_POWER = 102;  // gears skip at 127
inline constexpr double LIFT_MOTOR_DEG_PER_ARM_DEG = 7.0;

/* After a grab near the bottom, raise the lift so the cone clears the floor. */
inline constexpr double GRAB_LIFT_BELOW_DEG = 8.0;
inline constexpr double GRAB_LIFT_RAISE_DEG = 30.0;
inline constexpr std::uint32_t GRAB_LIFT_DELAY_MS = 150;  // let the piston close
inline constexpr int GRAB_LIFT_POWER = 127;
inline constexpr std::uint32_t GRAB_LIFT_TIMEOUT_MS = 600;

/* --- Claw ---------------------------------------------------------------- */
inline constexpr bool CLAW_EXTEND_TO_CLOSE = false;  // closed with solenoid off

/* Auto-clamp: fire when the distance sensor sees a seated cone. */
inline constexpr bool CLAMP_ARMED_AT_START = true;
inline constexpr int CLAMP_MIN_MM = 25;
inline constexpr int CLAMP_SEAT_MM = 80;
inline constexpr int CLAMP_MIN_CONFIDENCE = 40;      // only applies beyond 200 mm
inline constexpr int CLAMP_MIN_OBJECT_SIZE = 50;
inline constexpr bool CLAMP_ONLY_ON_UNKNOWN_SIZE = true;  // size reads -1 once seated
inline constexpr int CLAMP_DEBOUNCE_TICKS = 2;
inline constexpr std::uint32_t CLAMP_COOLDOWN_MS = 600;
// Fire early by predicting the gap CLAMP_LEAD_MS ahead. 0 = off.
inline constexpr int CLAMP_LEAD_MS = 120;
inline constexpr int CLAMP_LEAD_MAX_MM = 200;
// Cap forward power while armed, open and a cone is this close.
inline constexpr int APPROACH_SLOW_MM = 300;
inline constexpr int APPROACH_MAX_POWER = 60;

/* --- Controller screen --------------------------------------------------- */
inline constexpr std::uint32_t DRIVER_PERIOD_MS = 105000;
inline constexpr double HOT_MOTOR_C = 55.0;

}  // namespace robot::config
