/** Drive, lift and claw for 936B. Tunables live in robot/config.hpp. */
#pragma once

#include <cstdint>

#include "lemlib/api.hpp"

namespace robot {

/** LemLib chassis for autonomous. Don't run motions during driver control. */
extern lemlib::Chassis chassis;

/** Devices, control tasks and UI telemetry. Call once from initialize(), with
 *  the lift down (it zeroes there) and the robot still (IMU calibrates). */
void init();

/** One tick of driver control; call every ~10 ms from opcontrol(). */
void driver_step();

namespace drive {
/** Arcade with deadband, creep and auto-clamp approach slowdown. */
void arcade(int forward, int turn, bool creep);
}  // namespace drive

namespace lift {
/** Arm degrees above the hard stop. */
double height();
/** -127..127, 0 = stop (brake-hold keeps it there). Cancels raise_by(). */
void manual(int power);
/** After `delay_ms`, raise `arm_degrees` above the current height. */
void raise_by(double arm_degrees, std::uint32_t delay_ms);
/** Declare the current position to be zero. */
void tare();
}  // namespace lift

namespace claw {
bool closed();
void close();
void open();
void toggle();
int grabs();
bool armed();
void set_armed(bool on);
/** Last distance reading in mm, -1 with nothing in range. */
int distance_mm();
bool cone_seated();
/** mm/s, positive while something gets nearer. */
int closing_speed();
}  // namespace claw

}  // namespace robot
