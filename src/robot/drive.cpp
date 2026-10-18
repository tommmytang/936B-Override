/**
 * @file src/robot/drive.cpp
 *
 * Arcade drive with four layers stacked on top of the raw sticks:
 *
 *   1. creep scaling     -- precision speed while R2 is held
 *   2. lift-aware scaling -- slower and gentler the higher the DR4B is
 *   3. heading hold      -- IMU keeps you straight once you stop turning
 *   4. slew limiting     -- caps acceleration, which is what tips the robot
 *                           and shakes stacks over
 *
 * Layer 3 is the one that fixes line-ups: three motors per side never match,
 * so an uncorrected drive curves. Aim from a distance, then drive in straight.
 */
#include <atomic>
#include <cmath>

#include "internal.hpp"
#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot::drive {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {

double g_heading_target = 0.0;
bool g_heading_locked = false;
std::uint32_t g_turn_released_at = 0;
double g_last_heading_error = 0.0;

int g_left_out = 0;
int g_right_out = 0;

std::atomic<bool> g_tipping{false};

/** Zero out stick noise around centre. */
int deadband(int v) { return std::abs(v) < cfg::JOY_DEADBAND ? 0 : v; }

}  // namespace

bool imu_ready() { return imu_present && !imu.is_calibrating(); }

double tilt() {
    if (!imu_ready()) return 0.0;
    // fabs() deliberately: the magnitude is what matters and it makes the
    // IMU's mounting SIGN irrelevant. Which axis is right is still on you --
    // see ANTITIP_USE_ROLL in config.hpp.
    return std::fabs(cfg::ANTITIP_USE_ROLL ? imu.get_roll() : imu.get_pitch());
}

bool tipping() { return g_tipping.load(); }

namespace {
/** Watches tilt and puts the lift down if it gets bad enough. Lowering the
 *  arms is the only thing that actually moves the centre of mass; it will
 *  drop a held cone, which is cheaper than going over. */
void antitip_task(void*) {
    bool stowed = false;
    while (true) {
        const bool armed = cfg::ENABLE_ANTITIP && imu_ready() &&
                           lift::height_fraction() > cfg::ANTITIP_ARMED_ABOVE_FRACTION;
        const double t = armed ? tilt() : 0.0;

        g_tipping.store(armed && t > cfg::ANTITIP_WARN_DEG);

        if (armed && t > cfg::ANTITIP_CRITICAL_DEG) {
            if (!stowed) {
                stowed = true;
                ui::error("ANTI-TIP: %.0f deg -- stowing lift", t);
                if (g_controller != nullptr) g_controller->rumble("---");
                if (cfg::ANTITIP_AUTO_STOW) {
                    stack::abort();
                    lift::move_to(cfg::TRAVEL_HEIGHT);
                }
            }
        } else if (t < cfg::ANTITIP_WARN_DEG) {
            // Hysteresis: do not re-arm the stow until the robot is properly
            // back down, or a robot rocking on the warn threshold stows over
            // and over.
            stowed = false;
        }

        pros::delay(20);
    }
}
}  // namespace

double heading() { return g_heading_locked ? g_heading_target : 0.0; }

void release_heading() {
    g_heading_locked = false;
    g_turn_released_at = pros::millis();
}

void arcade(int forward_raw, int turn_raw, bool creep) {
    double forward = deadband(forward_raw);
    double turn = deadband(turn_raw);

    /* --- 1. creep -------------------------------------------------------- */
    if (creep && cfg::ENABLE_CREEP) {
        forward *= cfg::CREEP_FORWARD_SCALE;
        turn *= cfg::CREEP_TURN_SCALE;
    }

    /* --- 2. lift-aware scaling ------------------------------------------- */
    const double lift_fraction = lift::height_fraction();
    if (cfg::ENABLE_LIFT_SCALING) {
        const double lift_scale =
            1.0 - lift_fraction * (1.0 - cfg::LIFT_MIN_DRIVE_SCALE);
        forward *= lift_scale;
        turn *= lift_scale;
    }

    /* --- 2b. approach slowdown --------------------------------------------
     * Auto-clamp needs ~150 ms to shut the jaw; at full speed that is ~23 cm
     * of travel, and the cone gets knocked over before the claw closes. So
     * while the claw is open and waiting for a cone, and something is close
     * ahead, cap forward speed. Reversing away is never limited. */
    if (cfg::ENABLE_APPROACH_SLOWDOWN && cfg::ENABLE_AUTO_CLAMP && claw::armed() &&
        !claw::closed()) {
        const int mm = claw::distance_mm();
        if (mm > 0 && mm <= cfg::APPROACH_SLOW_MM && forward > cfg::APPROACH_MAX_POWER) {
            forward = cfg::APPROACH_MAX_POWER;
        }
    }

    /* --- 3. heading hold -------------------------------------------------
     * Latch a heading once the driver has been off the turn stick long enough
     * to have actually stopped rotating, then correct drift while moving. */
    const bool driver_turning = std::fabs(turn) > 0.0;

    if (!cfg::ENABLE_HEADING_HOLD) {
        // Heading hold off: the turn stick is the only thing steering.
    } else if (driver_turning) {
        // The driver is in charge of heading; drop the lock and restart the
        // settle timer so we do not latch a heading mid-rotation.
        g_heading_locked = false;
        g_turn_released_at = pros::millis();
        g_last_heading_error = 0.0;
    } else if (imu_ready()) {
        if (!g_heading_locked &&
            pros::millis() - g_turn_released_at > cfg::HEADING_SETTLE_MS) {
            // get_rotation() is continuous and never wraps at 360, so the
            // error below is plain subtraction with no angle wrapping.
            g_heading_target = imu.get_rotation();
            g_heading_locked = true;
            g_last_heading_error = 0.0;
        }

        // Only correct while actually driving. Correcting at a standstill
        // makes the robot creep sideways hunting for a heading it cannot
        // reach from rest.
        if (g_heading_locked && std::fabs(forward) >= cfg::HEADING_MIN_FORWARD) {
            const double error = g_heading_target - imu.get_rotation();
            const double derivative = error - g_last_heading_error;
            g_last_heading_error = error;

            const double correction =
                cfg::HEADING_KP * error + cfg::HEADING_KD * derivative;
            turn += clampd(correction, -cfg::HEADING_MAX_CORRECTION,
                           cfg::HEADING_MAX_CORRECTION);
        }
    }

    /* --- 3b. anti-tip ------------------------------------------------------
     * Clamp drive authority hard while tilted. The matching stow lives in
     * antitip_task(); this half just stops the driver from making it worse. */
    const bool tip = cfg::ENABLE_ANTITIP && g_tipping.load();
    if (tip) {
        forward *= cfg::ANTITIP_DRIVE_SCALE;
        turn *= cfg::ANTITIP_DRIVE_SCALE;
    }

    /* --- 4. mix and slew -------------------------------------------------- */
    int left_target = static_cast<int>(std::lround(forward + turn));
    int right_target = static_cast<int>(std::lround(forward - turn));

    left_target = clampi(left_target, -cfg::DRIVE_MAX_POWER, cfg::DRIVE_MAX_POWER);
    right_target = clampi(right_target, -cfg::DRIVE_MAX_POWER, cfg::DRIVE_MAX_POWER);

    // Tighter acceleration limit with the lift up. Top speed matters less for
    // tipping than how fast you get there.
    int slew_limit = cfg::DRIVE_MAX_POWER * 2;  // effectively unlimited
    if (cfg::ENABLE_DRIVE_SLEW) {
        slew_limit = static_cast<int>(std::lround(
            cfg::DRIVE_SLEW_LOW +
            lift_fraction * (cfg::DRIVE_SLEW_HIGH - cfg::DRIVE_SLEW_LOW)));
    }
    if (tip) slew_limit = cfg::ANTITIP_SLEW;

    g_left_out = slew(g_left_out, left_target, slew_limit);
    g_right_out = slew(g_right_out, right_target, slew_limit);

    left_mg.move(g_left_out);
    right_mg.move(g_right_out);
}

void stop() {
    g_left_out = 0;
    g_right_out = 0;
    left_mg.move(0);
    right_mg.move(0);
    release_heading();
}

}  // namespace robot::drive

namespace robot::internal {

void drive_init() {
    robot::drive::release_heading();
    if (config::ENABLE_ANTITIP) {
        pros::Task(robot::drive::antitip_task, nullptr, "anti-tip");
    }
}

}  // namespace robot::internal
