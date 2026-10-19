/**
 * @file src/robot/lift.cpp
 *
 * DR4B control: manual driving, position hold, cone-count presets and stall
 * protection. Heights are ARM DEGREES above the hard stop, measured by the
 * lift motors' built-in encoders.
 *
 * Runs in its own 10 ms task. move_to() returns immediately.
 */
#include <atomic>
#include <cmath>

#include "internal.hpp"
#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot::lift {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {

enum class Mode { Manual, Hold, Preset };

std::atomic<Mode> g_mode{Mode::Hold};
std::atomic<int> g_manual_power{0};
std::atomic<double> g_target{0.0};
std::atomic<double> g_height{0.0};
std::atomic<bool> g_at_target{false};
std::atomic<bool> g_stalled{false};
std::atomic<int> g_cones{0};

std::atomic<double> g_offset{0.0};  // raw units at the hard stop
double g_presets[cfg::MAX_CONES] = {};
int g_output = 0;                   // last power sent, for slew limiting

/** Raw encoder reading converted to arm degrees, before the zero offset. */
double read_raw_arm_degrees() {
    return lift_mg.get_position() / cfg::LIFT_MOTOR_DEG_PER_ARM_DEG;
}

/** Hottest absolute velocity in the group, in rpm. */
double lift_velocity() { return std::fabs(lift_mg.get_actual_velocity()); }

void control_task(void*) {
    double last_error = 0.0;
    int settle_ticks = 0;
    std::uint32_t stall_since = 0;

    while (true) {
        const double h = read_raw_arm_degrees() - g_offset.load();
        g_height.store(h);

        /* Raw passthrough. This is the original main.cpp behaviour: the
         * buttons drive the motors directly and brake-hold does the holding.
         * Height is still tracked so the DIAG page works, but nothing here
         * depends on it being correct. */
        if (!cfg::ENABLE_LIFT_HOLD) {
            lift_mg.move(clampi(g_manual_power.load(), -cfg::LIFT_MAX_POWER,
                                cfg::LIFT_MAX_POWER));
            pros::delay(10);
            continue;
        }

        const Mode mode = g_mode.load();
        int command = 0;

        if (mode == Mode::Manual) {
            command = g_manual_power.load();
            last_error = 0.0;
            settle_ticks = 0;
            g_at_target.store(false);

            // Refuse to drive past the ends of travel, but always allow the
            // direction that returns toward the legal range -- otherwise a
            // miscalibrated zero can trap the lift against a limit.
            if (cfg::ENABLE_LIFT_LIMITS) {
                if (h >= cfg::LIFT_MAX_DEG && command > 0) command = 0;
                if (h <= cfg::LIFT_MIN_DEG && command < 0) command = 0;
            }

        } else {
            const double target = g_target.load();
            const double error = target - h;
            const double derivative = error - last_error;
            last_error = error;

            const double pd = cfg::LIFT_KP * error + cfg::LIFT_KD * derivative;

            // kF is a constant anti-sag term. It always pushes up, because
            // gravity always pulls down -- a DR4B that is band-balanced needs
            // very little, one that is not needs a lot.
            command = static_cast<int>(std::lround(pd + cfg::LIFT_KF));

            if (std::fabs(error) <= cfg::LIFT_TOLERANCE) {
                if (settle_ticks < cfg::LIFT_SETTLE_TICKS) ++settle_ticks;
            } else {
                settle_ticks = 0;
            }

            const bool settled = settle_ticks >= cfg::LIFT_SETTLE_TICKS;
            g_at_target.store(settled);

            // A finished preset becomes a hold at the same height, so the arm
            // does not sag once the move completes.
            if (mode == Mode::Preset && settled) g_mode.store(Mode::Hold);
        }

        command = clampi(command, -cfg::LIFT_MAX_POWER, cfg::LIFT_MAX_POWER);

        /* --- stall protection -------------------------------------------
         * Commanded hard but not moving means the gears are skipping or the
         * arm is jammed. Your main.cpp already notes these gears skipped at
         * full power; chewing them further helps nobody, so drop to the
         * anti-sag term and let the driver react. */
        if (cfg::ENABLE_STALL_GUARD && std::abs(command) > cfg::LIFT_STALL_POWER &&
            lift_velocity() < cfg::LIFT_STALL_VELOCITY) {
            if (stall_since == 0) {
                stall_since = pros::millis();
            } else if (pros::millis() - stall_since > cfg::LIFT_STALL_MS) {
                if (!g_stalled.exchange(true)) {
                    ui::error("Lift stalled at %.1f deg -- backing off", g_height.load());
                }
                command = clampi(static_cast<int>(cfg::LIFT_KF), 0, cfg::LIFT_MAX_POWER);
            }
        } else {
            stall_since = 0;
            g_stalled.store(false);
        }

        // Slew the output rather than stepping it. A PD controller's first
        // tick after a big setpoint change is a full-power torque spike, which
        // is exactly what skips teeth.
        g_output = slew(g_output, command, cfg::LIFT_SLEW);
        lift_mg.move(g_output);

        pros::delay(10);
    }
}

}  // namespace

/* ======================================================================= */

double height() { return g_height.load(); }

double height_fraction() {
    const double span = cfg::LIFT_MAX_DEG - cfg::LIFT_MIN_DEG;
    if (span <= 0.0) return 0.0;
    return clampd((g_height.load() - cfg::LIFT_MIN_DEG) / span, 0.0, 1.0);
}

void manual(int power) {
    if (power == 0) {
        // Releasing the stick latches a hold at wherever the arm ended up,
        // rather than leaving it to brake-mode alone.
        if (g_mode.load() == Mode::Manual) {
            g_target.store(clampd(g_height.load(), cfg::LIFT_MIN_DEG, cfg::LIFT_MAX_DEG));
            g_mode.store(Mode::Hold);
        }
        g_manual_power.store(0);
        return;
    }
    g_manual_power.store(clampi(power, -cfg::LIFT_MAX_POWER, cfg::LIFT_MAX_POWER));
    g_mode.store(Mode::Manual);
}

void move_to(double arm_degrees) {
    g_target.store(clampd(arm_degrees, cfg::LIFT_MIN_DEG, cfg::LIFT_MAX_DEG));
    g_at_target.store(false);
    g_mode.store(Mode::Preset);
}

bool at_target() { return g_at_target.load(); }
bool is_manual() { return g_mode.load() == Mode::Manual; }
bool stalled() { return g_stalled.load(); }

void tare() {
    g_offset.store(read_raw_arm_degrees());
    g_height.store(0.0);
    g_target.store(0.0);
    g_mode.store(Mode::Hold);
    ui::log("Lift zeroed");
}

/* --- cone counter ------------------------------------------------------ */

int cone_count() { return g_cones.load(); }

void set_cone_count(int n) {
    g_cones.store(clampi(n, 0, cfg::MAX_CONES - 1));
}

void bump_cone_count(int delta) { set_cone_count(g_cones.load() + delta); }

double preset_height() { return g_presets[g_cones.load()]; }

void go_to_preset() {
    const double h = preset_height();
    ui::log("Lift -> cone %d (%.1f deg)", g_cones.load(), h);
    move_to(h);
}

void teach_preset() {
    const int n = g_cones.load();
    g_presets[n] = clampd(g_height.load(), cfg::LIFT_MIN_DEG, cfg::LIFT_MAX_DEG);
    presets_save(g_presets, cfg::MAX_CONES);
    ui::log("Saved cone %d height = %.1f deg", n, g_presets[n]);
}

}  // namespace robot::lift

namespace robot::internal {

void lift_init() {
    // Seed the preset table from config, then let anything saved on the SD
    // card override it. Compiled-in values are the fallback for a robot with
    // no card, not the source of truth -- teach_preset() is.
    for (int i = 0; i < config::MAX_CONES; ++i) {
        robot::lift::g_presets[i] = config::STACK_HEIGHTS[i];
    }
    presets_load(robot::lift::g_presets, config::MAX_CONES);

    // Zero against wherever the arms are sitting at boot. They must be down
    // on the hard stop for this to mean anything.
    robot::lift::tare();

    pros::Task(robot::lift::control_task, nullptr, "lift");
}

}  // namespace robot::internal
