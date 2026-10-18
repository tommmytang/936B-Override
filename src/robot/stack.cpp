/**
 * @file src/robot/stack.cpp
 *
 * The one-button stack macro.
 *
 *   press 1 -- raise to the preset for the current cone count, then stop
 *   (driver drives over the stack)
 *   press 2 -- lower slowly, release the instant the cone's weight lands on
 *              the stack, wait for the piston, lift clear
 *
 * The release trigger is LOAD TRANSFER rather than a delay or a fixed height.
 * While the lift lowers it is carrying the cone; the moment the cone's base
 * touches down that weight transfers to the stack and the lift motors' torque
 * drops. Watching for that drop means the macro lands correctly on a stack
 * that is a little taller or shorter than the preset assumed, which a timer
 * can never do.
 *
 * A fixed-depth floor guard backs it up, so a missed torque event releases
 * slightly low instead of driving the arm into the stack. A timeout aborts
 * WITHOUT releasing -- if nothing is happening, dropping a cone from height
 * is the wrong guess.
 *
 * Runs in its own task. Any manual lift input aborts it.
 */
#include <atomic>
#include <cmath>

#include "internal.hpp"
#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot::stack {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {

enum class State { Idle, Raising, Dunking, Releasing, Clearing };

std::atomic<State> g_state{State::Idle};
std::atomic<bool> g_entered{false};  // set on every transition, consumed by the task
std::atomic<bool> g_abort{false};

const char* label(State s) {
    switch (s) {
        case State::Raising: return "RAISE";
        case State::Dunking: return "DUNK ";
        case State::Releasing: return "DROP ";
        case State::Clearing: return "CLEAR";
        default: return "     ";
    }
}

void enter(State s) {
    g_state.store(s);
    g_entered.store(true);
}

void finish(const char* why) {
    g_state.store(State::Idle);
    g_entered.store(false);
    // Latch a hold wherever the arm ended up rather than leaving the lift
    // chasing a setpoint the macro was mid-way through ramping.
    lift::move_to(lift::height());
    if (why != nullptr) ui::log("Stack: %s", why);
}

void task(void*) {
    double descent_target = 0.0;
    double descent_start = 0.0;
    double torque_ema = 0.0;
    double torque_baseline = 0.0;
    int tick = 0;
    int confirm = 0;
    std::uint32_t entered_at = 0;

    while (true) {
        const State state = g_state.load();

        if (state == State::Idle) {
            g_abort.store(false);
            pros::delay(20);
            continue;
        }

        // Manual input always wins. The driver can take the lift back at any
        // point in the sequence without reaching for a cancel button.
        if (g_abort.exchange(false) || lift::is_manual()) {
            finish("aborted");
            pros::delay(20);
            continue;
        }

        const bool first = g_entered.exchange(false);
        if (first) {
            entered_at = pros::millis();
            tick = 0;
            confirm = 0;
        }

        switch (state) {
            case State::Raising: {
                if (first) {
                    const double target = lift::preset_height() + cfg::DUNK_CLEARANCE_DEG;
                    ui::log("Stack: raising to %.1f for cone %d", target, lift::cone_count());
                    lift::move_to(target);
                }
                if (lift::at_target()) {
                    finish("at height -- press again to dunk");
                } else if (pros::millis() - entered_at > cfg::DUNK_TIMEOUT_MS * 2) {
                    finish("raise timed out");
                }
                break;
            }

            case State::Dunking: {
                if (first) {
                    descent_start = lift::height();
                    descent_target = descent_start;
                    torque_ema = std::fabs(lift_mg.get_torque());
                    torque_baseline = 0.0;
                    ui::log("Stack: dunking from %.1f", descent_start);
                }

                // Ramp the setpoint down instead of commanding a raw power, so
                // the descent is a constant rate the PD loop holds against the
                // arm's own weight.
                descent_target -= cfg::DUNK_DESCENT_DEG_PER_SEC * 0.01;
                lift::move_to(descent_target);

                const double torque = std::fabs(lift_mg.get_torque());
                torque_ema = 0.7 * torque_ema + 0.3 * torque;
                ++tick;

                bool release = false;

                if (cfg::DUNK_USE_TORQUE_TRIGGER) {
                    if (tick == cfg::DUNK_TORQUE_BASELINE_TICKS) {
                        // Settled into the descent; this is what carrying the
                        // cone costs.
                        torque_baseline = torque_ema;
                    } else if (tick > cfg::DUNK_TORQUE_BASELINE_TICKS &&
                               torque_baseline - torque_ema > cfg::DUNK_TORQUE_DROP_NM) {
                        if (++confirm >= cfg::DUNK_TORQUE_CONFIRM_TICKS) {
                            ui::log("Stack: contact (%.3f -> %.3f Nm)", torque_baseline,
                                    torque_ema);
                            release = true;
                        }
                    } else {
                        confirm = 0;
                    }
                }

                // Floor guard. Reached the bottom of the dunk window without
                // ever seeing contact -- release here rather than keep pushing.
                if (!release && descent_start - lift::height() >= cfg::DUNK_MAX_DEPTH_DEG) {
                    ui::warn("Stack: no contact felt, releasing at depth limit");
                    release = true;
                }

                if (release) {
                    enter(State::Releasing);
                } else if (pros::millis() - entered_at > cfg::DUNK_TIMEOUT_MS) {
                    // Deliberately does NOT open the claw: if the lift is not
                    // descending, something is jammed, and letting go of a cone
                    // at an unknown height is the worse guess.
                    finish("dunk timed out -- cone NOT released");
                }
                break;
            }

            case State::Releasing: {
                if (first) claw::open();
                // Let the solenoid actually move before anything retracts.
                // Backing out through a half-open claw is how stacks get
                // dragged over.
                if (pros::millis() - entered_at >= cfg::DUNK_PISTON_MS) {
                    enter(State::Clearing);
                }
                break;
            }

            case State::Clearing: {
                if (first) lift::move_to(lift::height() + cfg::DUNK_CLEAR_DEG);
                if (lift::at_target() || pros::millis() - entered_at > cfg::DUNK_TIMEOUT_MS) {
                    finish("done");
                }
                break;
            }

            default: break;
        }

        pros::delay(10);
    }
}

}  // namespace

/* ======================================================================= */

bool active() { return g_state.load() != State::Idle; }

const char* step() { return label(g_state.load()); }

void abort() {
    if (active()) g_abort.store(true);
}

void begin() {
    if (!cfg::ENABLE_STACK_MACRO) return;
    if (active()) {
        abort();
        return;
    }

    const double ready = lift::preset_height() + cfg::DUNK_CLEARANCE_DEG;
    if (std::fabs(lift::height() - ready) <= cfg::DUNK_READY_TOLERANCE_DEG) {
        enter(State::Dunking);
    } else {
        enter(State::Raising);
    }
}

}  // namespace robot::stack

namespace robot::internal {

void stack_init() {
    if (!config::ENABLE_STACK_MACRO) return;
    pros::Task(robot::stack::task, nullptr, "stack");
}

}  // namespace robot::internal
