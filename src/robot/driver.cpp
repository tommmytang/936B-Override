/**
 * @file src/robot/driver.cpp
 *
 * init() and the per-tick driver control step, plus the controller-screen
 * readout.
 *
 * driver_step() never blocks and never loops. Everything it starts -- a
 * preset move, an auto-clamp -- runs in a task, so the driver keeps full
 * control of the robot while a macro is in progress, and any manual input
 * cancels the macro outright.
 */
#include <atomic>
#include <cmath>

#include "internal.hpp"
#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {

/* Hold-to-confirm state for the two destructive-ish bindings (X re-zero,
 * Y teach). A tap must not be enough -- both of them silently invalidate
 * heights the driver is relying on. */
std::uint32_t g_x_down_at = 0;
std::uint32_t g_y_down_at = 0;
bool g_x_fired = false;
bool g_y_fired = false;

pros::Controller* g_master = nullptr;

/* Match timer. opcontrol is killed and restarted between the autonomous and
 * driver periods, so rather than needing a hook in main.cpp we detect the
 * restart: driver_step() runs every 10 ms, so a gap of half a second means
 * the period just began. */
std::atomic<std::uint32_t> g_period_start{0};

/* Raw R2 state, shown on the controller screen regardless of whether creep is
 * enabled -- so "the button is dead" and "the feature is off" can be told
 * apart from the driver's seat. */
std::atomic<bool> g_r2_held{false};
std::uint32_t g_last_step_ms = 0;
bool g_alert_fired[3] = {false, false, false};

/* Aux motor toggle (L2). Reset to off at the start of every driver period so
 * the pair never starts spinning on its own. */
bool g_aux_on = false;

std::uint32_t match_remaining_ms() {
    const std::uint32_t start = g_period_start.load();
    if (start == 0) return cfg::DRIVER_PERIOD_MS;
    const std::uint32_t elapsed = pros::millis() - start;
    return elapsed >= cfg::DRIVER_PERIOD_MS ? 0 : cfg::DRIVER_PERIOD_MS - elapsed;
}

/** Rumbles once at each configured mark. Patterns differ so the driver can
 *  tell 35s from 5s without looking down. */
void service_match_alerts(pros::Controller& master) {
    if (!cfg::ENABLE_MATCH_ALERTS) return;
    const std::uint32_t remaining = match_remaining_ms();
    for (int i = 0; i < 3; ++i) {
        if (!g_alert_fired[i] && remaining <= cfg::MATCH_ALERT_MS[i]) {
            g_alert_fired[i] = true;
            master.rumble(cfg::MATCH_ALERT_PATTERN[i]);
            ui::log("%lu seconds left", static_cast<unsigned long>(cfg::MATCH_ALERT_MS[i] / 1000));
        }
    }
}

/** Returns true once, when the button has been held long enough. */
bool held_long_enough(bool pressed, std::uint32_t& down_at, bool& fired) {
    if (!pressed) {
        down_at = 0;
        fired = false;
        return false;
    }
    if (down_at == 0) down_at = pros::millis();
    if (!fired && pros::millis() - down_at >= cfg::HOLD_TO_CONFIRM_MS) {
        fired = true;
        return true;
    }
    return false;
}

/** The V5 controller screen rate-limits hard, so write one line per tick and
 *  let each line refresh every ~200 ms. Writing faster just drops lines. */
void controller_screen_task(void*) {
    int line = 0;
    while (true) {
        if (g_master != nullptr) {
            switch (line) {
                case 0:
                    g_master->print(0, 0, "Cone %d  Lift %3.0f  ", lift::cone_count(),
                                    lift::height());
                    break;
                case 1:
                    if (stack::active()) {
                        g_master->print(1, 0, "STACK %s      ", stack::step());
                    } else {
                        const int mm = claw::distance_mm();
                        g_master->print(1, 0, "%s %s %s   ",
                                        claw::closed() ? "GRIP" : "open",
                                        claw::armed() ? "auto" : "man ",
                                        mm >= 0 ? "ok" : "--");
                    }
                    break;
                default:
                    if (g_r2_held.load()) {
                        g_master->print(2, 0, cfg::ENABLE_CREEP ? "CREEP           "
                                                                : "R2 (creep OFF)  ");
                    } else if (drive::tipping()) {
                        g_master->print(2, 0, "!! TIP %2.0f deg !!  ", drive::tilt());
                    } else {
                        g_master->print(2, 0, "T-%02lu            ",
                                        static_cast<unsigned long>(match_remaining_ms() / 1000));
                    }
                    break;
            }
        }
        line = (line + 1) % 3;
        pros::delay(100);
    }
}

}  // namespace

/* ======================================================================= */

void init() {
    hardware_init();
    lift_init();
    claw_init();
    drive_init();
    stack_init();

    /* --- diagnostics page ------------------------------------------------ */
    ui::track_motor_group("Drive L", {cfg::DRIVE_L1, cfg::DRIVE_L2, cfg::DRIVE_L3});
    ui::track_motor_group("Drive R", {cfg::DRIVE_R1, cfg::DRIVE_R2, cfg::DRIVE_R3});
    ui::track_motor_group("Lift", {cfg::LIFT_1, cfg::LIFT_2});
    ui::track_motor_group("Aux", {cfg::AUX_1, cfg::AUX_2});

    if (imu_present) ui::track_imu("IMU", cfg::IMU_PORT);
    if (distance_present) ui::track_distance("Claw", cfg::DISTANCE_PORT);

    // Tune CLAMP_SEAT_MM by watching this row with a cone in the jaw.
    ui::track_value("Lift", "deg", [] { return lift::height(); });
    ui::track_value("Cones", "", [] { return static_cast<double>(lift::cone_count()); });
    ui::track_value("Claw gap", "mm", [] { return static_cast<double>(claw::distance_mm()); });

    // Auto-clamp also rejects reads below CLAMP_MIN_OBJECT_SIZE. If "Claw gap"
    // is under CLAMP_SEAT_MM but "Seated" never lights, check this row: the
    // sensor's size estimate may read low at close range.
    ui::track_value("Claw size", "", [] {
        return distance_present ? static_cast<double>(claw_distance.get_object_size()) : -1.0;
    });

    // Positive while a cone is getting nearer. Feeds the early-fire lead.
    ui::track_value("Closing", "mm/s", [] { return static_cast<double>(claw::closing_speed()); });

    ui::track_flag("Claw", [] { return claw::closed(); });
    ui::track_flag("Auto-clamp", [] { return claw::armed(); });
    ui::track_flag("Seated", [] { return claw::cone_seated(); });
    ui::track_flag("Heading", [] { return drive::imu_ready(); });
    ui::track_flag("Upright", [] { return !drive::tipping(); });

    // Tip the robot by hand and watch this move -- if it does not, the IMU is
    // mounted on a different axis and ANTITIP_USE_ROLL needs flipping.
    ui::track_value("Tilt", "deg", [] { return drive::tilt(); });

    // Watch this during a slow descent with a cone to size DUNK_TORQUE_DROP_NM.
    ui::track_value("Lift Nm", "", [] { return std::fabs(lift_mg.get_torque()); });
    ui::track_flag("Lift OK", [] { return !lift::stalled(); });

    /* Print what is actually live. With a half-configured robot this is the
     * difference between "the macro is broken" and "the macro is switched
     * off", which look identical from the driver's seat. */
    ui::log("--- enabled features ---");
    auto say = [](const char* name, bool on) {
        if (on) ui::log("  on: %s", name);
    };
    say("lift sensing trusted", cfg::ENABLE_LIFT_SENSING);
    say("creep", cfg::ENABLE_CREEP);
    say("drive slew", cfg::ENABLE_DRIVE_SLEW);
    say("lift-aware scaling", cfg::ENABLE_LIFT_SCALING);
    say("heading hold", cfg::ENABLE_HEADING_HOLD);
    say("lift PD hold", cfg::ENABLE_LIFT_HOLD);
    say("lift soft limits", cfg::ENABLE_LIFT_LIMITS);
    say("presets", cfg::ENABLE_PRESETS);
    say("auto-clamp", cfg::ENABLE_AUTO_CLAMP);
    say("anti-tip", cfg::ENABLE_ANTITIP);
    say("stack macro", cfg::ENABLE_STACK_MACRO);
    say("match alerts", cfg::ENABLE_MATCH_ALERTS);
    say("stall guard", cfg::ENABLE_STALL_GUARD);

    pros::Task(controller_screen_task, nullptr, "ctrl-screen");
}

void driver_step(pros::Controller& master) {
    // Re-point every tick, not just once: opcontrol is killed and restarted
    // between the autonomous and driver periods, so a pointer cached on the
    // first run would dangle into a destroyed stack frame on the second.
    g_master = &master;
    g_controller = &master;

    const std::uint32_t now = pros::millis();
    if (now - g_last_step_ms > 500) {
        g_period_start.store(now);
        for (bool& fired : g_alert_fired) fired = false;
        g_aux_on = false;
        ui::log("Driver period started");
    }
    g_last_step_ms = now;

    /* --- drive ----------------------------------------------------------- */
    const int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    const bool creep = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    g_r2_held.store(creep);

    drive::arcade(forward, turn, creep);

    /* --- lift: manual always wins ---------------------------------------- */
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        lift::manual(cfg::LIFT_MAX_POWER);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        lift::manual(-cfg::LIFT_MAX_POWER);
    } else {
        lift::manual(0);
    }

    /* --- aux motors: L2 toggles on / off ----------------------------------- */
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        g_aux_on = !g_aux_on;
        ui::log("Aux %s", g_aux_on ? "on" : "off");
    }
    aux_mg.move(g_aux_on ? cfg::AUX_POWER : 0);

    /* --- lift: presets and cone counter ------------------------------------ */
    if (cfg::ENABLE_PRESETS) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift::go_to_preset();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            ui::log("Lift -> travel");
            lift::move_to(cfg::TRAVEL_HEIGHT);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            lift::bump_cone_count(1);
            ui::log("Cone count %d", lift::cone_count());
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            lift::bump_cone_count(-1);
            ui::log("Cone count %d", lift::cone_count());
        }
    }

    /* --- stack macro -------------------------------------------------------
     * One button, context aware: raise if we are not at the preset yet, dunk
     * if we are, abort if it is already running. Manual lift input also
     * aborts, so this never takes the robot away from the driver. */
    if (cfg::ENABLE_STACK_MACRO &&
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        stack::begin();
    }

    /* --- claw -------------------------------------------------------------- */
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        claw::toggle();
    }
    if (cfg::ENABLE_AUTO_CLAMP &&
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        claw::set_armed(!claw::armed());
        master.rumble(claw::armed() ? "." : "-");
    }

    service_match_alerts(master);

    /* --- hold-to-confirm maintenance --------------------------------------- */
    if (held_long_enough(master.get_digital(pros::E_CONTROLLER_DIGITAL_X), g_x_down_at,
                         g_x_fired)) {
        lift::tare();
        master.rumble("..");
    }
    if (cfg::ENABLE_PRESETS &&
        held_long_enough(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y), g_y_down_at,
                         g_y_fired)) {
        lift::teach_preset();
        master.rumble("..");
    }
}

}  // namespace robot
