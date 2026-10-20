#include <atomic>

#include "internal.hpp"
#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot::lift {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {

std::atomic<int> g_manual_power{0};
std::atomic<double> g_height{0.0};
std::atomic<double> g_offset{0.0};

std::atomic<bool> g_raise_pending{false};
std::atomic<std::uint32_t> g_raise_at{0};
std::atomic<std::uint32_t> g_raise_deadline{0};
std::atomic<double> g_raise_target{0.0};

double raw_arm_degrees() { return lift_mg.get_position() / cfg::LIFT_MOTOR_DEG_PER_ARM_DEG; }

/** Power a pending raise_by() wants this tick, or 0. */
int raise_power(double h) {
    if (!g_raise_pending.load()) return 0;
    const std::uint32_t now = pros::millis();
    if (g_manual_power.load() != 0 || h >= g_raise_target.load() ||
        now >= g_raise_deadline.load()) {
        g_raise_pending.store(false);
        return 0;
    }
    return now < g_raise_at.load() ? 0 : cfg::GRAB_LIFT_POWER;
}

void control_task(void*) {
    while (true) {
        const double h = raw_arm_degrees() - g_offset.load();
        g_height.store(h);
        const int raise = raise_power(h);
        const int power = raise != 0 ? raise : g_manual_power.load();
        lift_mg.move(clampi(power, -cfg::LIFT_MAX_POWER, cfg::LIFT_MAX_POWER));
        pros::delay(10);
    }
}

}  // namespace

double height() { return g_height.load(); }

void manual(int power) {
    g_manual_power.store(clampi(power, -cfg::LIFT_MAX_POWER, cfg::LIFT_MAX_POWER));
}

void raise_by(double arm_degrees, std::uint32_t delay_ms) {
    const std::uint32_t now = pros::millis();
    g_raise_target.store(g_height.load() + arm_degrees);
    g_raise_at.store(now + delay_ms);
    g_raise_deadline.store(now + delay_ms + cfg::GRAB_LIFT_TIMEOUT_MS);
    g_raise_pending.store(true);
}

void tare() {
    g_offset.store(raw_arm_degrees());
    g_height.store(0.0);
    ui::log("Lift zeroed");
}

}  // namespace robot::lift

namespace robot::internal {

void lift_init() {
    robot::lift::tare();
    pros::Task(robot::lift::control_task, nullptr, "lift");
}

}  // namespace robot::internal
