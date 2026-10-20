#include <atomic>

#include "internal.hpp"
#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot::claw {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {

std::atomic<bool> g_closed{false};
std::atomic<int> g_grabs{0};
std::atomic<bool> g_armed{cfg::CLAMP_ARMED_AT_START};
std::atomic<int> g_distance_mm{-1};
std::atomic<bool> g_seated{false};
std::atomic<std::uint32_t> g_cooldown_until{0};
// Set on open; auto-clamp waits for an empty jaw so it doesn't re-grab the cone it just let go.
std::atomic<bool> g_needs_clear{false};

// Closing speed, measured over >= 40 ms (the sensor updates slower than 10 ms) and smoothed.
constexpr std::uint32_t SPEED_BASELINE_MS = 40;
std::atomic<int> g_closing_mm_s{0};
double g_speed_filtered = 0.0;
int g_ref_mm = -1;
std::uint32_t g_ref_ms = 0;

void update_closing_speed(int mm) {
    const std::uint32_t now = pros::millis();
    if (mm < 0) {
        g_ref_mm = -1;
        g_speed_filtered = 0.0;
    } else if (g_ref_mm < 0) {
        g_ref_mm = mm;
        g_ref_ms = now;
    } else if (now - g_ref_ms >= SPEED_BASELINE_MS) {
        const double v = (g_ref_mm - mm) * 1000.0 / static_cast<double>(now - g_ref_ms);
        g_speed_filtered = 0.5 * g_speed_filtered + 0.5 * v;
        g_ref_mm = mm;
        g_ref_ms = now;
    }
    g_closing_mm_s.store(static_cast<int>(g_speed_filtered));
}

bool valid_seat_reading() {
    const int mm = claw_distance.get_distance();
    if (mm == PROS_ERR || mm <= 0 || mm >= 9999) {
        g_distance_mm.store(-1);
        update_closing_speed(-1);
        return false;
    }
    g_distance_mm.store(mm);
    update_closing_speed(mm);

    // Where the cone will be once the piston has shut.
    const double closing = g_speed_filtered > 0.0 ? g_speed_filtered : 0.0;
    const double predicted = mm - closing * cfg::CLAMP_LEAD_MS / 1000.0;
    const int reach = cfg::CLAMP_LEAD_MS > 0 && cfg::CLAMP_LEAD_MAX_MM > cfg::CLAMP_SEAT_MM
                          ? cfg::CLAMP_LEAD_MAX_MM
                          : cfg::CLAMP_SEAT_MM;

    if (mm < cfg::CLAMP_MIN_MM || mm > reach) return false;
    if (predicted > cfg::CLAMP_SEAT_MM) return false;
    // Confidence is a constant 10 under 200 mm, so only gate on it beyond that.
    if (mm > 200 && claw_distance.get_confidence() < cfg::CLAMP_MIN_CONFIDENCE) return false;

    const int size = claw_distance.get_object_size();
    if (cfg::CLAMP_ONLY_ON_UNKNOWN_SIZE) return size == -1;
    return size < 0 || size >= cfg::CLAMP_MIN_OBJECT_SIZE;
}

void auto_clamp_task(void*) {
    int good_ticks = 0;
    const char* last_blocked = nullptr;

    while (true) {
        if (!distance_present) {
            pros::delay(100);
            continue;
        }

        const bool seated = valid_seat_reading();
        g_seated.store(seated);
        if (!seated) g_needs_clear.store(false);

        const char* blocked = nullptr;
        if (seated) {
            if (!g_armed.load()) blocked = "disarmed (D-pad right re-arms)";
            else if (g_closed.load()) blocked = "claw already closed";
            else if (g_needs_clear.load()) blocked = "waiting for an empty jaw";
        }
        if (blocked != last_blocked) {
            if (blocked != nullptr) ui::warn("Auto-clamp blocked: %s", blocked);
            last_blocked = blocked;
        }

        const bool cooling = pros::millis() < g_cooldown_until.load();
        if (seated && blocked == nullptr && !cooling) {
            if (++good_ticks >= cfg::CLAMP_DEBOUNCE_TICKS) {
                close();
                ui::log("Auto-clamp: grabbed at %d mm, closing at %d mm/s",
                        g_distance_mm.load(), g_closing_mm_s.load());
                master.rumble(".");
                good_ticks = 0;
            }
        } else {
            if (good_ticks > 0 && !seated) {
                ui::warn("Auto-clamp: seated %d/%d reads, then lost it at %d mm", good_ticks,
                         cfg::CLAMP_DEBOUNCE_TICKS, g_distance_mm.load());
            }
            good_ticks = 0;
        }
        pros::delay(10);
    }
}

bool solenoid_for(bool closed) { return closed == cfg::CLAW_EXTEND_TO_CLOSE; }

}  // namespace

bool closed() { return g_closed.load(); }
bool armed() { return g_armed.load(); }
int distance_mm() { return g_distance_mm.load(); }
bool cone_seated() { return g_seated.load(); }
int closing_speed() { return g_closing_mm_s.load(); }
int grabs() { return g_grabs.load(); }

void close() {
    if (!g_closed.load()) g_grabs.fetch_add(1);
    claw_solenoid.set_value(solenoid_for(true));
    g_closed.store(true);
    g_cooldown_until.store(pros::millis() + cfg::CLAMP_COOLDOWN_MS);
}

void open() {
    claw_solenoid.set_value(solenoid_for(false));
    g_closed.store(false);
    g_needs_clear.store(true);
    g_cooldown_until.store(pros::millis() + cfg::CLAMP_COOLDOWN_MS);
}

void toggle() { g_closed.load() ? open() : close(); }

void set_armed(bool on) {
    g_armed.store(on);
    ui::log("Auto-clamp %s", on ? "ARMED" : "off");
}

}  // namespace robot::claw

namespace robot::internal {

void claw_init() {
    // Solenoid off at boot, which is closed on this robot.
    claw_solenoid.set_value(false);
    robot::claw::g_closed.store(!config::CLAW_EXTEND_TO_CLOSE);
    pros::Task(robot::claw::auto_clamp_task, nullptr, "auto-clamp");
}

}  // namespace robot::internal
