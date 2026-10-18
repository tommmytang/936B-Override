/**
 * @file src/robot/claw.cpp
 *
 * Pneumatic claw, plus auto-clamp.
 *
 * Auto-clamp is the answer to a narrow jaw: instead of the driver lining up
 * AND timing the grab, a Distance sensor in the throat of the claw watches
 * for a cone entering the gap and fires the solenoid the instant one is
 * seated. The driver only has to drive at the cone.
 *
 * Why Distance and not Optical: the Optical sensor's proximity channel is an
 * IR reflectance reading, so its value changes with venue lighting and with
 * how dark the cone is -- a threshold tuned on a light cone misfires on a
 * dark one. Distance reports real millimetres and does neither.
 *
 * Releasing is never automatic. Dropping a cone you did not mean to drop
 * costs more than a slow release ever saves.
 */
#include <atomic>

#include "internal.hpp"
#include "pros/rtos.hpp"
#include "ui/ui.hpp"

namespace robot::claw {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {

std::atomic<bool> g_closed{false};
std::atomic<bool> g_armed{cfg::CLAMP_ARMED_AT_START};
std::atomic<int> g_distance_mm{-1};
std::atomic<bool> g_seated{false};
std::atomic<std::uint32_t> g_cooldown_until{0};

/* Set whenever the claw opens. Auto-clamp refuses to fire again until the
 * sensor has seen an empty jaw at least once, so releasing a cone and then
 * sitting over it for a moment does not immediately re-grab it. */
std::atomic<bool> g_needs_clear{false};

/* How fast the gap is shrinking, mm/s (positive = getting nearer). The
 * sensor refreshes slower than the task loop, so this is measured across a
 * baseline of at least SPEED_BASELINE_MS rather than tick to tick, then
 * smoothed. Only the auto-clamp task writes it. */
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

/** A reading we are willing to act on. The confidence and object-size gates
 *  are what stop a far wall or a stray reflection from closing the claw in
 *  the middle of the field. */
bool valid_seat_reading() {
    const int mm = claw_distance.get_distance();
    if (mm == PROS_ERR || mm <= 0 || mm >= 9999) {
        g_distance_mm.store(-1);
        update_closing_speed(-1);
        return false;
    }
    g_distance_mm.store(mm);
    update_closing_speed(mm);

    /* Where the cone will be by the time the piston has actually shut the
     * jaw. Standing still (or backing away) this is just the reading. */
    const double closing = g_speed_filtered > 0.0 ? g_speed_filtered : 0.0;
    const double predicted = mm - closing * cfg::CLAMP_LEAD_MS / 1000.0;
    const int reach = cfg::CLAMP_LEAD_MS > 0 && cfg::CLAMP_LEAD_MAX_MM > cfg::CLAMP_SEAT_MM
                          ? cfg::CLAMP_LEAD_MAX_MM
                          : cfg::CLAMP_SEAT_MM;

    if (mm < cfg::CLAMP_MIN_MM || mm > reach) return false;
    if (predicted > cfg::CLAMP_SEAT_MM) return false;
    // Confidence only exists beyond 200 mm; closer than that the sensor
    // always reports 10 (see pros/distance.h). Gating on it inside the jaw
    // rejected every read, so only apply it where it means something.
    if (mm > 200 && claw_distance.get_confidence() < cfg::CLAMP_MIN_CONFIDENCE) return false;

    // At close range the sensor gives up estimating size and reports -1 --
    // seen on the DIAG page: 70 at a distance, -1 once the cone is in the jaw.
    const int size = claw_distance.get_object_size();
    if (cfg::CLAMP_ONLY_ON_UNKNOWN_SIZE) {
        // Use that -1 as the "seated" signal itself.
        if (size != -1) return false;
    } else if (size >= 0 && size < cfg::CLAMP_MIN_OBJECT_SIZE) {
        // -1 means "unknown", not "small", so only a real size can reject.
        return false;
    }
    return true;
}

void auto_clamp_task(void*) {
    int good_ticks = 0;
    const char* last_blocked = nullptr;  //< so each reason is logged once, not every tick

    while (true) {
        if (!cfg::ENABLE_AUTO_CLAMP || !distance_present) {
            pros::delay(100);
            continue;
        }

        const bool seated = valid_seat_reading();
        g_seated.store(seated);

        const bool cooling = pros::millis() < g_cooldown_until.load();

        // An empty jaw re-arms the trigger.
        if (!seated) g_needs_clear.store(false);

        /* Seated but not allowed to fire: say why on the LOG page. From the
         * driver's seat "it saw the cone and did nothing" looks the same for
         * every one of these. */
        const char* blocked = nullptr;
        if (seated) {
            if (!g_armed.load()) {
                blocked = "disarmed (D-pad right re-arms)";
            } else if (g_closed.load()) {
                blocked = "code thinks the claw is already closed";
            } else if (g_needs_clear.load()) {
                blocked = "waiting for an empty jaw after opening";
            }
        }
        if (blocked != last_blocked) {
            if (blocked != nullptr) ui::warn("Auto-clamp blocked: %s", blocked);
            last_blocked = blocked;
        }

        if (g_armed.load() && !g_closed.load() && !cooling && seated &&
            !g_needs_clear.load()) {
            // Require a few consecutive reads. One frame of noise should not
            // be able to close the claw on nothing.
            if (++good_ticks >= cfg::CLAMP_DEBOUNCE_TICKS) {
                close();
                ui::log("Auto-clamp: grabbed at %d mm, closing at %d mm/s",
                        g_distance_mm.load(), g_closing_mm_s.load());
                if (g_controller != nullptr) g_controller->rumble(".");
                good_ticks = 0;
            }
        } else {
            // A streak that started and broke before the debounce finished
            // means the reading is flickering in and out of the window.
            if (good_ticks > 0 && !seated) {
                ui::warn("Auto-clamp: seated %d/%d reads, then lost it at %d mm", good_ticks,
                         cfg::CLAMP_DEBOUNCE_TICKS, g_distance_mm.load());
            }
            good_ticks = 0;
        }

        // 10 ms: every tick of latency here is ~15 mm of robot travel at
        // full speed. The debounce (CLAMP_DEBOUNCE_TICKS) is counted in these.
        pros::delay(10);
    }
}

}  // namespace

/* ======================================================================= */

bool closed() { return g_closed.load(); }
bool armed() { return g_armed.load(); }
int distance_mm() { return g_distance_mm.load(); }
bool cone_seated() { return g_seated.load(); }
int closing_speed() { return g_closing_mm_s.load(); }

/** Solenoid output for a claw state. CLAW_EXTEND_TO_CLOSE says which way the
 *  piston grips; nothing else should call claw_solenoid.set_value(). */
bool solenoid_for(bool closed) { return closed == cfg::CLAW_EXTEND_TO_CLOSE; }

void close() {
    claw_solenoid.set_value(solenoid_for(true));
    g_closed.store(true);
    g_cooldown_until.store(pros::millis() + cfg::CLAMP_COOLDOWN_MS);
}

void open() {
    // Bump the cone counter on the way out: opening the claw up high is the
    // tail end of a successful stack, so the next preset should be one higher
    // without the driver touching the D-pad.
    if (cfg::ENABLE_PRESETS && cfg::AUTO_INCREMENT_ON_RELEASE && g_closed.load() &&
        lift::height() > cfg::RELEASE_COUNTS_ABOVE_DEG) {
        lift::bump_cone_count(1);
        ui::log("Stacked -- cone count now %d", lift::cone_count());
    }

    claw_solenoid.set_value(solenoid_for(false));
    g_closed.store(false);
    g_needs_clear.store(true);
    g_cooldown_until.store(pros::millis() + cfg::CLAMP_COOLDOWN_MS);
}

void toggle() {
    if (g_closed.load()) {
        open();
    } else {
        close();
    }
}

void set_armed(bool on) {
    g_armed.store(on);
    ui::log("Auto-clamp %s", on ? "ARMED" : "off");
}

}  // namespace robot::claw

namespace robot::internal {

void claw_init() {
    // Leave the solenoid off at power-on so nothing moves when the robot
    // boots, and record whichever claw state "off" happens to be. With
    // CLAW_EXTEND_TO_CLOSE false that is CLOSED: the driver (or the auton)
    // opens it before the first auto-grab.
    claw_solenoid.set_value(false);
    robot::claw::g_closed.store(!config::CLAW_EXTEND_TO_CLOSE);
    pros::Task(robot::claw::auto_clamp_task, nullptr, "auto-clamp");
}

}  // namespace robot::internal
