#include <cmath>

#include "internal.hpp"

namespace robot::drive {

namespace cfg = robot::config;
using namespace robot::internal;

namespace {
int deadband(int v) { return std::abs(v) < cfg::JOY_DEADBAND ? 0 : v; }
}  // namespace

void arcade(int forward_raw, int turn_raw, bool creep) {
    double forward = deadband(forward_raw);
    double turn = deadband(turn_raw);

    if (creep) {
        forward *= cfg::CREEP_FORWARD_SCALE;
        turn *= cfg::CREEP_TURN_SCALE;
    }

    // The jaw takes ~150 ms to shut, so slow down on the way into a cone.
    if (claw::armed() && !claw::closed()) {
        const int mm = claw::distance_mm();
        if (mm > 0 && mm <= cfg::APPROACH_SLOW_MM && forward > cfg::APPROACH_MAX_POWER) {
            forward = cfg::APPROACH_MAX_POWER;
        }
    }

    left_mg.move(clampi(static_cast<int>(std::lround(forward + turn)), -127, 127));
    right_mg.move(clampi(static_cast<int>(std::lround(forward - turn)), -127, 127));
}

}  // namespace robot::drive
