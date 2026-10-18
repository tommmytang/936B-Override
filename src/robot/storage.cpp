/**
 * @file src/robot/storage.cpp
 *
 * Persists taught preset heights to the SD card so tuning survives a reboot
 * and a re-flash. Degrades to a silent no-op with no card inserted, because a
 * missing card must never stop the robot.
 */
#include <cstdio>

#include "internal.hpp"
#include "pros/misc.hpp"
#include "ui/ui.hpp"

namespace robot::internal {

namespace {
constexpr const char* PATH = "/usd/936b_lift.cfg";
}

void presets_load(double* out, int count) {
    if (pros::usd::is_installed() == 0) return;

    std::FILE* f = std::fopen(PATH, "r");
    if (f == nullptr) return;  // first run; the file appears on the first teach

    int loaded = 0;
    for (int i = 0; i < count; ++i) {
        int index = 0;
        double value = 0.0;
        if (std::fscanf(f, "cone%d=%lf\n", &index, &value) != 2) break;
        if (index >= 0 && index < count) {
            out[index] = value;
            ++loaded;
        }
    }
    std::fclose(f);
    if (loaded > 0) ui::log("Loaded %d lift presets from SD", loaded);
}

void presets_save(const double* in, int count) {
    if (pros::usd::is_installed() == 0) {
        ui::warn("No SD card -- preset not saved");
        return;
    }

    std::FILE* f = std::fopen(PATH, "w");
    if (f == nullptr) {
        ui::warn("Could not write %s", PATH);
        return;
    }
    for (int i = 0; i < count; ++i) {
        std::fprintf(f, "cone%d=%.2f\n", i, in[i]);
    }
    std::fclose(f);
}

}  // namespace robot::internal
