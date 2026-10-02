/**
 * @file src/ui/storage.cpp
 *
 * Remembers the autonomous selection across reboots by writing a two-line
 * file to the SD card. Everything here degrades to a silent no-op when no
 * card is inserted, because a missing card must never stop the robot.
 */
#include <cstdio>
#include <cstring>

#include "internal.hpp"
#include "pros/misc.hpp"

namespace ui {
namespace internal {

namespace {
constexpr const char* PATH = "/usd/936b_ui.cfg";
bool g_available = false;
}  // namespace

bool storage_available() { return g_available; }

void storage_load() {
    g_available = pros::usd::is_installed() != 0;
    if (!g_available) return;

    std::FILE* f = std::fopen(PATH, "r");
    if (f == nullptr) return;  // first run; the file appears on the first tap

    int alliance = 0;
    char name[48] = {};
    if (std::fscanf(f, "alliance=%d\n", &alliance) == 1 &&
        std::fgets(name, sizeof(name), f) != nullptr) {
        /* fgets keeps the newline and the "auton=" prefix */
        char* value = std::strstr(name, "auton=");
        if (value != nullptr) {
            value += 6;
            value[std::strcspn(value, "\r\n")] = '\0';
            if (alliance >= 0 && alliance <= 2) {
                selected_alliance() = static_cast<Alliance>(alliance);
            }
            for (std::size_t i = 0; i < autons().size(); ++i) {
                if (autons()[i].alliance == selected_alliance() && autons()[i].name == value) {
                    selected_index() = static_cast<int>(i);
                    break;
                }
            }
        }
    }
    std::fclose(f);
}

void storage_save() {
    if (!g_available) return;
    const AutonDef* a = selected_auton();
    std::FILE* f = std::fopen(PATH, "w");
    if (f == nullptr) {
        g_available = false;  // card pulled mid-match; stop trying
        return;
    }
    std::fprintf(f, "alliance=%d\n", static_cast<int>(selected_alliance()));
    std::fprintf(f, "auton=%s\n", a != nullptr ? a->name.c_str() : "");
    std::fclose(f);
}

}  // namespace internal
}  // namespace ui
