#include "ui/theme.hpp"

namespace ui {
namespace theme {

const lv_font_t* f_micro() { return &lv_font_montserrat_10; }
const lv_font_t* f_small() { return &lv_font_montserrat_12; }
const lv_font_t* f_body() { return &lv_font_montserrat_14; }
const lv_font_t* f_title() { return &lv_font_montserrat_16; }
const lv_font_t* f_large() { return &lv_font_montserrat_20; }
const lv_font_t* f_huge() { return &lv_font_montserrat_36; }

lv_color_t mix(std::uint32_t a, std::uint32_t b, std::uint8_t amount) {
    return lv_color_mix(lv_color_hex(b), lv_color_hex(a), amount);
}

std::uint32_t heat_color(double value, double warn, double danger) {
    if (value >= danger) return DANGER;
    if (value >= warn) return WARN;
    return OK;
}

}  // namespace theme
}  // namespace ui
