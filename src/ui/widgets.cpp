/**
 * @file src/ui/widgets.cpp
 *
 * The handful of building blocks every page is made of. Keeping them here is
 * what makes the pages short enough to read in one sitting.
 */
#include <cstdio>

#include "internal.hpp"

namespace ui {
namespace internal {

using namespace theme;

lv_obj_t* blank(lv_obj_t* parent, std::int32_t w, std::int32_t h) {
    lv_obj_t* o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, w, h);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

lv_obj_t* card(lv_obj_t* parent, std::int32_t w, std::int32_t h) {
    lv_obj_t* o = blank(parent, w, h);
    lv_obj_set_style_bg_color(o, c(SURFACE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(o, 7, LV_PART_MAIN);
    lv_obj_set_style_border_width(o, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(o, c(BORDER), LV_PART_MAIN);
    lv_obj_set_style_pad_all(o, 6, LV_PART_MAIN);
    return o;
}

lv_obj_t* text(lv_obj_t* parent, const char* str, const lv_font_t* font,
               std::uint32_t color) {
    lv_obj_t* l = lv_label_create(parent);
    lv_label_set_text(l, str);
    if (font != nullptr) lv_obj_set_style_text_font(l, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(l, c(color), LV_PART_MAIN);
    return l;
}

lv_obj_t* stat_tile(lv_obj_t* parent, std::int32_t w, std::int32_t h,
                    const char* caption, lv_obj_t** out_value,
                    const lv_font_t* value_font) {
    lv_obj_t* box = card(parent, w, h);
    lv_obj_set_style_pad_all(box, 5, LV_PART_MAIN);

    lv_obj_t* cap = text(box, caption, f_micro(), TEXT_FAINT);
    lv_obj_align(cap, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_letter_space(cap, 1, LV_PART_MAIN);

    lv_obj_t* val = text(box, "--", value_font ? value_font : f_large(), TEXT);
    lv_obj_align(val, LV_ALIGN_BOTTOM_LEFT, 0, 1);

    if (out_value != nullptr) *out_value = val;
    return box;
}

lv_obj_t* meter(lv_obj_t* parent, std::int32_t w, std::int32_t h) {
    lv_obj_t* b = lv_bar_create(parent);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, w, h);
    lv_bar_set_range(b, 0, 100);
    lv_bar_set_value(b, 0, LV_ANIM_OFF);

    lv_obj_set_style_bg_color(b, c(ACCENT_DEEP), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(b, h / 2, LV_PART_MAIN);

    lv_obj_set_style_bg_color(b, c(ACCENT_HI), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(b, h / 2, LV_PART_INDICATOR);
    return b;
}

lv_obj_t* dot(lv_obj_t* parent, std::int32_t size, std::uint32_t color) {
    lv_obj_t* d = blank(parent, size, size);
    lv_obj_set_style_bg_color(d, c(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    return d;
}

std::uint32_t alliance_color() {
    switch (selected_alliance()) {
        case Alliance::Red: return RED_ALLIANCE;
        case Alliance::Blue: return BLUE_ALLIANCE;
        default: return ACCENT_HI;
    }
}

void format_clock(char* out, std::size_t n, std::uint32_t ms) {
    std::uint32_t total = ms / 1000;
    std::snprintf(out, n, "%lu:%02lu", static_cast<unsigned long>(total / 60),
                  static_cast<unsigned long>(total % 60));
}

}  // namespace internal
}  // namespace ui
