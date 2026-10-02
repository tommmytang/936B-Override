/**
 * @file ui/theme.hpp
 *
 * Every colour, font and dimension the brain UI uses lives here.
 *
 * Want to re-skin the whole interface? Change ACCENT below and everything --
 * buttons, highlights, the field map, the nav rail -- follows automatically.
 */
#pragma once

#include <cstdint>

#include "liblvgl/lvgl.h"

namespace ui {
namespace theme {

/* ------------------------------------------------------------------ brand */

/** 936B purple. The one colour to change if you want a different look. */
inline constexpr std::uint32_t ACCENT = 0x5A2D81;
/** A brightened ACCENT, used for text and thin lines that sit on the dark
 *  background. The raw ACCENT is too dark to read at small sizes. */
inline constexpr std::uint32_t ACCENT_HI = 0xA862DA;
/** Midway between the two -- gradients, pressed states, chart fills. */
inline constexpr std::uint32_t ACCENT_MID = 0x7B3FB0;
/** A very dark wash of ACCENT, for inactive fills and subtle panels. */
inline constexpr std::uint32_t ACCENT_DEEP = 0x2E1642;

/* -------------------------------------------------------------- surfaces */

inline constexpr std::uint32_t BG = 0x0B0A10;          //< page background
inline constexpr std::uint32_t SURFACE = 0x171320;     //< cards, list rows
inline constexpr std::uint32_t SURFACE_HI = 0x221C30;  //< raised / selected
inline constexpr std::uint32_t BORDER = 0x2E2740;      //< hairlines

/* ------------------------------------------------------------------ text */

inline constexpr std::uint32_t TEXT = 0xF2EFF7;
inline constexpr std::uint32_t TEXT_DIM = 0x9A92AD;
inline constexpr std::uint32_t TEXT_FAINT = 0x655C7A;

/* -------------------------------------------------------------- semantic */

inline constexpr std::uint32_t OK = 0x2BD46B;
inline constexpr std::uint32_t WARN = 0xF5A524;
inline constexpr std::uint32_t DANGER = 0xF04A4A;
inline constexpr std::uint32_t RED_ALLIANCE = 0xE3392E;
inline constexpr std::uint32_t BLUE_ALLIANCE = 0x2B7FE0;

/* ------------------------------------------------------------- geometry */

inline constexpr std::int32_t SCREEN_W = 480;
inline constexpr std::int32_t SCREEN_H = 240;
inline constexpr std::int32_t RAIL_W = 62;   //< left navigation rail
inline constexpr std::int32_t TOPBAR_H = 26; //< status strip above content
inline constexpr std::int32_t CONTENT_X = RAIL_W;
inline constexpr std::int32_t CONTENT_Y = TOPBAR_H;
inline constexpr std::int32_t CONTENT_W = SCREEN_W - RAIL_W;
inline constexpr std::int32_t CONTENT_H = SCREEN_H - TOPBAR_H;

/* ----------------------------------------------------------------- fonts */

const lv_font_t* f_micro();  //< 10px -- captions
const lv_font_t* f_small();  //< 12px -- secondary labels
const lv_font_t* f_body();   //< 14px -- default body text
const lv_font_t* f_title();  //< 16px -- card titles
const lv_font_t* f_large();  //< 20px -- stat values
const lv_font_t* f_huge();   //< 36px -- the battery / timer readout

/* --------------------------------------------------------------- helpers */

inline lv_color_t c(std::uint32_t hex) { return lv_color_hex(hex); }

/** Blends `a` toward `b`. `mix` is 0-255, where 0 is all `a`. */
lv_color_t mix(std::uint32_t a, std::uint32_t b, std::uint8_t mix);

/** Green below `warn`, amber up to `danger`, red above. Used for temps. */
std::uint32_t heat_color(double value, double warn, double danger);

}  // namespace theme
}  // namespace ui
