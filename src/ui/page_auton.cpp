/**
 * @file src/ui/page_auton.cpp
 *
 * The autonomous selector: an alliance segmented control, a scrolling list of
 * routines, and a footer that confirms what will run. Tapping anything writes
 * the choice straight to the SD card, so an accidental reboot in the queue
 * line costs nothing.
 */
#include <cstdio>

#include "internal.hpp"
#include "pros/rtos.hpp"

namespace ui {
namespace internal {

using namespace theme;

namespace {

constexpr std::int32_t INNER_W = CONTENT_W - 12;   // 406
constexpr std::int32_t TAB_H = 28;
constexpr std::int32_t FOOT_H = 24;
constexpr std::int32_t LIST_H = CONTENT_H - 12 - TAB_H - FOOT_H - 12;
constexpr std::int32_t ROW_H = 42;

const char* const TAB_NAMES[3] = {"RED", "BLUE", "SKILLS"};

lv_obj_t* g_tab[3] = {};
lv_obj_t* g_tab_label[3] = {};
lv_obj_t* g_list = nullptr;
lv_obj_t* g_foot_name = nullptr;
lv_obj_t* g_foot_saved = nullptr;
lv_obj_t* g_test_btn = nullptr;
lv_obj_t* g_test_label = nullptr;

/** Row objects, parallel to the visible subset of autons(). */
std::vector<lv_obj_t*> g_rows;
std::vector<int> g_row_index;  //< maps a row back to its index in autons()

bool g_test_running = false;

std::uint32_t tab_color(int i) {
    switch (i) {
        case 0: return RED_ALLIANCE;
        case 1: return BLUE_ALLIANCE;
        default: return ACCENT_HI;
    }
}

void paint_tabs() {
    const int active = static_cast<int>(selected_alliance());
    for (int i = 0; i < 3; ++i) {
        const bool on = (i == active);
        lv_obj_set_style_bg_color(g_tab[i], c(on ? tab_color(i) : SURFACE), LV_PART_MAIN);
        lv_obj_set_style_border_color(g_tab[i], c(on ? tab_color(i) : BORDER), LV_PART_MAIN);
        lv_obj_set_style_text_color(g_tab_label[i], c(on ? TEXT : TEXT_DIM), LV_PART_MAIN);
    }
}

void paint_rows() {
    const int sel = selected_index();
    for (std::size_t i = 0; i < g_rows.size(); ++i) {
        const bool on = (g_row_index[i] == sel);
        lv_obj_set_style_bg_color(g_rows[i], c(on ? SURFACE_HI : SURFACE), LV_PART_MAIN);
        lv_obj_set_style_border_color(g_rows[i], c(on ? alliance_color() : BORDER), LV_PART_MAIN);
        lv_obj_set_style_border_width(g_rows[i], on ? 2 : 1, LV_PART_MAIN);
        /* child 0 is the left accent bar, child 3 the tick */
        lv_obj_set_style_bg_opa(lv_obj_get_child(g_rows[i], 0),
                                on ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_bg_color(lv_obj_get_child(g_rows[i], 0), c(alliance_color()),
                                  LV_PART_MAIN);
        lv_obj_set_style_text_opa(lv_obj_get_child(g_rows[i], 3),
                                  on ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_text_color(lv_obj_get_child(g_rows[i], 3), c(alliance_color()),
                                    LV_PART_MAIN);
    }
}

void paint_footer() {
    const AutonDef* a = selected_auton();
    lv_label_set_text_fmt(g_foot_name, "%s  " LV_SYMBOL_RIGHT "  %s", TAB_NAMES[static_cast<int>(selected_alliance())],
                          a != nullptr ? a->name.c_str() : "nothing selected");
    lv_obj_set_style_text_color(g_foot_name, c(a != nullptr ? TEXT : TEXT_FAINT), LV_PART_MAIN);

    if (storage_available()) {
        lv_label_set_text(g_foot_saved, LV_SYMBOL_SD_CARD " saved");
        lv_obj_set_style_text_color(g_foot_saved, c(OK), LV_PART_MAIN);
    } else {
        lv_label_set_text(g_foot_saved, LV_SYMBOL_WARNING " no card");
        lv_obj_set_style_text_color(g_foot_saved, c(WARN), LV_PART_MAIN);
    }
    lv_obj_align(g_foot_saved, LV_ALIGN_RIGHT_MID, -74, 0);
}

void on_row_click(lv_event_t* e) {
    const int index = static_cast<int>(reinterpret_cast<std::intptr_t>(lv_event_get_user_data(e)));
    {
        std::lock_guard<pros::Mutex> guard(data_lock());
        selected_index() = index;
        storage_save();
    }
    paint_rows();
    paint_footer();
}

/** Rebuilds the routine list for the currently selected alliance. */
void fill_list() {
    lv_obj_clean(g_list);
    g_rows.clear();
    g_row_index.clear();

    const Alliance want = selected_alliance();
    for (std::size_t i = 0; i < autons().size(); ++i) {
        const AutonDef& a = autons()[i];
        if (a.alliance != want) continue;

        lv_obj_t* row = card(g_list, INNER_W - 8, ROW_H);
        lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
        lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(row, on_row_click, LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(static_cast<std::intptr_t>(i)));

        lv_obj_t* bar = blank(row, 4, ROW_H - 12);  //< child 0
        lv_obj_align(bar, LV_ALIGN_LEFT_MID, 5, 0);
        lv_obj_set_style_radius(bar, 2, LV_PART_MAIN);

        /* 16px Montserrat is 22px tall and the caption 13px: in a 42px row that
         * leaves exactly 3px of padding top and bottom, and no overlap. */
        lv_obj_t* name = text(row, a.name.c_str(), f_title(), TEXT);  //< child 1
        lv_obj_align(name, a.description.empty() ? LV_ALIGN_LEFT_MID : LV_ALIGN_TOP_LEFT, 16,
                     a.description.empty() ? 0 : 3);

        lv_obj_t* desc = text(row, a.description.c_str(), f_micro(), TEXT_DIM);  //< child 2
        lv_obj_align(desc, LV_ALIGN_BOTTOM_LEFT, 16, -3);

        lv_obj_t* tick = text(row, LV_SYMBOL_OK, f_title(), TEXT);  //< child 3
        lv_obj_align(tick, LV_ALIGN_RIGHT_MID, -10, 0);

        g_rows.push_back(row);
        g_row_index.push_back(static_cast<int>(i));
    }

    if (g_rows.empty()) {
        lv_obj_t* empty = text(g_list, "No routines registered for this alliance.\n"
                                       "Add them with ui::add_auton() in initialize().",
                               f_small(), TEXT_FAINT);
        lv_obj_set_style_text_align(empty, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_width(empty, INNER_W - 8);
        lv_obj_set_style_pad_top(empty, 40, LV_PART_MAIN);
    }
    paint_rows();
}

void on_tab_click(lv_event_t* e) {
    const int which = static_cast<int>(reinterpret_cast<std::intptr_t>(lv_event_get_user_data(e)));
    {
        std::lock_guard<pros::Mutex> guard(data_lock());
        selected_alliance() = static_cast<Alliance>(which);
        /* Jump to the first routine on the new alliance so the footer is never
         * showing a routine from the tab you just left. */
        selected_index() = -1;
        for (std::size_t i = 0; i < autons().size(); ++i) {
            if (autons()[i].alliance == selected_alliance()) {
                selected_index() = static_cast<int>(i);
                break;
            }
        }
        storage_save();
    }
    paint_tabs();
    fill_list();
    paint_footer();
}

/** Runs the selected routine from the brain, for practice runs in the pits. */
void test_task(void*) {
    ui::run_auton();
    g_test_running = false;
}

void on_test_click(lv_event_t*) {
    if (g_test_running) return;
    Snapshot s;
    {
        std::lock_guard<pros::Mutex> guard(data_lock());
        s.competition_linked = snapshot().competition_linked;
    }
    if (s.competition_linked) {
        ui::warn("Test blocked: competition switch is connected");
        return;
    }
    if (!ui::has_auton()) {
        ui::warn("Test blocked: nothing selected");
        return;
    }
    g_test_running = true;
    pros::Task(test_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "ui-test");
}

}  // namespace

void build_auton(lv_obj_t* parent) {
    /* ---- alliance tabs ---- */
    const std::int32_t tab_w = (INNER_W - 8) / 3;
    for (int i = 0; i < 3; ++i) {
        lv_obj_t* t = card(parent, tab_w, TAB_H);
        lv_obj_set_pos(t, i * (tab_w + 4), 0);
        lv_obj_add_flag(t, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(t, on_tab_click, LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(static_cast<std::intptr_t>(i)));
        lv_obj_t* l = text(t, TAB_NAMES[i], f_small(), TEXT_DIM);
        lv_obj_center(l);
        lv_obj_set_style_text_letter_space(l, 2, LV_PART_MAIN);
        g_tab[i] = t;
        g_tab_label[i] = l;
    }

    /* ---- scrolling routine list ---- */
    g_list = blank(parent, INNER_W, LIST_H);
    lv_obj_set_pos(g_list, 0, TAB_H + 6);
    lv_obj_add_flag(g_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(g_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(g_list, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_set_flex_flow(g_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_list, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_right(g_list, 4, LV_PART_MAIN);

    /* ---- footer ---- */
    lv_obj_t* foot = blank(parent, INNER_W, FOOT_H);
    lv_obj_set_pos(foot, 0, TAB_H + 6 + LIST_H + 6);

    g_foot_name = text(foot, "", f_small(), TEXT);
    lv_obj_align(g_foot_name, LV_ALIGN_LEFT_MID, 2, 0);

    g_foot_saved = text(foot, "", f_micro(), OK);
    lv_obj_align(g_foot_saved, LV_ALIGN_RIGHT_MID, -74, 0);

    g_test_btn = card(foot, 66, FOOT_H);
    lv_obj_align(g_test_btn, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_pad_all(g_test_btn, 0, LV_PART_MAIN);
    lv_obj_add_flag(g_test_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_test_btn, on_test_click, LV_EVENT_CLICKED, nullptr);
    g_test_label = text(g_test_btn, LV_SYMBOL_PLAY " TEST", f_micro(), TEXT_DIM);
    lv_obj_center(g_test_label);

    paint_tabs();
    fill_list();
    paint_footer();
}

void update_auton(const Snapshot& s) {
    /* The only thing that changes without a tap is whether TEST is allowed. */
    const bool allowed = !s.competition_linked && !g_test_running;
    lv_obj_set_style_text_color(g_test_label, c(allowed ? ACCENT_HI : TEXT_FAINT), LV_PART_MAIN);
    lv_obj_set_style_border_color(g_test_btn, c(allowed ? ACCENT : BORDER), LV_PART_MAIN);
    lv_label_set_text(g_test_label, g_test_running ? LV_SYMBOL_STOP " RUN" : LV_SYMBOL_PLAY " TEST");
}

void retint_auton() {
    paint_tabs();
    paint_rows();
    paint_footer();
}

}  // namespace internal
}  // namespace ui
