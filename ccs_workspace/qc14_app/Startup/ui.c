/*
 * ui.c
 *
 *  Created on: Jun 25, 2017
 *      Author: George
 */
#include <stdint.h>
#include <string.h>

#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include "ExtFlash.h"

#include "qc14.h"
#include "ui.h"
#include "serial.h"
#include "tlc_driver.h"

#define BLINK_LEN (BLINK_LEN_MS * 100)

extern uint8_t led_buf[11][7][3];
void ui_update(uint8_t ui_next);

// OS and Task constructs:
char screen_anim_task_stack[1024];
Task_Struct screen_anim_task; // Main UI task

Clock_Handle screen_anim_clock_h;  // Ticks when we need a new screen
void screen_anim_tick_swi(UArg a0);

Clock_Handle arm_anim_clock_h;  // Ticks when we need a new screen
void arm_anim_tick_swi(UArg a0);

Clock_Handle screen_blink_clock_h; // Ticks when we're blinking
void screen_blink_tick_swi(UArg a0);

Clock_Handle sw_debounce_clock; // Ticks to debounce the switches
void sw_clock_swi(UArg a0);

Clock_Handle csecs_clock_h; // Ticks to set the centisecond clock
void csecs_swi(UArg a0);

Semaphore_Handle screen_anim_sem; // Posted when we need a new screen
Semaphore_Handle arm_anim_sem; // Posted when we need new arms.
Semaphore_Handle flash_sem; // Protects the flash.
Semaphore_Handle sw_sem; // Posted when the switch is clicked.
Semaphore_Handle save_sem; // Posted when we need to save.
Semaphore_Handle unlock_sem;
Semaphore_Handle pool_sem;
Semaphore_Handle club_sem;

static PIN_State sw_pin_state;
PIN_Handle sw_pin_h;

volatile uint8_t screen_blink_status = 0;

uint16_t screen_frame_index = 0;
screen_anim_t screen_anim_storage;
screen_anim_t *screen_anim = &screen_anim_storage;

game_icon_t game_curr_icon;
tile_t curr_tile;
uint8_t sel_id = 0;

uint8_t ui_screen = UI_SCREEN_BOOT;

uint8_t sw_signal = SW_SIGNAL_NONE;
uint8_t sw_l_clicked = 0;
uint8_t sw_r_clicked = 0;
uint8_t sw_c_clicked = 0;

game_arm_status_t game_arm_status[4] = {0};
tile_arm_status_t tile_arm_status[4] = {0};

uint_fast32_t screen_timeout_ticks = 0;

PIN_Config sw_pin_table[] = {
    // Rocker switch:
    SW_L            | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    SW_R            | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    SW_CLICK        | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

const uint8_t rainbow_colors[6][3] = {{255,0,0}, {255,30,0}, {255,255,0}, {0,255,0}, {0,0,255}, {98,0,255}};
const screen_frame_t power_bmp = {{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}}}};
const screen_frame_t tile_placeholder = {{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}};
const screen_frame_t needflash_icon = {{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}}}, 0};
const screen_frame_t all_off = {0};

uint8_t tile_active = 0;
int8_t tile_offset = 0;

void screen_anim_tick_swi(UArg a0) {
    Semaphore_post(screen_anim_sem);
}
void arm_anim_tick_swi(UArg a0) {
    Semaphore_post(arm_anim_sem);
}
void screen_blink_tick_swi(UArg a0) {
    led_blank_set(screen_blink_status);
    screen_blink_status = !screen_blink_status;
}

volatile uint8_t onetime_animation = 0;

// TASK CONTEXT!
void do_icon_transition(uint16_t dest_icon) {
    if (dest_icon >= ICON_COUNT)
        return;

    game_set_icon(dest_icon);

    set_screen_animation(FLASH_POOF_ANIM_LOC, 0);
    do_animation_loop();
    ui_update(UI_SCREEN_GAME);
}

void its_cold() {
    // It's cold!
    //
    if (ui_screen == UI_SCREEN_GAME && my_conf.current_icon == ICON_WATER &&
            !serial_in_progress()) {
        // We're not mated, we're in game mode, and we have water up.
        // FREEZE IT.
        do_icon_transition(ICON_ICE);
    }
}

void its_bright() {
    // It's bright!
    //
    if (ui_screen == UI_SCREEN_GAME && my_conf.current_icon == ICON_EARTH &&
            !serial_in_progress()) {
        // We're not mated, we're in game mode, and we have water up.
        // FREEZE IT.
        do_icon_transition(ICON_SUN);
    }
}

void csecs_swi(UArg a0) {
    // It's been another centisecond of queercon.
    my_conf.csecs_of_queercon++;

    if (my_conf.csecs_of_queercon == POOL_TILE_TIME) {
        Semaphore_post(pool_sem);
    } else if (my_conf.csecs_of_queercon == CLUB_TILE_TIME) {
        Semaphore_post(club_sem);
    } else if (my_conf.csecs_of_queercon == UNLOCK_TIME) {
        Semaphore_post(unlock_sem);
    }

    if (!(my_conf.csecs_of_queercon % 8192)) {
        Semaphore_post(save_sem);
    }
}

void sw_clock_swi(UArg a0) {
    static uint8_t sw_l_last = 1;
    static uint8_t sw_r_last = 1;
    static uint8_t sw_c_last = 1;

    uint8_t sw_l_curr = PIN_getInputValue(SW_L);
    uint8_t sw_r_curr = PIN_getInputValue(SW_R);
    uint8_t sw_c_curr = PIN_getInputValue(SW_CLICK);

    sw_signal = SW_SIGNAL_NONE;

    if (!sw_l_curr && !sw_l_last && !sw_l_clicked) {
        // left clicked
        sw_l_clicked = 1;
        sw_signal = SW_SIGNAL_L;
        // do stuff
    } else if (sw_l_curr && sw_l_last && sw_l_clicked) {
        sw_l_clicked = 0;
        sw_signal = SW_SIGNAL_OPEN;
        // unclicked.
    }
    sw_l_last = sw_l_curr;

    if (!sw_r_curr && !sw_r_last && !sw_r_clicked) {
        // right clicked
        sw_r_clicked = 1;
        sw_signal = SW_SIGNAL_R;
        // do stuff
    } else if (sw_r_curr && sw_r_last && sw_r_clicked) {
        sw_r_clicked = 0;
        sw_signal = SW_SIGNAL_OPEN;
        // unclicked.
    }
    sw_r_last = sw_r_curr;

    if (!sw_c_curr && !sw_c_last && !sw_c_clicked) {
        // click clicked
        sw_c_clicked = 1;
        sw_signal = SW_SIGNAL_C;
        // do stuff
    } else if (sw_c_curr && sw_c_last && sw_c_clicked) {
        sw_c_clicked = 0;
        sw_signal = SW_SIGNAL_OPEN;
        // unclicked.
    }
    sw_c_last = sw_c_curr;

    if (serial_in_progress())
        return; // No UI during mating, but keep debouncing.

    if (sw_signal != SW_SIGNAL_NONE) {
        // User interaction of some kind.
        screen_timeout_ticks = 0;
        Semaphore_post(sw_sem);
    } else if (ui_screen != UI_SCREEN_SLEEPING && ui_screen != UI_SCREEN_HUNGRY_FOR_DATA) {
        screen_timeout_ticks++;
        if (((ui_screen & UI_SCREEN_SEL_MASK) && screen_timeout_ticks > UI_TIMEOUT_MATCH_SEL) || (screen_timeout_ticks > UI_TIMEOUT_MATCH_MAIN)) {
            // Timeout.
            screen_timeout_ticks = 0;
            sw_signal = SW_SIGNAL_TIMEOUT;
            Semaphore_post(sw_sem);
        }
    }
}

void arm_color(UArg uart_id, uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i=0; i<6; i++) {
        led_buf[7+uart_id][i][0] = r;
        led_buf[7+uart_id][i][1] = g;
        led_buf[7+uart_id][i][2] = b;
    }
}

void inner_arm_color(UArg uart_id, uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i=0; i<3; i++) {
        led_buf[7+uart_id][i][0] = r;
        led_buf[7+uart_id][i][1] = g;
        led_buf[7+uart_id][i][2] = b;
    }
}

void outer_arm_color(UArg uart_id, uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i=3; i<6; i++) {
        led_buf[7+uart_id][i][0] = r;
        led_buf[7+uart_id][i][1] = g;
        led_buf[7+uart_id][i][2] = b;
    }
}

void arm_color_rgb(UArg uart_id, rgbcolor_t rgb) {
    arm_color(uart_id, rgb.red, rgb.green, rgb.blue);
}

void outer_arm_color_rgb(UArg uart_id, rgbcolor_t rgb) {
    outer_arm_color(uart_id, rgb.red, rgb.green, rgb.blue);
}

void inner_arm_color_rgb(UArg uart_id, rgbcolor_t rgb) {
    inner_arm_color(uart_id, rgb.red, rgb.green, rgb.blue);
}

void screen_blink_on(uint8_t start_off) {
    screen_blink_status = start_off;
    Clock_start(screen_blink_clock_h);
}

void screen_blink_off() {
    // Don't bother sending new `fun` data if blink_status is 0. Of course,
    // blink_status==0 implies the lights are lit, but since we invert
    // blink_status _after_ we set the bit in the `fun` buffer, actually
    // blink_status==0 means we're in the off part of a cycle.
    // also, lol, look a 3-deep stack of `blink_status` in this comment.
    if (!screen_blink_status)
        led_blank_set(0);
    Clock_stop(screen_blink_clock_h);
}

inline void screen_put_buffer(screen_frame_t *frame) {
    memcpy(led_buf, frame, 7*7*3);
}

inline void screen_put_buffer_from_flash(uint32_t frame_id) {
    Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
    if (ExtFlash_open()) {
        ExtFlash_read(FLASH_SCREEN_FRAMES_STARTPT
                      + frame_id*sizeof(screen_frame_t),
                      7*7*3, (uint8_t *) led_buf);
        ExtFlash_close();
    }
    Semaphore_post(flash_sem);
}

void set_screen_animation(size_t base, uint32_t index) {
    // Stop animating.
    Clock_stop(screen_anim_clock_h);
    Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
    while (!ExtFlash_open());
    // Load up the animation from base and index.
    ExtFlash_read(base + index*sizeof(screen_anim_t),
                  sizeof(screen_anim_t),
                  (uint8_t *) screen_anim);
    screen_frame_index = 0;
    ExtFlash_close();
    // Kick the clock back off to change frames basically immediately:
    Clock_setTimeout(screen_anim_clock_h,
                     2);
    Clock_start(screen_anim_clock_h);
    Semaphore_post(flash_sem);
    for (uint8_t i=0; i<4; i++) {
        arm_color(i, 0, 0, 0);
    }
}

void set_screen_tile(uint32_t index, uint8_t sel) {
    // Load the icon:
    // Stop animating.
    Clock_stop(screen_anim_clock_h);
    Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
    while (!ExtFlash_open());
    // Load up the animation from base and index.
    ExtFlash_read(FLASH_TILE_ANIM_LOC + index*sizeof(tile_t),
                  sizeof(tile_t),
                  (uint8_t *) &curr_tile);
    ExtFlash_close();

    memcpy(screen_anim, &curr_tile.animation, sizeof(screen_anim_t));

    screen_frame_index = (10*my_conf.csecs_of_queercon / screen_anim->anim_frame_delay_ms) % screen_anim->anim_len;

    uint32_t timeout = screen_anim->anim_frame_delay_ms - ((10*my_conf.csecs_of_queercon) % screen_anim->anim_frame_delay_ms);
    Clock_setTimeout(screen_anim_clock_h,
                     timeout * 100);
    Clock_start(screen_anim_clock_h);
    Semaphore_post(flash_sem);

    if (serial_in_progress()) {
        // If serial is in progress, the arm colors are well in hand
        //  and we do not need to clear them.
        return;
    }

    if (!sel) {
        // Turn the arms off if we're not in selection mode.
        for (uint8_t i=0; i<4; i++) {
            arm_color(i, 0, 0, 0);
        }
    } else {
        for (uint8_t i=0; i<4; i++) {
            arm_color(i, 0, 0, 0);
        }
    }
}

void switch_to_tile(uint8_t index, uint8_t unlock) {
    if (index > TILE_COUNT)
        return;

    my_conf.current_tile = index;

    if (unlock && !tile_available(index)) {
        // This is a new unlock of a tile.
        unlock_tile(index);
    }

    if (ui_screen == UI_SCREEN_TILE || ui_screen == UI_SCREEN_TILE_SEL ||
            ((ui_screen == UI_SCREEN_GAME || ui_screen == UI_SCREEN_GAME_SEL)
                    && !serial_in_progress())) {
        ui_update(UI_SCREEN_TILE);
    }
}

void set_screen_game(uint32_t index, uint8_t sel) {
    // Load the icon:
    // Stop animating.
    Clock_stop(screen_anim_clock_h);
    Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
    while (!ExtFlash_open());
    // Load up the animation from base and index.
    ExtFlash_read(FLASH_GAME_ANIM_LOC + index*sizeof(game_icon_t),
                  sizeof(game_icon_t),
                  (uint8_t *) &game_curr_icon);
    ExtFlash_close();

    memcpy(screen_anim, &game_curr_icon.animation, sizeof(screen_anim_t));

    uint32_t timeout = screen_anim->anim_frame_delay_ms;

    if (sel) {
        // If we're just selecting, start it on frame 1 so we can actually
        //  see it.
        screen_frame_index = 0;
    } else {
        // We're in the real display, so synchronize it to the centisecond
        //  clock:
        screen_frame_index = (10*my_conf.csecs_of_queercon / screen_anim->anim_frame_delay_ms) % screen_anim->anim_len;
        timeout = screen_anim->anim_frame_delay_ms - ((10*my_conf.csecs_of_queercon) % screen_anim->anim_frame_delay_ms);
    }

    // In either case, we're going to set the clock to go off when it's time
    //  to change frames. So we'll need to post to the semaphore ourselves
    //  here, since we're doing its job for the first frame.
    Semaphore_post(screen_anim_sem);

    Clock_setTimeout(screen_anim_clock_h,
                     timeout * 100);
    Clock_start(screen_anim_clock_h);
    Semaphore_post(flash_sem);


    for (uint8_t i=0; i<4; i++) {
        arm_color(i, 0, 0, 0);
        for (uint8_t i=0; i<4; i++) {
            game_arm_status[i].arm_anim_index = 0;
        }
    }
    if (!sel) {
        Clock_setTimeout(arm_anim_clock_h, ARM_ANIM_PERIOD);
        Clock_start(arm_anim_clock_h);
        Semaphore_post(arm_anim_sem);
    }

}

void set_screen_solid_local(const screen_frame_t *frame) {
    // Pre-empt the animation semaphore:
    Semaphore_pend(screen_anim_sem, BIOS_NO_WAIT);
    // Pre-emption is off, so there is no need to protect this clock
    //  with a semaphore, and this is going to be called from a Task
    //  context.
    Clock_stop(screen_anim_clock_h);
    screen_frame_index = 0;
    screen_anim->anim_len = 0;
    screen_anim->anim_frame_delay_ms = 0;
    // Don't start the clock. Just let it ride.
    screen_put_buffer((screen_frame_t *)frame);
}

uint8_t icon_available(uint8_t icon_id) {
    if (my_conf.icons_unlocked) {
        return icon_id == ICON_AIR || icon_id == ICON_EARTH ||
                icon_id == ICON_WATER || icon_id == ICON_FIRE ||
                game_been_icon(icon_id);
    }

    if (icon_id == game_starting_icon(my_conf.badge_id))
        return 1; // Can always go back to the start.

    if (icon_id == my_conf.earned_icon)
        return 1; // Can also go back to our last earned icon.

    if (icon_id == ICON_COFFEE &&
            (is_handler(my_conf.badge_id) || is_sponsor(my_conf.badge_id)))
        return 1; // Handlers and sponsors can be covfefe.

    if (icon_id == my_conf.current_icon)
        return 1; // Can always select your current icon.

    return icon_id == ICON_AIR || icon_id == ICON_EARTH ||
            icon_id == ICON_WATER || icon_id == ICON_FIRE;
}

uint8_t tile_available(uint16_t tile_id) {
    return (0x0001 << tile_id) & my_conf.avail_tiles ? 1 : 0;
}

// lol @ the efficiency of these:
void sel_next_icon() {
    do {
        sel_id = (sel_id + 1) % ICON_COUNT;
    } while (!icon_available(sel_id));
}

void sel_prev_icon() {
    do {
        sel_id = (sel_id + ICON_COUNT - 1) % ICON_COUNT;
    } while (!icon_available(sel_id));
}

// lol @ the efficiency of these:
void sel_next_tile() {
    do {
        sel_id = (sel_id + 1) % TILE_COUNT;
    } while (!tile_available(sel_id));
}

void sel_prev_tile() {
    do {
        sel_id = (sel_id + TILE_COUNT - 1) % TILE_COUNT;
    } while (!tile_available(sel_id));
}

// NB: This must be called from a Task context:
void ui_click(uint8_t sw_signal)
{
    uint8_t ui_next = ui_screen;

    // Disregard if it's a release.
    if (sw_signal == SW_SIGNAL_OPEN)
        return; // We don't care

    switch(ui_screen) {
    case UI_SCREEN_BOOT:
        return; // Should be unreachable.
    case UI_SCREEN_HUNGRY_FOR_DATA:
        ui_next = UI_SCREEN_HUNGRY_FOR_DATA_W;
        break;
    case UI_SCREEN_HUNGRY_FOR_DATA_W:
        ui_next = UI_SCREEN_HUNGRY_FOR_DATA;
        break;
    case UI_SCREEN_GAME_SEL: // Icon select
        switch(sw_signal) {
        case SW_SIGNAL_R:
            sel_next_icon();
            break;
        case SW_SIGNAL_L:
            sel_prev_icon();
            break;
        case SW_SIGNAL_C:
            // click.
            // The video updating is handled in the ui_update function,
            // but I have to handle the actual assignment and saving here.
            game_set_icon(sel_id);
            ui_next = UI_SCREEN_GAME;
        }
        break;
    case UI_SCREEN_TILE_SEL: // Tile select
        switch(sw_signal) {
        case SW_SIGNAL_R:
            // left or right
            sel_next_tile();
            break;
        case SW_SIGNAL_L:
            sel_prev_tile();
            break;
        case SW_SIGNAL_C:
            // click.
            // The video updating is handled in the ui_update function,
            // but I have to handle the actual assignment here.
            my_conf.current_tile = sel_id;
            // We don't care too terribly much about saving this, so we'll
            //  just let it get saved at our next auto-save or whenever
            //  something else changes.
            ui_next = UI_SCREEN_TILE;
            break;
        }
        break;
    case UI_SCREEN_SLEEPING: // We're asleep.
        // Doesn't matter what we click. Time to wake up like a timeout:
        ui_screen = UI_SCREEN_SLEEP;
        ui_timeout();
        return;
    default: // We are in one of the switchable versions:
        switch(sw_signal) {
        case SW_SIGNAL_L:
            ui_next = (ui_screen + 2) % 3; // Go left.
            break;
        case SW_SIGNAL_R:
            ui_next = (ui_screen + 1) % 3; // Go right.
            break;
        default: // click
            sel_id = ((ui_screen == UI_SCREEN_GAME) ?
                        my_conf.current_icon : my_conf.current_tile);
            ui_next = ui_screen | UI_SCREEN_SEL_MASK;
        }
    }

    ui_update(ui_next);
}

void ui_timeout() {
    uint8_t ui_next = ui_screen;
    // Are we in a SEL mode? If so, timeout back to the non-SEL version.
    // This can't be called while we're sleeping, because we don't do timeouts
    // from inside the switch clock SWI when sleeping.
    if (ui_next == UI_SCREEN_HUNGRY_FOR_DATA) {
        // We don't time out from this.
        return;
    } else if (ui_next == UI_SCREEN_HUNGRY_FOR_DATA_W) {
        ui_next = UI_SCREEN_HUNGRY_FOR_DATA;
    } else if (ui_next & UI_SCREEN_SEL_MASK) {
        ui_next &= ~UI_SCREEN_SEL_MASK;
    } else if (((my_conf.csecs_of_queercon > POOL_TIME &&
            my_conf.csecs_of_queercon < POOL_OVER_TIME) ||
            (my_conf.csecs_of_queercon > CLUB_TIME &&
                    my_conf.csecs_of_queercon < CLUB_OVER_TIME))) {
        // We're at a party - timeout to tile.
        ui_next = UI_SCREEN_TILE;
    } else { // Not at a party, not hungry for data. Timeout is game.
        ui_next = UI_SCREEN_GAME;
    }
    // Update the stuff to display the correct things:
    if (ui_next != ui_screen)
        ui_update(ui_next);
}

void ui_update(uint8_t ui_next) {
    screen_blink_off();
    switch(ui_next) { // Destination screen:
    case UI_SCREEN_SLEEPING:
        // Shut it down. Shut everything down.
        set_screen_solid_local(&all_off);
        arm_color(0, 0, 0, 0);
        arm_color(1, 0, 0, 0);
        arm_color(2, 0, 0, 0);
        arm_color(3, 0, 0, 0);
        break;
    case UI_SCREEN_SLEEP:
        screen_blink_on(0);
        set_screen_solid_local(&power_bmp);
        arm_color(0, 0, 0, 0);
        arm_color(1, 0, 0, 0);
        arm_color(2, 0, 0, 0);
        arm_color(3, 0, 0, 0);
        break;
    case UI_SCREEN_BOOT:
        set_screen_animation(FLASH_BOOT_ANIM_LOC, 0);
        break;
    case UI_SCREEN_HUNGRY_FOR_DATA:
        screen_blink_on(0);
        set_screen_solid_local(&needflash_icon);
        for (uint8_t i=0; i<4; i++)
            arm_color(i, 0x00, 0x00, 0x00);
        break;
    case UI_SCREEN_HUNGRY_FOR_DATA_W:
        memset(led_buf, 0xff, 7*7*3);
        for (uint8_t i=0; i<4; i++)
            arm_color(i, 0xff, 0xff, 0xff);
        break;
    case UI_SCREEN_GAME_SEL:
        screen_blink_on(ui_screen != ui_next);
        set_screen_game(sel_id, 1);
        break;
    case UI_SCREEN_GAME:
        set_screen_game(my_conf.current_icon, 0);
        break;
    case UI_SCREEN_TILE_SEL:
        screen_blink_on(ui_screen != ui_next);
        set_screen_tile(sel_id, 1);
        break;
    case UI_SCREEN_TILE:
        set_screen_tile(my_conf.current_tile, 0);
        break;
    }
    ui_screen = ui_next;
}

void do_animation_loop_body(uint8_t csecs_sync) {
    screen_put_buffer_from_flash(screen_anim->anim_start_frame
                                 + screen_frame_index);

    int16_t offset_frames = 0;
    if (ui_screen == UI_SCREEN_TILE && tile_active && tile_offset) {
        offset_frames = tile_offsets[my_conf.current_tile] * tile_offset;
        offset_frames = offset_frames % screen_anim->anim_len; // signed.
        if (offset_frames < 0)
            // Make it positive. This is probably irrelevant because of how
            //  enormous csecs_of_queercon is, but whatever.
            offset_frames = screen_anim->anim_len + offset_frames;
    }

    if (screen_frame_index < screen_anim->anim_len) {
        if (csecs_sync) {
            screen_frame_index = (10*my_conf.csecs_of_queercon / screen_anim->anim_frame_delay_ms + offset_frames) % screen_anim->anim_len;
            Clock_setTimeout(screen_anim_clock_h,
                             screen_anim->anim_frame_delay_ms - ((10*my_conf.csecs_of_queercon) % screen_anim->anim_frame_delay_ms));
            Clock_start(screen_anim_clock_h);
        } else {
            screen_frame_index++;
            Clock_setTimeout(screen_anim_clock_h,
                             screen_anim->anim_frame_delay_ms * 100);
            Clock_start(screen_anim_clock_h);
        }
    }
}

void do_animation_loop() {
    while (screen_frame_index < screen_anim->anim_len) {
        Semaphore_pend(screen_anim_sem, BIOS_WAIT_FOREVER);
        do_animation_loop_body(0);
    }
}

inline void bootup_sequence() {
    ui_screen = UI_SCREEN_BOOT;
    // Load the starting animation.
    set_screen_animation(FLASH_BOOT_ANIM_LOC, 0);

    if (screen_anim->anim_start_frame == 0xffffffff) { // sentinel for unprog
        // Badge is not programmed. We're going to flash our hunger.
        ui_screen = UI_SCREEN_HUNGRY_FOR_DATA_W;
    } else {
        // Badge has flash data. Do the intro animation, then
        //  switch to game mode.
        do_animation_loop();
    }
}

void screen_anim_task_fn(UArg a0, UArg a1) {
    // Bootup animation time!
    bootup_sequence(); // Only returns if we're programmed.

    // Now that we've showed off our screen, time to start the badge.
    start_badge();
    ui_timeout(); // Pick the destination based on what time we think it is.

    Clock_setTimeout(arm_anim_clock_h, ARM_ANIM_PERIOD);
    Clock_start(arm_anim_clock_h);

    uint8_t arm_anim_index = 0;

    while (1) {
        // Handle user input:
        if (Semaphore_pend(sw_sem, BIOS_NO_WAIT)) {
            if (sw_signal == SW_SIGNAL_TIMEOUT) {
                ui_timeout();
            } else {
                ui_click(sw_signal);
            }
        }

        if (Semaphore_pend(club_sem, BIOS_NO_WAIT)) {
            // time for the club tile
            switch_to_tile(TILE_HEARTONBLACK, 1);
        }

        if (Semaphore_pend(pool_sem, BIOS_NO_WAIT)) {
            // time for the pool tile
            switch_to_tile(TILE_RAINBOWBOOM, 1);
        }

        if (Semaphore_pend(unlock_sem, BIOS_NO_WAIT)) {
            // Time to unlock icons.
            my_conf.icons_unlocked = 1;
            Semaphore_post(save_sem);
        }

        // Handle animations:
        if (Semaphore_pend(screen_anim_sem, BIOS_NO_WAIT)) {
            do_animation_loop_body(ui_screen == UI_SCREEN_TILE);

            if (screen_frame_index == screen_anim->anim_len) {
                if (ui_screen == UI_SCREEN_BOOT) {
                    // Currently this is unreachable.
                } else {
                    screen_frame_index = 0;
                    Clock_setTimeout(screen_anim_clock_h,
                                     (screen_anim->anim_frame_delay_ms - ((10*my_conf.csecs_of_queercon) % screen_anim->anim_frame_delay_ms)) * 100);
                    Clock_start(screen_anim_clock_h);
                }
            }
        }

        if (Semaphore_pend(arm_anim_sem, BIOS_NO_WAIT)) {
            Clock_setTimeout(arm_anim_clock_h, ARM_ANIM_PERIOD);

            if (ui_screen == UI_SCREEN_GAME) {
                for (uint8_t i=0; i<4; i++) {
                    inner_arm_color(i, 0, 0, 0);
                    if (!game_curr_icon.arms[i].sufficient_flag || !game_arm_status[i].connectable)
                        continue;

                    inner_arm_color_rgb(i, game_curr_icon.arms[i].arm_color);

                    uint8_t loopat = 8;

                    if (game_arm_status[i].arm_anim_index < 3) {
                        led_buf[7+i][game_arm_status[i].arm_anim_index][0] /= 4;
                        led_buf[7+i][game_arm_status[i].arm_anim_index][1] /= 4;
                        led_buf[7+i][game_arm_status[i].arm_anim_index][2] /= 4;
                    }

                    if (!game_arm_status[i].arm_anim_dir)
                        game_arm_status[i].arm_anim_index = (game_arm_status[i].arm_anim_index + 1) % loopat;
                    else
                        game_arm_status[i].arm_anim_index = (game_arm_status[i].arm_anim_index + loopat-1) % loopat;
                }
            } else if (ui_screen == UI_SCREEN_TILE) {
                // The tiles kinda look best with blank arms...
                //  animate them when we're plugged in.

                uint8_t loopat = 4;

                for (uint8_t i=0; i<4; i++) {
                    if (!tile_arm_status[i].connected)
                        continue;

                    inner_arm_color(i, 0, 0, 0);
                    if (arm_anim_index < 3) {
                        led_buf[7+i][(tile_frame_periods[my_conf.current_tile][i] >= 0) ? arm_anim_index : 2-arm_anim_index][0] = 5;
                        led_buf[7+i][(tile_frame_periods[my_conf.current_tile][i] >= 0) ? arm_anim_index : 2-arm_anim_index][1] = 5;
                        led_buf[7+i][(tile_frame_periods[my_conf.current_tile][i] >= 0) ? arm_anim_index : 2-arm_anim_index][2] = 5;
                    }
                }

                arm_anim_index = (arm_anim_index + 1) % loopat;
                Clock_setTimeout(arm_anim_clock_h, ARM_ANIM_PERIOD/2);
            }
            Clock_start(arm_anim_clock_h);
        }

        // Save if necessary.
        if (Semaphore_pend(save_sem, BIOS_NO_WAIT))
            qc14conf_save();

        Task_yield();
    }
}

void ui_init() {
    sw_pin_h = PIN_open(&sw_pin_state, sw_pin_table);

    Clock_Params clockParams;
    Error_Block eb;
    Error_init(&eb);
    Clock_Params_init(&clockParams);
    clockParams.period = UI_CLOCK_TICKS;
    clockParams.startFlag = TRUE;
    sw_debounce_clock = Clock_create(sw_clock_swi, 2, &clockParams, &eb);
}

void screen_init() {
    Semaphore_Params params;
    Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
    screen_anim_sem = Semaphore_create(0, &params, NULL);
    Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
    arm_anim_sem = Semaphore_create(0, &params, NULL);

    Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
    flash_sem = Semaphore_create(1, &params, NULL);

    Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
    sw_sem = Semaphore_create(0, &params, NULL);

    Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
    save_sem = Semaphore_create(0, &params, NULL);

    Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
    unlock_sem = Semaphore_create(0, &params, NULL);
    Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
    pool_sem = Semaphore_create(0, &params, NULL);
    Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
    club_sem = Semaphore_create(0, &params, NULL);

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = screen_anim_task_stack;
    taskParams.stackSize = sizeof(screen_anim_task_stack);
    taskParams.priority = 1;
    Task_construct(&screen_anim_task, screen_anim_task_fn, &taskParams, NULL);

    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.period = 0; // One-shot clock.
    clockParams.startFlag = FALSE;
    screen_anim_clock_h = Clock_create(screen_anim_tick_swi, 100, &clockParams, NULL); // Wait 100 ticks (1ms) before firing for the first time.

    Clock_Params_init(&clockParams);
    clockParams.period = 0;
    clockParams.startFlag = FALSE;
    arm_anim_clock_h = Clock_create(arm_anim_tick_swi, ARM_ANIM_PERIOD, &clockParams, NULL);

    Clock_Params blink_clock_params;
    Clock_Params_init(&blink_clock_params);
    blink_clock_params.period = BLINK_LEN; // 500 ms recurring
    blink_clock_params.startFlag = FALSE; // Don't auto-start (only when we blink-on)
    screen_blink_clock_h = Clock_create(screen_blink_tick_swi, 0, &blink_clock_params, NULL);

    Clock_Params csecs_clock_params;
    Clock_Params_init(&csecs_clock_params);
    csecs_clock_params.period = 1000; // 10 ms recurring ( 1 centisecond)
    csecs_clock_params.startFlag = FALSE; // Auto-start off.
    csecs_clock_h = Clock_create(csecs_swi, 1000, &csecs_clock_params, NULL);
}

