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

extern uint8_t led_buf[11][7][3];
void ui_update(uint8_t ui_next);

// OS and Task constructs:
char screen_anim_task_stack[768];
Task_Struct screen_anim_task; // Main UI task

Clock_Handle screen_anim_clock_h;  // Ticks when we need a new screen
void screen_anim_tick_swi(UArg a0);

Clock_Handle screen_blink_clock_h; // Ticks when we're blinking
void screen_blink_tick_swi(UArg a0);

Clock_Handle sw_debounce_clock; // Ticks to debounce the switches
void sw_clock_swi(UArg a0);

Clock_Handle csecs_clock_h; // Ticks to set the centisecond clock
void csecs_swi(UArg a0);

Semaphore_Handle anim_sem; // Posted when we need a new screen
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

void screen_anim_tick_swi(UArg a0) {
    Semaphore_post(anim_sem);
}
void screen_blink_tick_swi(UArg a0) {
    led_blank_set(screen_blink_status);
    screen_blink_status = !screen_blink_status;
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
}

void set_screen_tile(uint32_t index) {
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
    // Kick the clock back off to change frames basically immediately:
    Clock_setTimeout(screen_anim_clock_h,
                     timeout * 100);
    Clock_start(screen_anim_clock_h);
    Semaphore_post(flash_sem);

    // TODO: Set arms.

}

void set_screen_game(uint32_t index) {
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

    screen_frame_index = (10*my_conf.csecs_of_queercon / screen_anim->anim_frame_delay_ms) % screen_anim->anim_len;

    uint32_t timeout = screen_anim->anim_frame_delay_ms - ((10*my_conf.csecs_of_queercon) % screen_anim->anim_frame_delay_ms);
    // Kick the clock back off to change frames basically immediately:
    Clock_setTimeout(screen_anim_clock_h,
                     timeout * 100);
    Clock_start(screen_anim_clock_h);
    Semaphore_post(flash_sem);

    // TODO: Set arms.

}

void set_screen_solid_local(const screen_frame_t *frame) {
    // Pre-empt the animation semaphore:
    Semaphore_pend(anim_sem, BIOS_NO_WAIT);
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

void arm_color(UArg uart_id, uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i=0; i<6; i++) {
        led_buf[7+uart_id][i][0] = r;
        led_buf[7+uart_id][i][1] = g;
        led_buf[7+uart_id][i][2] = b;
    }
}

uint8_t icon_available(uint8_t icon_id) {
    return 1; // TODO
    if (my_conf.icons_unlocked) {
        return game_been_icon(icon_id);
    }

    if (icon_id == game_starting_icon(my_conf.badge_id))
        return 1; // Can always go back to the start.

    if (icon_id == ICON_COFFEE_ID &&
            (is_handler(my_conf.badge_id) || is_sponsor(my_conf.badge_id)))
        return 1; // Handlers and sponsors can be covfefe.

    if (icon_id == my_conf.current_icon)
        return 1; // Can always select your current icon.

    return 0;
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
        sel_id = (sel_id + 1) % ICON_COUNT;
    } while (!tile_available(sel_id));
}

void sel_prev_tile() {
    do {
        sel_id = (sel_id + ICON_COUNT - 1) % ICON_COUNT;
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
        break;
    case UI_SCREEN_SLEEP:
        screen_blink_on(0);
        set_screen_solid_local(&power_bmp);
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
        set_screen_game(sel_id);
        break;
    case UI_SCREEN_GAME:
        set_screen_game(my_conf.current_icon);
        break;
    case UI_SCREEN_TILE_SEL:
        screen_blink_on(ui_screen != ui_next);
        set_screen_tile(sel_id);
        break;
    case UI_SCREEN_TILE:
        set_screen_tile(my_conf.current_tile);
        break;
    }
    ui_screen = ui_next;
}

void do_animation_loop_body() {
    screen_put_buffer_from_flash(screen_anim->anim_start_frame
                                 + screen_frame_index);
    screen_frame_index++;
    if (screen_frame_index < screen_anim->anim_len) {
        Clock_setTimeout(screen_anim_clock_h,
                         screen_anim->anim_frame_delay_ms * 100);
        Clock_start(screen_anim_clock_h);
    }
}

void do_animation_loop() {
    while (screen_frame_index < screen_anim->anim_len) {
        Semaphore_pend(anim_sem, BIOS_WAIT_FOREVER);
        do_animation_loop_body();
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
            // TODO: Unlock tile
            my_conf.current_tile = 1; // TODO!!!
            if (ui_screen != UI_SCREEN_SLEEPING && !serial_in_progress()) {
                ui_update(UI_SCREEN_TILE);
            }
        }

        if (Semaphore_pend(pool_sem, BIOS_NO_WAIT)) {
            // time for the pool tile
            // TODO: Unlock tile
            my_conf.current_tile = 2; // TODO!!!
            if (ui_screen != UI_SCREEN_SLEEPING && !serial_in_progress()) {
                ui_update(UI_SCREEN_TILE);
            }
        }

        if (Semaphore_pend(unlock_sem, BIOS_NO_WAIT)) {
            // Time to unlock icons.
            my_conf.icons_unlocked = 1;
            Semaphore_post(save_sem);
        }

        // Handle animations:
        if (Semaphore_pend(anim_sem, BIOS_NO_WAIT)) {
            do_animation_loop_body();

            if (screen_frame_index == screen_anim->anim_len) {
                if (ui_screen == UI_SCREEN_BOOT) {
                    // Currently this is unreachable.
                } else {
                    screen_frame_index = 0;
                    Clock_setTimeout(screen_anim_clock_h,
                                     screen_anim->anim_frame_delay_ms * 100);
                    Clock_start(screen_anim_clock_h);
                }
            }
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
    anim_sem = Semaphore_create(0, &params, NULL);

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

    Clock_Params blink_clock_params;
    Clock_Params_init(&blink_clock_params);
    blink_clock_params.period = 50000; // 500 ms recurring
    blink_clock_params.startFlag = FALSE; // Don't auto-start (only when we blink-on)
    screen_blink_clock_h = Clock_create(screen_blink_tick_swi, 0, &blink_clock_params, NULL);

    Clock_Params csecs_clock_params;
    Clock_Params_init(&csecs_clock_params);
    csecs_clock_params.period = 1000; // 10 ms recurring ( 1 centisecond)
    csecs_clock_params.startFlag = FALSE; // Auto-start off.
    csecs_clock_h = Clock_create(csecs_swi, 1000, &csecs_clock_params, NULL);
}

