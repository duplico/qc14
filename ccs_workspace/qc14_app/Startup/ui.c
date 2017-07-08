/*
 * ui.c
 *
 *  Created on: Jun 25, 2017
 *      Author: George
 */
#include <stdint.h>

#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

#include "qc14.h"
#include "ui.h"
#include "screen.h"

uint8_t ui_screen = UI_SCREEN_GAME;

uint8_t click_signal = SW_SIGNAL_NONE;

uint8_t sw_l_clicked = 0;
uint8_t sw_r_clicked = 0;
uint8_t sw_c_clicked = 0;

uint_fast32_t timeout_ticks = 0;

void sw_clock_f(UArg a0) {
    static uint8_t sw_l_last = 1;
    static uint8_t sw_r_last = 1;
    static uint8_t sw_c_last = 1;

    uint8_t sw_l_curr = PIN_getInputValue(SW_L);
    uint8_t sw_r_curr = PIN_getInputValue(SW_R);
    uint8_t sw_c_curr = PIN_getInputValue(SW_CLICK);

    click_signal = SW_SIGNAL_NONE;

    if (!sw_l_curr && !sw_l_last && !sw_l_clicked) {
        // left clicked
        sw_l_clicked = 1;
        click_signal = SW_SIGNAL_L;
        // do stuff
    } else if (sw_l_curr && sw_l_last && sw_l_clicked) {
        sw_l_clicked = 0;
        click_signal = SW_SIGNAL_OPEN;
        // unclicked.
    }
    sw_l_last = sw_l_curr;

    if (!sw_r_curr && !sw_r_last && !sw_r_clicked) {
        // right clicked
        sw_r_clicked = 1;
        click_signal = SW_SIGNAL_R;
        // do stuff
    } else if (sw_r_curr && sw_r_last && sw_r_clicked) {
        sw_r_clicked = 0;
        click_signal = SW_SIGNAL_OPEN;
        // unclicked.
    }
    sw_r_last = sw_r_curr;

    if (!sw_c_curr && !sw_c_last && !sw_c_clicked) {
        // click clicked
        sw_c_clicked = 1;
        click_signal = SW_SIGNAL_C;
        // do stuff
    } else if (sw_c_curr && sw_c_last && sw_c_clicked) {
        sw_c_clicked = 0;
        click_signal = SW_SIGNAL_OPEN;
        // unclicked.
    }
    sw_c_last = sw_c_curr;

    if (click_signal != SW_SIGNAL_NONE) {
        // User interaction of some kind.
        timeout_ticks = 0;
        ui_click(click_signal);
    } else if (ui_screen != UI_SCREEN_SLEEPING) {
        timeout_ticks++;
        if (((ui_screen & UI_SCREEN_SEL_MASK) && timeout_ticks > UI_TIMEOUT_MATCH_SEL) || (timeout_ticks > UI_TIMEOUT_MATCH_MAIN)) {
            ui_timeout();
            timeout_ticks = 0;
        }
    }
}

Clock_Handle sw_clock;
void ui_update();

void ui_click(uint8_t sw_signal)
{
    // Disregard if mated.
    if (uart_proto_state[0] || uart_proto_state[1] || uart_proto_state[2] || uart_proto_state[3])
        return; // No UI during mating.

    // Disregard if it's a release.
    if (sw_signal == SW_SIGNAL_OPEN)
        return; // We don't care

    switch(ui_screen) {
    case UI_SCREEN_GAME_SEL: // Icon select
        if (sw_signal & SW_SIGNAL_DIR_MASK)
            screen_blink_off();
        break;
    case UI_SCREEN_TILE_SEL: // Tile select
        if (sw_signal & SW_SIGNAL_DIR_MASK)
            screen_blink_off();
        break;
    case UI_SCREEN_SLEEPING: // We're asleep.
        // Doesn't matter what we click. Time to wake up and go to UI_SCREEN_SLEEP:
        if (sw_signal & SW_SIGNAL_DIR_MASK)
            screen_blink_off();
        break;
    default: // We are in one of the switchable versions:
        if (sw_signal & SW_SIGNAL_DIR_MASK) { // Left or right was clicked.
            if (sw_signal == SW_SIGNAL_L)
                ui_screen = (ui_screen + 2) % 3; // Go left.
            else
                ui_screen = (ui_screen + 1) % 3; // Go right.
        } else { // CLICK was clicked, and screen is switchable (so goto clicked version):
            ui_screen = ui_screen | UI_SCREEN_SEL_MASK;
            screen_blink_on();
        }
    }

    ui_update();
}

void ui_timeout() {
    if (ui_screen == UI_SCREEN_GAME)
        return; // Nothing to do.

    ui_screen = UI_SCREEN_GAME;
    ui_update();
}

void ui_update() {
    screen_update_now();
}

void ui_init() {
    Clock_Params clockParams;
    Error_Block eb;
    Error_init(&eb);
    Clock_Params_init(&clockParams);
    clockParams.period = UI_CLOCK_TICKS;
    clockParams.startFlag = TRUE;
    sw_clock = Clock_create(sw_clock_f, 2, &clockParams, &eb);
}
