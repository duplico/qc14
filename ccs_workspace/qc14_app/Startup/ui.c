
// TODO: Handle timeouts.
// TODO: We probably need a blinking timeloop.
// TODO: Probably merge this with the screen module.

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

#include "ui.h"
#include "qc14.h"

uint8_t ui_screen = UI_SCREEN_GAME;

uint8_t click_signal = SW_SIGNAL_OPEN;

uint8_t sw_l_clicked = 0;
uint8_t sw_r_clicked = 0;
uint8_t sw_c_clicked = 0;

void sw_clock_f() {
    static uint8_t sw_l_last = 1;
    static uint8_t sw_r_last = 1;
    static uint8_t sw_c_last = 1;

    uint8_t sw_l_curr = PIN_getInputValue(SW_L);
    uint8_t sw_r_curr = PIN_getInputValue(SW_R);
    uint8_t sw_c_curr = PIN_getInputValue(SW_CLICK);

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

    if (click_signal) {
        // User interaction of some kind.
    }
}

Clock_Handle sw_clock;

void ui_click(uint8_t sw_signal)
{
    // TODO: Disregard if mated.
    // Disregard if it's a release.
    if (sw_signal == SW_SIGNAL_OPEN)
        return; // We don't care

    // Left or right, and we can switch:
    if ((sw_signal & SW_SIGNAL_DIR_MASK) && UI_SCREEN_SWITCHABLE) {
        if (sw_signal == SW_SIGNAL_L)
            ui_screen = (ui_screen + 2) % 3;
        else
            ui_screen = (ui_screen + 1) % 3;
    } else if (UI_SCREEN_SWITCHABLE) { // Clicked, and screen is switchable (so go to the clicked version).
        ui_screen = ui_screen | UI_SCREEN_SWITCHABLE_MASK;
    }

    // OK, now every option from the main "bottom 3" menus have been

    // TODO: Do the graphics update.
}

void ui_init() {
    Clock_Params clockParams;
    Error_Block eb;
    Error_init(&eb);
    Clock_Params_init(&clockParams);
    clockParams.period = 100;
    clockParams.startFlag = TRUE;
    sw_clock = Clock_create(sw_clock_f, 2, &clockParams, &eb);
}
