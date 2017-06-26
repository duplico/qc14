/*
 * ui.c
 *
 *  Created on: Jun 25, 2017
 *      Author: George
 */
#include <stdint.h>

#include "ui.h"
#include "qc14.h"

uint8_t ui_screen = UI_SCREEN_GAME;

// TODO: Handle timeouts.

// TODO: We probably need a blinking timeloop.

// TODO: Probably merge this with the screen module.

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
