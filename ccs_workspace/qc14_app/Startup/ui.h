/*
 * ui.h
 *
 *  Created on: Jun 25, 2017
 *      Author: George
 */

#ifndef STARTUP_UI_H_
#define STARTUP_UI_H_

#define UI_SCREEN_GAME 0x00
#define UI_SCREEN_GAME_SEL 0x10
#define UI_SCREEN_TILE 0x01
#define UI_SCREEN_TILE_SEL 0x11
#define UI_SCREEN_SLEEP 0x02
#define UI_SCREEN_SLEEPING 0x12

#define UI_SCREEN_SWITCHABLE_MASK 0x10

#define UI_SCREEN_SWITCHABLE (ui_screen & UI_SCREEN_SWITCHABLE_MASK)

void ui_click(uint8_t sw_signal);
void ui_init();

#endif /* STARTUP_UI_H_ */
