/*
 * ui.h
 *
 *  Created on: Jun 25, 2017
 *      Author: George
 */

#ifndef STARTUP_UI_H_
#define STARTUP_UI_H_

#define UI_SCREEN_SEL_MASK 0x10

#define UI_SCREEN_BOOT 0xf0
#define UI_SCREEN_HUNGRY_FOR_DATA 0xe0
#define UI_SCREEN_HUNGRY_FOR_DATA_W 0xd0
#define UI_SCREEN_GAME 0x00
#define UI_SCREEN_GAME_SEL (UI_SCREEN_GAME | UI_SCREEN_SEL_MASK)
#define UI_SCREEN_TILE 0x01
#define UI_SCREEN_TILE_SEL (UI_SCREEN_TILE | UI_SCREEN_SEL_MASK)
#define UI_SCREEN_SLEEP 0x02
#define UI_SCREEN_SLEEPING 0x12

// Derived UI configuration
#define UI_CLOCK_TICKS (UI_CLOCK_MS * 100)
#define UI_TIMEOUT_MATCH_SEL (1000 * UI_SEL_TIMEOUT_SEC / UI_CLOCK_MS)
#define UI_TIMEOUT_MATCH_MAIN (1000 * UI_MAIN_TIMEOUT_SEC / UI_CLOCK_MS)

void ui_click(uint8_t sw_signal);
void ui_init();
void ui_timeout();
void arm_color(UArg uart_id, uint8_t r, uint8_t g, uint8_t b);
extern uint8_t ui_screen;

#include <ti/sysbios/knl/Semaphore.h>

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgbcolor_t;

typedef struct {
    rgbcolor_t pixels[7][7];
    uint8_t pad;
} screen_frame_t;

typedef struct {
    rgbcolor_t pixels[6];
} arm_frame_t;

typedef struct {
    uint32_t anim_start_frame;
    uint16_t anim_len;
    uint16_t anim_frame_delay_ms;
} screen_anim_t;

typedef struct {
    uint16_t mate_icon_id;
    uint16_t result_icon_id;
    uint16_t arm_anim_id;
    uint8_t sufficient_flag;
    uint8_t other_arm_id;
} mate_spec_t;

typedef struct {
    uint16_t id;
    screen_anim_t animation;
    mate_spec_t arms[4];
} game_icon_t;

extern Semaphore_Handle flash_sem;

void screen_init();
void screen_blink_on(uint8_t start_off);
void screen_blink_off();
void screen_update_now();

#endif /* STARTUP_UI_H_ */
