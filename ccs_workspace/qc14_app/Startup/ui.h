/*
 * ui.h
 *
 *  Created on: Jun 25, 2017
 *      Author: George
 */

#ifndef STARTUP_UI_H_
#define STARTUP_UI_H_

#define UI_SCREEN_SEL_MASK 0x10

#define UI_SCREEN_BOOT 0x80
#define UI_SCREEN_HUNGRY_FOR_DATA 0x40
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

Clock_Handle csecs_clock_h;
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
    uint8_t sufficient_flag; // 0 if non-connection.
    uint8_t other_arm_id;
    rgbcolor_t arm_color;
    // Probably a pad byte here.
} mate_spec_t;

typedef struct {
    uint16_t id;
    screen_anim_t animation;
    mate_spec_t arms[4];
} game_icon_t;

typedef struct {
    screen_anim_t animation;
} tile_t;

typedef struct {
    uint8_t connectable;
    uint8_t connected;
    uint16_t icon_id;
    uint8_t sufficiency_info;
    uint8_t nts;
    uint8_t nts_done;
} game_arm_status_t;

#define ARM_CONNECT_STATUS_DIS 0
#define ARM_CONNECT_STATUS_DONE 1
#define ARM_CONNECT_STATUS_WAITMSG 2 // Waiting on a message from
#define ARM_CONNECT_STATUS_WAITCON 3

extern game_arm_status_t game_arm_status[4];
extern Semaphore_Handle flash_sem;
extern game_icon_t game_curr_icon;

void screen_init();
void screen_blink_on(uint8_t start_off);
void screen_blink_off();
void screen_update_now();
void ui_click(uint8_t sw_signal);
void ui_init();
void ui_timeout();
void arm_color(UArg uart_id, uint8_t r, uint8_t g, uint8_t b);
void outer_arm_color(UArg uart_id, uint8_t r, uint8_t g, uint8_t b);
void arm_color_rgb(UArg uart_id, rgbcolor_t rgb);
void its_cold();
void its_bright();
void set_screen_animation(size_t base, uint32_t index);
void do_animation_loop();
void do_icon_transition(uint16_t dest_icon);

extern Semaphore_Handle save_sem;

#endif /* STARTUP_UI_H_ */
