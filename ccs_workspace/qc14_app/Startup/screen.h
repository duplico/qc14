/*
 * screen.h
 *
 *  Created on: Jun 12, 2017
 *      Author: George
 */

#ifndef STARTUP_SCREEN_H_
#define STARTUP_SCREEN_H_

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

void screen_init();
void screen_blink_on();
void screen_blink_off();
void screen_update_now();

#endif /* STARTUP_SCREEN_H_ */
