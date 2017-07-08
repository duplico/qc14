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

void screen_init();
void screen_blink_on();
void screen_blink_off();
void screen_update_now();

#endif /* STARTUP_SCREEN_H_ */
