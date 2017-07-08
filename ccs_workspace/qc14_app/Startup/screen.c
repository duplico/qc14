/*
 * screen.c
 *
 *  Created on: Jun 12, 2017
 *      Author: George
 */
#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Swi.h>

#include "tlc_driver.h"
#include "ui.h"

extern uint8_t led_buf[11][7][3];

Task_Struct screen_anim_task;
char screen_anim_task_stack[256];

Clock_Handle screen_anim_clock_h;
Clock_Handle screen_blink_clock_h;

Semaphore_Handle anim_sem;

void screen_update_now() {
    Clock_stop(screen_anim_clock_h);
    Semaphore_post(anim_sem);
}

void screen_anim_tick(UArg a0) {
    Semaphore_post(anim_sem);
}

volatile uint8_t blink_status = 0;

void screen_blink_on() {
    blink_status = 1;
    Clock_start(screen_blink_clock_h);
}

void screen_blink_off() {
    // Don't bother sending new `fun` data if blink_status is 0. Of course,
    // blink_status==0 implies the lights are lit, but since we invert
    // blink_status _after_ we set the bit in the `fun` buffer, actually
    // blink_status==0 means we're in the off part of a cycle.
    // also, lol, look a 3-deep stack of `blink_status` in this comment.
    if (!blink_status)
        led_blank_set(0);
    Clock_stop(screen_blink_clock_h);
}

void screen_blink_tick(UArg a0) {
    led_blank_set(blink_status);
    blink_status = !blink_status;
}

uint8_t rainbow_colors[6][3] = {{255,0,0}, {255,30,0}, {255,255,0}, {0,255,0}, {0,0,255}, {98,0,255}};

uint8_t game_bmp[7][7][3] = {{{0, 0, 0}, {0, 0, 0}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 0, 0}}, {{0, 140, 255}, {0, 0, 0}, {0, 0, 0}, {0, 140, 255}, {0, 0, 0}, {0, 0, 0}, {0, 140, 255}}, {{0, 140, 255}, {0, 0, 0}, {99, 58, 148}, {0, 140, 255}, {0, 0, 0}, {99, 58, 148}, {0, 140, 255}}, {{0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}}, {{0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}}, {{0, 140, 255}, {0, 0, 0}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 0, 0}, {0, 140, 255}}};

uint8_t power_bmp[7][7][3] = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}}};
uint8_t tile_placeholder[7][7][3] = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
//uint8_t game_placeholder[7][7][3] = {{{0, 0, 0}, {0, 0, 0}, {11, 11, 11}, {255, 255, 255}, {11, 11, 11}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {105, 105, 105}, {64, 64, 64}, {104, 104, 104}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {1, 1, 1}, {170, 170, 170}, {0, 0, 0}, {174, 174, 174}, {1, 1, 1}, {0, 0, 0}}, {{0, 0, 0}, {54, 54, 54}, {35, 35, 35}, {0, 0, 0}, {36, 36, 36}, {53, 53, 53}, {0, 0, 0}}, {{0, 0, 0}, {208, 208, 208}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {208, 208, 208}, {0, 0, 0}}, {{22, 22, 22}, {81, 81, 81}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {81, 81, 81}, {21, 21, 21}}, {{142, 142, 142}, {5, 5, 5}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {5, 5, 5}, {141, 141, 141}}};

void screen_anim_task_fn(UArg a0, UArg a1) {

    while (1) {
        Semaphore_pend(anim_sem, BIOS_WAIT_FOREVER);
        switch(ui_screen) {
        case UI_SCREEN_GAME:
        case UI_SCREEN_GAME_SEL:
            memcpy(led_buf, game_bmp, sizeof game_bmp);
            break;
        case UI_SCREEN_TILE:
        case UI_SCREEN_TILE_SEL:
            memcpy(led_buf, tile_placeholder, sizeof tile_placeholder);
            break;
        case UI_SCREEN_SLEEP:
            memcpy(led_buf, power_bmp, sizeof power_bmp);
            break;
        case UI_SCREEN_SLEEPING:
            memset(led_buf, 0, sizeof led_buf);
        }

        Clock_setTimeout(screen_anim_clock_h, 500000); // 5 seconds
        Clock_start(screen_anim_clock_h);
    }
}

void screen_init() {
    Semaphore_Params params;
    Semaphore_Params_init(&params);
    anim_sem = Semaphore_create(0, &params, NULL);

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = screen_anim_task_stack;
    taskParams.stackSize = sizeof(screen_anim_task_stack);
    taskParams.priority = 1;
    Task_construct(&screen_anim_task, screen_anim_task_fn, &taskParams, NULL);

    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.period = 0; // One-shot clock.
    clockParams.startFlag = TRUE;
    screen_anim_clock_h = Clock_create(screen_anim_tick, 100, &clockParams, NULL); // Wait 100 ticks (1ms) before firing for the first time.

    Clock_Params blink_clock_params;
    Clock_Params_init(&blink_clock_params);
    blink_clock_params.period = 50000; // 500 ms recurring
    blink_clock_params.startFlag = FALSE; // Don't auto-start (only when we blink-on)
    screen_blink_clock_h = Clock_create(screen_blink_tick, 0, &blink_clock_params, NULL);
}
