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

extern uint8_t led_buf[11][7][3];

Task_Struct screen_anim_task;
char screen_anim_task_stack[256];

Clock_Handle screen_anim_clock_h;

Semaphore_Handle anim_sem;

void screen_update_now() {
    Clock_stop(screen_anim_clock_h);
    Semaphore_post(anim_sem);
}

void screen_anim_tick(UArg a0) {
    Semaphore_post(anim_sem);
}

uint8_t rainbow_colors[6][3] = {{255,0,0}, {255,30,0}, {255,255,0}, {0,255,0}, {0,0,255}, {98,0,255}};

uint8_t game_bmp[7][7][3] = {{{0, 0, 0}, {0, 0, 0}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 0, 0}}, {{0, 140, 255}, {0, 0, 0}, {0, 0, 0}, {0, 140, 255}, {0, 0, 0}, {0, 0, 0}, {0, 140, 255}}, {{0, 140, 255}, {0, 0, 0}, {99, 58, 148}, {0, 140, 255}, {0, 0, 0}, {99, 58, 148}, {0, 140, 255}}, {{0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}}, {{0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}}, {{0, 140, 255}, {0, 0, 0}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255}, {0, 0, 0}, {0, 140, 255}}};

uint8_t power_bmp[7][7][3] = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}}, {{0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}}};
uint8_t tile_placeholder[7][7][3] = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
//uint8_t game_placeholder[7][7][3] = {{{0, 0, 0}, {0, 0, 0}, {11, 11, 11}, {255, 255, 255}, {11, 11, 11}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {105, 105, 105}, {64, 64, 64}, {104, 104, 104}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {1, 1, 1}, {170, 170, 170}, {0, 0, 0}, {174, 174, 174}, {1, 1, 1}, {0, 0, 0}}, {{0, 0, 0}, {54, 54, 54}, {35, 35, 35}, {0, 0, 0}, {36, 36, 36}, {53, 53, 53}, {0, 0, 0}}, {{0, 0, 0}, {208, 208, 208}, {255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {208, 208, 208}, {0, 0, 0}}, {{22, 22, 22}, {81, 81, 81}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {81, 81, 81}, {21, 21, 21}}, {{142, 142, 142}, {5, 5, 5}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {5, 5, 5}, {141, 141, 141}}};

void screen_anim_task_fn(UArg a0, UArg a1) {
//    static uint8_t blink = 0;

    while (1) {
        Semaphore_pend(anim_sem, BIOS_WAIT_FOREVER);
//
//        if (blink)
//            memset(led_buf, 0x00, sizeof led_buf); // clear out the buffer.
//        else
//            memcpy(led_buf, power_bmp, sizeof power_bmp);
//
//        blink = !blink;

        Clock_setTimeout(screen_anim_clock_h, 50000); // set time for next animation
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

//    memcpy(led_buf, power_bmp, sizeof(power_bmp));
}
