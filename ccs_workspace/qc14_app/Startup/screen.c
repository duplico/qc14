/*
 * screen.c
 *
 *  Created on: Jun 12, 2017
 *      Author: George
 */

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

extern uint8_t led_buf[11][7][3];

Task_Struct screen_anim_task;
char screen_anim_task_stack[200];

Clock_Handle screen_anim_clock_h;

Semaphore_Handle anim_sem;

void screen_anim_tick(UArg a0) {
    Semaphore_post(anim_sem);
}

void screen_anim_task_fn(UArg a0, UArg a1) {
    static uint8_t all_led_color = 0;
    while (1) {
        Semaphore_pend(anim_sem, BIOS_WAIT_FOREVER);
        for (uint8_t r=0; r<11; r++)
            for (uint8_t c=0; c<7; c++)
                for (uint8_t q=0; q<3; q++)
                    led_buf[r][c][q] = all_led_color;
        all_led_color++;
        Task_sleep(10);
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
    clockParams.period = 3000; // times 10 us
    clockParams.startFlag = TRUE;
    screen_anim_clock_h = Clock_create(screen_anim_tick, 2, &clockParams, NULL);
}
