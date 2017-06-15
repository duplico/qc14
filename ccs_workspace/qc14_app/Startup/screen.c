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

extern uint8_t led_buf[11][7][3];

Task_Struct screen_anim_task;
char screen_anim_task_stack[200];

Clock_Handle screen_anim_clock_h;

Semaphore_Handle anim_sem;

void screen_anim_tick(UArg a0) {
    Semaphore_post(anim_sem);
}

uint8_t rainbow_colors[6][3] = {{255,0,0}, {255,30,0}, {255,255,0}, {0,255,0}, {0,0,255}, {98,0,255}};

void screen_anim_task_fn(UArg a0, UArg a1) {
    static uint8_t all_led_color = 0;
    static uint8_t rainbow_index = 0;

    while (1) {
        Semaphore_pend(anim_sem, BIOS_WAIT_FOREVER);
        memset(led_buf, 0xff, sizeof led_buf);
        Clock_setPeriod(screen_anim_clock_h, 5000); // set time for next animation
        continue;

        memset(led_buf, 0, 147);
        for (int i=0; i<6; i++) {
            int r;
            r = (rainbow_index + i) % 7;
            for (int c=0; c<7; c++) {
                led_buf[r][c][0] = rainbow_colors[i][0];
                led_buf[r][c][1] = rainbow_colors[i][1];
                led_buf[r][c][2] = rainbow_colors[i][2];
            }
        }
        rainbow_index = (rainbow_index+1) % 7;
        Clock_setPeriod(screen_anim_clock_h, 5000); // set time for next animation

//        for (uint8_t r=0; r<11; r++)
//            for (uint8_t c=0; c<7; c++)
//                for (uint8_t q=0; q<3; q++)
//                    led_buf[r][c][q] = all_led_color;
//        all_led_color++;
//        Clock_setPeriod(screen_anim_clock_h, 3000); // set time for next animation
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
