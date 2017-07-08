/*
 * serial.c
 *
 *  Created on: Jul 8, 2017
 *      Author: George
 */
// Standard library includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// TI runtime
#include <xdc/runtime/Error.h>

// SYS/BIOS primitives
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

// Instruction fetch cache dependencies:
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

// TI-RTOS drivers
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>

// Application includes
#include "simple_broadcaster.h"

// QC14 Drivers
#include "serial.h"
#include "qc14.h"
#include "tlc_driver.h"

#define PROTO_STATE_DIS 0
#define PROTO_STATE_PLUGGING 5
#define PROTO_STATE_IDLE 1
#define PROTO_STATE_RTS_WAIT 2
#define PROTO_STATE_CTS_WAIT 3
#define PROTO_STATE_SERIAL_PLACEHOLDER 4

// in 10s of us:
#define RTS_TIMEOUT (RTS_TIMEOUT_MS*100)
#define PLUG_TIMEOUT (PLUG_TIMEOUT_MS*100)

UART_Handle uart_h;
UART_Params uart_p;
Semaphore_Handle uart_mutex;

Task_Struct uart_arm_tasks[4];
char uart_arm_task_stacks[4][512];

uint32_t uart_rts_timeout[4] = {0, 0, 0, 0};
uint32_t uart_plug_timeout[4] = {0, 0, 0, 0};
uint8_t uart_rts_old_val[4] = {1, 1, 1, 1};
uint8_t uart_rts_cur_val[4] = {1, 1, 1, 1};
uint8_t uart_proto_state[4] = {0, 0, 0, 0};
uint8_t uart_hold_sem[4] = {0, 0, 0, 0};

PIN_Config arm_gpio_init_tables[4][3] = {
    {
         P1_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
         P1_RX           | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_DIS,
         PIN_TERMINATE
    }, {
        P2_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P2_RX           | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }, {
        P3_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P3_RX           | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }, {
        P4_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P4_RX           | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }
};
PIN_State arm_gpio_pin_states[4];
PIN_Handle arm_gpio_pin_handles[4];

uint64_t arm_gpio_txs[4] = {P1_TX, P2_TX, P3_TX, P4_TX};
uint64_t arm_gpio_rxs[4] = {P1_RX, P2_RX, P3_RX, P4_RX};

#define arm_rts_timeout uart_rts_timeout[uart_id]
#define arm_plug_timeout uart_plug_timeout[uart_id]
#define arm_old_val uart_rts_old_val[uart_id]
#define arm_cur_val uart_rts_cur_val[uart_id]
#define arm_proto_state uart_proto_state[uart_id]
#define arm_gpio_init_table arm_gpio_init_tables[uart_id]
#define arm_gpio_pin_state arm_gpio_pin_states[uart_id]
#define arm_gpio_pin_handle arm_gpio_pin_handles[uart_id]
#define arm_gpio_tx arm_gpio_txs[uart_id]
#define arm_gpio_rx arm_gpio_rxs[uart_id]
#define arm_i_have_uart uart_hold_sem[uart_id]

uint8_t arm_read_in_debounced(uint8_t uart_id) {
    uint8_t read_val = PINCC26XX_getInputValue(arm_gpio_rx);

    if (arm_old_val == read_val)
        arm_cur_val = read_val;
    arm_old_val = read_val;

    return arm_cur_val;
}

uint8_t arm_read_same(uint8_t uart_id) {
    uint8_t read_val = PINCC26XX_getInputValue(arm_gpio_rx);

    if (arm_old_val == read_val) {
        arm_cur_val = read_val;
        return 1;
    }
    arm_old_val = read_val;

    return 0;
}

//void serial_do_stuff(uint8_t uart_id) {
//    char buf[12] = "test";
//
//    // Tear down my GPIO and prepare to switch to the peripheral:
//    PIN_close(arm_gpio_pin_handle);
//
//    // Secure control of the global UART:
//    Semaphore_pend(uart_mutex, BIOS_WAIT_FOREVER);
//    uart_h = UART_open(uart_id, &uart_p);
//    UART_write(uart_h, buf, strlen(buf));
//    // Release the global UART:
//    UART_close(uart_h);
//    Semaphore_post(uart_mutex);
//    // Reallocate our pins.
//    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct *disconnected* values.
//}

void set_state(UArg uart_id, uint8_t dest_state) {
    arm_proto_state = dest_state;

    switch(dest_state) {
    case PROTO_STATE_DIS:
        memset(led_buf[7+uart_id], 0x00, 6*3);
        break;
    case PROTO_STATE_PLUGGING:
        memset(led_buf[7+uart_id], 0x20, 6*3);
        break;
    case PROTO_STATE_IDLE:
        memset(led_buf[7+uart_id], 0xff, 6*3);
        break;
    case PROTO_STATE_RTS_WAIT:
        break;
    case PROTO_STATE_CTS_WAIT:
        break;
    }
}

// All four arms share the same function, even though they have separate tasks.
void serial_arm_task(UArg uart_id, UArg arg1) {
    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
    do {
        // Possible "events":
        //  IN Low (debounced)
        //  IN High (debounced)

        switch(arm_proto_state) {
        case PROTO_STATE_DIS:
            // input has pullup, output low.
            // can react to IN going low.
            if (!arm_read_in_debounced(uart_id)) {
                set_state(uart_id, PROTO_STATE_PLUGGING);
                arm_plug_timeout = Clock_getTicks() + PLUG_TIMEOUT;
            }
            break;
        case PROTO_STATE_PLUGGING:
            if (arm_read_same(uart_id)) {
                // counting toward timeout continues
                if ((int32_t) (arm_plug_timeout - Clock_getTicks()) <= 0) {
                    // timed out: we've settled.
                    if (arm_cur_val) { // input high, disconnected
                        set_state(uart_id, PROTO_STATE_DIS);
                    } else { // input low, fully connected
                        set_state(uart_id, PROTO_STATE_IDLE);
                    }
                }
            } else {
                // reset timeout
                arm_plug_timeout = Clock_getTicks() + PLUG_TIMEOUT;
            }
            break;
        case PROTO_STATE_IDLE:
            // input has pullup, output low.
            // can react to IN going high.
            if (arm_read_in_debounced(uart_id)) {
                // debounced input has gone high.
                // this is either a disconnect or a RTS signal.
                // Either way, we wait until we have control of our UART, and
                // attempt to respond.
                set_state(uart_id, PROTO_STATE_RTS_WAIT);
                arm_rts_timeout = Clock_getTicks() + RTS_TIMEOUT;
                // fall through...
            } else {
                break;
            }
        case PROTO_STATE_RTS_WAIT:
            if (Semaphore_pend(uart_mutex, RTS_TIMEOUT)) {
                // we have the UART.
                // we respond to an RTS signal by taking our output high, too.
                arm_i_have_uart = 1;
                // We are clear to send. Signal CTS.
                PINCC26XX_setOutputValue(arm_gpio_tx, 1);
                set_state(uart_id, PROTO_STATE_CTS_WAIT);
                // TODO: Release it at the real time.
                Semaphore_post(uart_mutex);
            } else {
                // We didn't get the UART in time.
                // arm_proto_state = PROTO_STATE_DIS;
            }
            break;
        case PROTO_STATE_CTS_WAIT:
            // in is high, out set high.
            // can react to IN going low (do serial stuff)

            if (!arm_read_in_debounced(uart_id)) {
                // Our CTS has been acknowledged.
            }
            // can react to TIMEOUT.
            if ((int32_t) (arm_rts_timeout - Clock_getTicks()) <= 0) {
                // DISCONNECTED!!!
                set_state(uart_id, PROTO_STATE_DIS);
                PINCC26XX_setOutputValue(arm_gpio_tx, 0); // Bring output low.
            }
        }

        Task_sleep(100); // 1000 us
    } while (1);
}

void serial_init() {
    Semaphore_Params uart_mutex_params;
    Semaphore_Params_init(&uart_mutex_params);

    uart_mutex_params.mode = Semaphore_Mode_BINARY;
    uart_mutex = Semaphore_create(1, &uart_mutex_params, NULL);

    UART_Params_init(&uart_p);
    uart_p.baudRate = 9600;

    // OK, so. Here's the deal. We only have one UART peripheral for some
    //  reason (Thanks TI!). But we've got four arms that need to have serial
    //  connections. Happily, we can remap the GPIO pins' assignments to
    //  peripherals. So as far as the RTOS is concerned, we've got four
    //  UART configs, each of which map to the same peripheral but different
    //  pins.
    // So, to manage each of those connections we're going to have a separate
    //  thread/task. They all have the same function associated with them,
    //  because their behavior is identical. We pass the function an index,
    //  as the Task API lets us send a task-specific pair of parameters to
    //  their functions. Then inside the task function, we use a bunch of
    //  preprocessor macros to translate that parameter into the right set
    //  of identifiers for the appropriate side of the cube.
    for (uint8_t task_arm_id=0; task_arm_id<4; task_arm_id++) {
        Task_Params taskParams;
        Task_Params_init(&taskParams);
        taskParams.stack = uart_arm_task_stacks[task_arm_id];
        taskParams.stackSize = sizeof(uart_arm_task_stacks[0]);
        taskParams.priority = 1;
        taskParams.arg0 = task_arm_id;
        Task_construct(&uart_arm_tasks[task_arm_id], serial_arm_task, &taskParams, NULL);
    }
}
