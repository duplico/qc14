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
#include <ti/drivers/uart/UARTCC26XX.h>

// Application includes
#include "simple_broadcaster.h"

// QC14 Drivers
#include "serial.h"
#include "qc14.h"
#include "tlc_driver.h"
#include "ui.h"

// in 10s of us:
#define RTS_TIMEOUT (RTS_TIMEOUT_MS*100)
#define PLUG_TIMEOUT (PLUG_TIMEOUT_MS*100)
#define IDLE_BACKOFF (IDLE_BACKOFF_MS*100)

serial_message_t uart_tx_buf[4] = {0};
// NB: This must be protected by the semaphore:
serial_message_t arm_rx_buf;

UART_Handle uart_h;
UART_Params uart_p;
Semaphore_Handle uart_mutex;

Task_Struct uart_arm_tasks[4];
char uart_arm_task_stacks[4][320];

uint32_t uart_timeout[4] = {0, 0, 0, 0};
uint8_t uart_proto_state[4] = {0, 0, 0, 0};
uint8_t uart_nts_flag[4] = {0, 0, 0, 0};

PIN_Config arm_gpio_init_tables[4][3] = {
    {
         P1_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
         P1_RX           | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS | PIN_IRQ_DIS,
         PIN_TERMINATE
    }, {
        P2_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P2_RX           | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }, {
        P3_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P3_RX           | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }, {
        P4_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P4_RX           | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }
};
PIN_State arm_gpio_pin_states[4];
PIN_Handle arm_gpio_pin_handles[4];

uint64_t arm_gpio_txs[4] = {P1_TX, P2_TX, P3_TX, P4_TX};
uint64_t arm_gpio_rxs[4] = {P1_RX, P2_RX, P3_RX, P4_RX};

#define arm_tx_buf uart_tx_buf[uart_id]
#define arm_nts uart_nts_flag[uart_id]
#define arm_timeout uart_timeout[uart_id]
#define arm_proto_state uart_proto_state[uart_id]
#define arm_gpio_init_table arm_gpio_init_tables[uart_id]
#define arm_gpio_pin_state arm_gpio_pin_states[uart_id]
#define arm_gpio_pin_handle arm_gpio_pin_handles[uart_id]
#define arm_gpio_tx arm_gpio_txs[uart_id]
#define arm_gpio_rx arm_gpio_rxs[uart_id]

void setup_tx_buf_no_payload(UArg uart_id) {
    arm_tx_buf.badge_id = my_conf.badge_id;
    arm_tx_buf.current_time = my_conf.csecs_of_queercon;
    arm_tx_buf.current_time_authority = my_conf.time_is_set;
    arm_tx_buf.arm_id = uart_id;
}

inline void send_serial_handshake(UArg uart_id) {
    arm_tx_buf.msg_type = SERIAL_MSG_TYPE_HANDSHAKE;
    uint8_t* payload = arm_tx_buf.payload;
    serial_handshake_t* handshake_payload = (serial_handshake_t*) payload;
    handshake_payload->current_mode = ui_screen;
    handshake_payload->current_icon_or_tile_id = (ui_screen? my_conf.current_tile : my_conf.current_icon);
    memcpy(handshake_payload->badges_mated, my_conf.badges_mated, BADGES_MATED_BYTES);
    arm_nts = SERIAL_MSG_TYPE_HANDSHAKE;
}

uint8_t rx_valid(UArg uart_id) {
    if (arm_rx_buf.badge_id > BADGES_IN_SYSTEM)
        return 0;
    if (arm_rx_buf.msg_type > SERIAL_MSG_TYPE_MAX)
        return 0;
    if (arm_rx_buf.badge_id == my_conf.badge_id)
        return 0;
    if (arm_rx_buf.crc != crc16((uint8_t *) &arm_rx_buf,
                                sizeof(serial_message_t) - 2))
        return 0;
    return 1;
}

void rx_done(UArg uart_id) {
    set_badge_mated(arm_rx_buf.badge_id);
    // We take our clock setting from this person if:
    //  1. They are authoritative and we are not, OR
    //  2. Neither of us are authoritative and they are more than 10 csecs
    //     (500 ms) ahead of us.
    //     This is to avoid any accidental positive feedback loops on time.

    if (!my_conf.time_is_set &&
            (arm_rx_buf.current_time_authority ||
             arm_rx_buf.current_time > my_conf.csecs_of_queercon+50)) {
        set_clock(arm_rx_buf.current_time);
        my_conf.time_is_set = arm_rx_buf.current_time_authority;
    }
}

uint8_t serial_in_progress() {
    return uart_proto_state[0] || uart_proto_state[1] || uart_proto_state[2] || uart_proto_state[3];
}

void disconnected(UArg uart_id) {
    arm_color(uart_id, 0,0,0);
    arm_proto_state=SERIAL_PHY_STATE_DIS;
    PINCC26XX_setOutputValue(arm_gpio_tx, 0);
    Task_sleep(RTS_TIMEOUT*1.2);
}

uint8_t wait_with_timeout(UArg uart_id, uint8_t match_val, uint32_t timeout_ms, uint32_t settle_reads) {
    uint32_t read_in_ms = 0;

    while (timeout_ms) {
        if (PINCC26XX_getInputValue(arm_gpio_rx) == match_val) {
            if (++read_in_ms == settle_reads)
                return 1;
        }
        else {
            if (timeout_ms != UINT32_MAX) // If we've passed in UINT32_MAX, don't time out ever.
                timeout_ms--;
            read_in_ms = 0;
        }
        Task_sleep(100); // 1 ms.
    }
    return 0;
}

void serial_arm_task(UArg uart_id, UArg arg1) {
    arm_proto_state=SERIAL_PHY_STATE_DIS;
    uint32_t timeout_ms = PLUG_TIMEOUT_MS;
    int results_flag = 0;

    // This table holds the correct disconnected values:
    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table);

    do {
        // A "continue" takes us here:
        Task_yield();

        if (arm_proto_state == SERIAL_PHY_STATE_DIS) { // We are disconnected.
           /*
            * While disconnected:
            *  Listen for input HIGH.
            *  Wait for it to stay HIGH for PLUG_TIMEOUT.
            *  We are now connected and idle.
            */
            PINCC26XX_setOutputValue(arm_gpio_tx, 1);

            timeout_ms = PLUG_TIMEOUT_MS;

            while (timeout_ms) {
                // This is the only place we can get connected,
                // so there's no need to leave this loop. We do,
                // however, need to yield so that other threads
                // (including other arms) can actually function.

                if (PINCC26XX_getInputValue(arm_gpio_rx)) {
                    timeout_ms--;
                } else {
                    timeout_ms = PLUG_TIMEOUT_MS;
                }
                // We're only allowed to get connected if we're in the tile or
                //  game mode.
                // So if we're in those modes we'll go ahead and do the normal
                //  sleeping thing. Otherwise start the whole business over.

                Task_sleep(100); // 1 ms.

                if (ui_screen == UI_SCREEN_GAME ||
                        ui_screen == UI_SCREEN_TILE) {
                    // we're OK, go back up to the while.
                } else {
                    // not OK to continue, restart the timeout.
                    timeout_ms = PLUG_TIMEOUT_MS;
                }
            }

            // we are now connected, can fall through:
            arm_proto_state = SERIAL_PHY_STATE_CON;
            arm_color(uart_id, 255,255,255);

            // Prep a handshake:
            send_serial_handshake(uart_id);
        }

        if (arm_proto_state==SERIAL_PHY_STATE_CON) { // We are connected.
            // Regardless of whether we need to send, we need to check whether
            //  the other badge has already decided _it_ needs to send:

            // So let's listen for a LOW: (RTS assert | disconnect)
            if (wait_with_timeout(uart_id, 0, 500, 1)) { // this blocks for 500 ms
                // We got a LOW.
                // We got a LOW input: either RTS assert or disconnect.
                /*
                 *   Get the semaphore, and bring our input LOW. (dim blue)
                 *   Wait with timeout for input HIGH:
                 *      If we time out, we are DISCONNECTED. (off)
                 *      If we receive input HIGH then we need to receive. (bright blue)
                 *      Switch to UART mode (idle has output high).
                 *      Receive. (green or red)
                 *      Process message.
                 *      We are now connected and idle. On error, DISCONNECTED.
                 */

                // Get the semaphore.
                arm_color(uart_id, 255,255,0);
                Semaphore_pend(uart_mutex, BIOS_WAIT_FOREVER);
                // Set our own low:
                PINCC26XX_setOutputValue(arm_gpio_tx, 0);
                arm_color(uart_id, 0,0,255);
                // Wait with a timeout for a HIGH input:
                if (!wait_with_timeout(uart_id, 1, RTS_TIMEOUT_MS, SERIAL_SETTLE_TIME_MS)) {
                    // timed out. We are DISCONNECTED.
                    Semaphore_post(uart_mutex);
                    disconnected(uart_id);
                    continue; // back to the beginning.
                }
                // We got the HIGH we needed.
                // Time to set up to receive.
                //  A byproduct of this is sending a HIGH.
                PIN_close(arm_gpio_pin_handle);
                uart_h = UART_open(uart_id, &uart_p);
                results_flag = UART_read(uart_h, &arm_rx_buf, sizeof(serial_message_t));

                // Free up resources.
                // This must be up here because disconnected() needs gpio access:
                UART_close(uart_h);
                arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state,
                                               arm_gpio_init_table);

                if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
                    // Error. We are now disconnected.
                    disconnected(uart_id);

                    // The debugger can look at these to figure out what happened:
                    // volatile UARTCC26XX_Object *o = (UARTCC26XX_Object *) uart_h->object;
                    // o->status;

                } else { // success:
                    // Process message.
                    rx_done(uart_id);

                    // Good, back to connected.
                    arm_color(uart_id, 255,255,255);
                    // Give the other side a bit to stabilize:
                    Task_sleep(5000); // 50000 us = 50 ms
                }
                // This must be here to protect the buffer in rx_done:
                Semaphore_post(uart_mutex);
                // Loop again.
                continue;
            } else {
                // We're just connected as normal, no signal coming in.
                //  Determine whether we need to send, ourselves.
                if (arm_nts) { // need to send
                    /*
                     *  If we DO need to send: (dim blue arm)
                     *   Get the semaphore.
                     *   Set our output LOW. (yellow arm)
                     *   Wait for input LOW. (bright blue)
                     *   They have accepted, or have disconnected. Set output HIGH.
                     *   Wait with timeout for input HIGH:
                     *      If we time out, we are DISCONNECTED. (arm off)
                     *      If we receive input HIGH then we need to send a message.
                     *       (the HIGH came from other side's UART)
                     *      Switch to UART mode
                     *      Send. (green or red)
                     *      We are now IDLE. On error, we are DISCONNECTED.
                     */
                    arm_color(uart_id, 0,0,50);
                    if (!Semaphore_pend(uart_mutex, BIOS_NO_WAIT))
                        continue; // didn't get the semaphore, yield.

                    // Output low to signal RTS.
                    PINCC26XX_setOutputValue(arm_gpio_tx, 0);
                    // Wait to get a low input (means CTS):
                    arm_color(uart_id, 0,0,50);
                    while (PINCC26XX_getInputValue(arm_gpio_rx)) {
                        Task_yield(); // it will always eventually go low
                    }
                    arm_color(uart_id, 255,255,0);

                    // Got low input. They've either accepted, or disconnected.
                    // Set output high and wait with timeout for high input.
                    PINCC26XX_setOutputValue(arm_gpio_tx, 1);
                    if (wait_with_timeout(uart_id, 1, RTS_TIMEOUT_MS, SERIAL_SETTLE_TIME_MS)) {
                        arm_color(uart_id, 0,0,255);
                        // They went high. Means they've accepted.
                        // The HIGH signal came from the other side's UART,
                        //  so it should be all built and such.
                        // Switch to UART mode and send.
                        PIN_close(arm_gpio_pin_handle);
                        uart_h = UART_open(uart_id, &uart_p);

                        setup_tx_buf_no_payload(uart_id);

                        results_flag = UART_write(uart_h, &arm_tx_buf,
                                                  sizeof(serial_message_t));

                        // Done sending, so we can cancel this flag:
                        arm_nts = SERIAL_MSG_TYPE_NOMSG;

                        // Yield ALL THE THINGS
                        UART_close(uart_h);
                        arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state,
                                                       arm_gpio_init_table);
                        Semaphore_post(uart_mutex);

                        // We are now IDLE. On error, we are DISCONNECTED:
                        if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
                            // something broke.
                            // The debugger can look at these lines to diagnose:
                            // volatile UARTCC26XX_Object *o = (UARTCC26XX_Object *) uart_h->object;
                            // o->status;
                            disconnected(uart_id);
                        } else { // success:
                            // Good, back to connected.
                            arm_color(uart_id, 255,255,255);
                            // Give the other side a bit to stabilize:
                            Task_sleep(5000); // 50000 us = 50 ms
                        }
                    } else {
                        // They never went high. Means they've disconnected.
                        disconnected(uart_id);
                        Semaphore_post(uart_mutex);
                    }
                } // end of if (arm_nts)
                continue; // No signal coming in, and regardless of whether we
                // just sent something, we need to start over.
            }
        }
    } while (1);
}

void serial_init() {
    Semaphore_Params uart_mutex_params;
    Semaphore_Params_init(&uart_mutex_params);

    uart_mutex_params.mode = Semaphore_Mode_BINARY;
    uart_mutex = Semaphore_create(1, &uart_mutex_params, NULL);

    UART_Params_init(&uart_p);
    // Defaults used:
    // blocking reads and writes, no write timeout, 8N1.
    uart_p.baudRate = 115200;
    uart_p.readTimeout = RTS_TIMEOUT;
    uart_p.writeTimeout = RTS_TIMEOUT;
    uart_p.readMode = UART_MODE_BLOCKING;
    uart_p.writeMode = UART_MODE_BLOCKING;
    uart_p.readEcho = UART_ECHO_OFF;
    uart_p.readDataMode = UART_DATA_BINARY;
    uart_p.writeDataMode = UART_DATA_BINARY;

    // OK, so. Here's the deal. We only have one UART peripheral for some
    //  reason (Thanks TI!). But we've got four arms that need to have serial
    //  connections. Happily, we can remap the GPIO pins' assignments to
    //  peripherals. So as far as the RTOS is concerned, we've got four
    //  UART configs, each of which map to the same peripheral but different
    //  pins.
    // So, to manage each of those connections we're going to have separate
    //  thread/tasks. They all have the same function associated with them,
    //  because their behavior is identical. We pass the function an index,
    //  as the Task API lets us send a task-specific pair of parameters to
    //  their functions. Then inside the task function, we use a bunch of
    //  preprocessor macros to translate that parameter into the right set
    //  of identifiers for the appropriate port on the cube.
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
