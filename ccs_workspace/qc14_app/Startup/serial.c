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
#include "ui.h"

// in 10s of us:
#define RTS_TIMEOUT (RTS_TIMEOUT_MS*100)
#define PLUG_TIMEOUT (PLUG_TIMEOUT_MS*100)
#define IDLE_BACKOFF (IDLE_BACKOFF_MS*100)

// NB: These should probably be protected by the semaphore:
serial_message_t uart_tx_buf;
serial_message_t uart_rx_buf;

UART_Handle uart_h;
UART_Params uart_p;
Semaphore_Handle uart_mutex;

Task_Struct uart_arm_tasks[4];
char uart_arm_task_stacks[4][512];

uint32_t uart_timeout[4] = {0, 0, 0, 0};
uint8_t uart_rts_old_val[4] = {1, 1, 1, 1};
uint8_t uart_rts_cur_val[4] = {1, 1, 1, 1};
uint8_t uart_proto_state[4] = {0, 0, 0, 0};
uint8_t uart_nts_flag[4] = {0, 0, 0, 0};

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

#define arm_nts uart_nts_flag[uart_id]
#define arm_timeout uart_timeout[uart_id]
#define arm_old_val uart_rts_old_val[uart_id]
#define arm_cur_val uart_rts_cur_val[uart_id]
#define arm_proto_state uart_proto_state[uart_id]
#define arm_gpio_init_table arm_gpio_init_tables[uart_id]
#define arm_gpio_pin_state arm_gpio_pin_states[uart_id]
#define arm_gpio_pin_handle arm_gpio_pin_handles[uart_id]
#define arm_gpio_tx arm_gpio_txs[uart_id]
#define arm_gpio_rx arm_gpio_rxs[uart_id]

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

inline void send_serial_handshake(UArg uart_id) {
    uart_tx_buf.badge_id = my_conf.badge_id;
    uart_tx_buf.msg_type = SERIAL_MSG_TYPE_HANDSHAKE;
    uint8_t* payload = uart_tx_buf.payload;
    serial_handshake_t* handshake_payload = (serial_handshake_t*) payload;
    handshake_payload->current_mode = ui_screen;
    handshake_payload->current_icon_or_tile_id = (ui_screen? my_conf.current_tile : my_conf.current_icon);
    memcpy(handshake_payload->badges_mated, my_conf.badges_mated, 36); // TODO: constant here?
    arm_nts = SERIAL_MSG_TYPE_HANDSHAKE;
}

void rx_done(UArg uart_id) {

}

void disconnected(UArg uart_id) {

}

void arm_color(UArg uart_id, uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i=0; i<6; i++) {
        led_buf[7+uart_id][i][0] = r;
        led_buf[7+uart_id][i][1] = g;
        led_buf[7+uart_id][i][2] = b;
    }
}

void set_state(UArg uart_id, uint8_t dest_state) {
    if (arm_proto_state == PROTO_STATE_CTS_WAIT || arm_proto_state == PROTO_STATE_RTS_OUT) {
        // We're leaving one of the two states in which we hold the
        // UART semaphore. We need to clean up GPIO and UART so we
        // can guarantee they're ready to go for everybody else.

        PIN_close(arm_gpio_pin_handle);
        UART_close(uart_h);
        arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table);
        Semaphore_post(uart_mutex);
    }

    switch(dest_state) {
    case PROTO_STATE_DIS:
        // Return to a known good default disconnected state.
        arm_nts = SERIAL_MSG_TYPE_NOMSG;
        PINCC26XX_setOutputValue(arm_gpio_tx, 0); // Bring output low.
        // TODO: See if we need to adjust the output anywhere else.
        arm_color(uart_id, 0,0,0);
        break;
    case PROTO_STATE_PLUGGING:
        arm_timeout = Clock_getTicks() + PLUG_TIMEOUT;
        arm_color(uart_id, 100,100,0);
        break;
    case PROTO_STATE_IDLE:
        if (arm_proto_state == PROTO_STATE_PLUGGING) {
            // we just made a new connection and need to handshake.
            send_serial_handshake(uart_id);
            arm_timeout = Clock_getTicks() + IDLE_BACKOFF;
        }
        arm_color(uart_id, 255,255,255);
        break;
    case PROTO_STATE_RTS_WAIT:
        arm_timeout = Clock_getTicks() + RTS_TIMEOUT;
        arm_color(uart_id, 0,0,100);
        break;
    case PROTO_STATE_CTS_WAIT:
        arm_color(uart_id, 0,0,255);
        break;
    case PROTO_STATE_RTS_OUT:
        arm_color(uart_id, 0,255,0);
        // We're going to say we're RTS and then wait to hear a CTS back.
        PINCC26XX_setOutputValue(arm_gpio_tx, 1); // Bring output low.
        arm_timeout = Clock_getTicks() + RTS_TIMEOUT;
        break;
    }
    arm_proto_state = dest_state;
}

void serial_handle_state_machine(UArg uart_id) {
    int results_flag;
    switch(arm_proto_state) {
    case PROTO_STATE_DIS:
        // input has pullup, output low.
        // can react to IN going low.
        if (!arm_read_in_debounced(uart_id)) {
            set_state(uart_id, PROTO_STATE_PLUGGING);
        }
        break;
    case PROTO_STATE_PLUGGING:
        if (arm_read_same(uart_id)) {
            // counting toward timeout continues
            if ((int32_t) (arm_timeout - Clock_getTicks()) <= 0) {
                // timed out: we've settled.
                if (arm_cur_val) { // input high, disconnected
                    set_state(uart_id, PROTO_STATE_DIS);
                } else { // input low, fully connected
                    set_state(uart_id, PROTO_STATE_IDLE);
                }
            }
        } else {
            // reset timeout
            arm_timeout = Clock_getTicks() + PLUG_TIMEOUT;
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
            // fall through...

        } else if ((arm_nts == SERIAL_MSG_TYPE_HANDSHAKE && ((int32_t) (arm_timeout - Clock_getTicks()) <= 0) && Semaphore_pend(uart_mutex, BIOS_NO_WAIT))
                   ||  (arm_nts && arm_nts != SERIAL_MSG_TYPE_HANDSHAKE && Semaphore_pend(uart_mutex, BIOS_NO_WAIT)))
        {
            set_state(uart_id, PROTO_STATE_RTS_OUT);
            break;
        }  else {
            // either no need to send or not ready to send what we need.
            // We'll be back soon.
            break;
        }
        // body of first if statement will fall through...
    case PROTO_STATE_RTS_WAIT:
        // We have an incoming "ready to send" signal. We need to
        //  get ready to receive it. Once we have the UART semaphore,
        //  we can signal clear to send.
        if (Semaphore_pend(uart_mutex, RTS_TIMEOUT)) {
            // we have the UART.
            // we respond to an RTS signal by taking our output high, too.
            // We are clear to send. Signal CTS.
            PINCC26XX_setOutputValue(arm_gpio_tx, 1);
            set_state(uart_id, PROTO_STATE_CTS_WAIT);
        } else {
            // We didn't get the UART in time.
            // arm_proto_state = PROTO_STATE_DIS;?????
        }
        break;
    case PROTO_STATE_CTS_WAIT:
        // We've had an incoming RTS, and signaled CTS.
        //  The correct response should be for RTS to be deasserted
        //  (go low). If that doesn't happen, we know we were
        //  unplugged instead of receiving a RTS. We respond to the
        //  deasserted RTS by deasserting CTS and activating our UART
        //  in receive mode.

        // Can react to input going LOW (again, CTS ack):
        if (!arm_read_in_debounced(uart_id)) {
            // Our CTS has been acknowledged.
            // Now we're just going to block and listen.

            // Tear down my GPIO and prepare to switch to the peripheral:
            PIN_close(arm_gpio_pin_handle);
            uart_h = UART_open(uart_id, &uart_p);
            results_flag = UART_read(uart_h, &uart_rx_buf, sizeof(serial_message_t));

            if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
                // borken:
                set_state(uart_id, PROTO_STATE_DIS);
            } else {
                // parse the message here...
                set_state(uart_id, PROTO_STATE_IDLE);
            }
        } // or can react to TIMEOUT:
        else if ((int32_t) (arm_timeout - Clock_getTicks()) <= 0) {
            // TODO: get our GPIO and UART in known-good states.
            // DISCONNECTED!!!
            set_state(uart_id, PROTO_STATE_DIS);
        }
        break;
    case PROTO_STATE_RTS_OUT:
        // We've asserted our RTS. The expected response is to see
        //  an asserted CTS within the timeout timeframe.

        if (arm_read_in_debounced(uart_id)) {
            // We are clear to send. Acknowledge by deasserting RTS.
            PINCC26XX_setOutputValue(arm_gpio_tx, 0);
            //  Then we'll wait for a moment to send our data.
            //  This is to be totally sure the other badge has time
            //  to tear down its GPIO and stand up its UART.

            Task_sleep(100); // 1000 us = 1 ms

            // We've already received a CTS and acknowledged it.
            // The other side should be blocking and listening by now.
            // Time to get set up and send.

            // Tear down my GPIO and prepare to switch to the peripheral:
            PIN_close(arm_gpio_pin_handle);
            uart_h = UART_open(uart_id, &uart_p);
            results_flag = UART_write(uart_h, &uart_tx_buf, sizeof(serial_message_t));
            arm_nts = SERIAL_MSG_TYPE_NOMSG;

            if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
                // something broke:
                set_state(uart_id, PROTO_STATE_DIS);
            } else {
                // sent successfully.
                set_state(uart_id, PROTO_STATE_IDLE);
            }
        } // or can react to TIMEOUT:
        else if ((int32_t) (arm_timeout - Clock_getTicks()) <= 0) {
            // The other side couldn't get its UART in time.
            // Or else it had some other kind of error. Fail to
            // disconnected.
            set_state(uart_id, PROTO_STATE_DIS); // TODO: Should this be idle?
        }
        break;
    }
}

// All four arms share the same function, even though they have separate tasks.
void serial_arm_task(UArg uart_id, UArg arg1) {
    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
    do {
        // We only care about mating if we're in a selected mode,
        // not if we're sleeping or selecting.
        if (ui_screen == UI_SCREEN_GAME || ui_screen == UI_SCREEN_TILE)
            serial_handle_state_machine(uart_id);

        Task_sleep(100); // 1000 us
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
    uart_p.baudRate = 9600;
    uart_p.readTimeout = RTS_TIMEOUT;
    uart_p.writeTimeout = RTS_TIMEOUT;
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