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

// NB: These should probably be protected by the semaphore:
serial_message_t uart_tx_buf;
serial_message_t uart_rx_buf;

UART_Handle uart_h;
UART_Params uart_p;
Semaphore_Handle uart_mutex;

Task_Struct uart_arm_tasks[4];
char uart_arm_task_stacks[4][512];

uint32_t uart_timeout[4] = {0, 0, 0, 0};
uint8_t uart_rts_old_val[4] = {0, 0, 0, 0};
uint8_t uart_rts_cur_val[4] = {0, 0, 0, 0};
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
    }

    switch(dest_state) {
    case PROTO_STATE_DIS:
        // Return to a known good default disconnected state.
        arm_nts = SERIAL_MSG_TYPE_NOMSG;
        PINCC26XX_setOutputValue(arm_gpio_tx, 1); // Bring output high (normal)
        // TODO: See if we need to adjust the output anywhere else.
        arm_color(uart_id, 0,0,0);
        Task_sleep(10000); // 100000 us
        break;
    case PROTO_STATE_PLUGGING:
        arm_color(uart_id, 100,100,0);
        break;
    case PROTO_STATE_IDLE:
        PINCC26XX_setOutputValue(arm_gpio_tx, 1); // Bring output high (normal)
        if (arm_proto_state == PROTO_STATE_PLUGGING) {
            // we just made a new connection and need to handshake.
//            send_serial_handshake(uart_id); // TODO
        }
//        if (arm_nts == SERIAL_MSG_TYPE_HANDSHAKE)
//            arm_timeout = Clock_getTicks() + IDLE_BACKOFF;
        arm_color(uart_id, 255,255,255);
        break;
    case PROTO_STATE_RTS_WAIT:
//        arm_timeout = Clock_getTicks() + RTS_TIMEOUT; // TODO:
//        arm_color(uart_id, 0,0,100);
        break;
    case PROTO_STATE_CTS_WAIT:
//        arm_color(uart_id, 0,0,255);
        break;
    case PROTO_STATE_RTS_OUT:
        arm_color(uart_id, 0,255,0);
        // We're going to say we're RTS and then wait to hear a CTS back.
        PINCC26XX_setOutputValue(arm_gpio_tx, 0); // Bring output low to signal
        arm_timeout = Clock_getTicks() + RTS_TIMEOUT;
        break;
    }
    arm_proto_state = dest_state;
}

void serial_handle_state_machine(UArg uart_id) {
    int results_flag;
    uint32_t wait_ms;
    switch(arm_proto_state) {
    case PROTO_STATE_DIS:
        // input has pulldown, output high.
        // can react to IN going high.
        if (arm_read_in_debounced(uart_id)) {
            set_state(uart_id, PROTO_STATE_PLUGGING);
        }
        break;
    case PROTO_STATE_PLUGGING:
        wait_ms = 2000;
        while (wait_ms) {
            Task_sleep(100); // 1 ms
            if (!arm_read_same(uart_id))
                wait_ms = 2000; // change, reset timeout
            else
                wait_ms--; // no change, keep ticking
        }
        // timeout over.
        if (arm_cur_val) { // input high, connected.
            set_state(uart_id, PROTO_STATE_IDLE);
        } else { // low, disconnected.
            set_state(uart_id, PROTO_STATE_DIS);
        }
        break;
    case PROTO_STATE_IDLE:
        // input has pulldown, output high.
        // can react to IN going low.
        if (!arm_read_in_debounced(uart_id)) {
            // debounced input has gone low.
            // this is either a disconnect or a RTS signal.
            // Either way, we wait until we have control of our UART, and
            // attempt to respond.
            set_state(uart_id, PROTO_STATE_RTS_WAIT);
            // fall through...
//        } else if ((arm_nts == SERIAL_MSG_TYPE_HANDSHAKE && ((int32_t) (arm_timeout - Clock_getTicks()) <= 0) && Semaphore_pend(uart_mutex, BIOS_NO_WAIT))
//                   ||  (arm_nts && arm_nts != SERIAL_MSG_TYPE_HANDSHAKE && Semaphore_pend(uart_mutex, BIOS_NO_WAIT)))
//        {
//            set_state(uart_id, PROTO_STATE_RTS_OUT);
//            break;
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

        // yellow = maybe disconnected
        arm_color(uart_id, 100,100,0);

        // now our input is low. We're going to respond by going low as well,
        // once we have access to the UART.
        Semaphore_pend(uart_mutex, BIOS_WAIT_FOREVER);
        PINCC26XX_setOutputValue(arm_gpio_tx, 0);
        set_state(uart_id, PROTO_STATE_CTS_WAIT);
        break;
    case PROTO_STATE_CTS_WAIT:
        // We've had an incoming RTS, and signaled CTS.
        //  The correct response should be for RTS to be deasserted
        //  (go high). If that doesn't happen, we know we were
        //  unplugged instead of receiving a RTS.
        // If that does happen, we're in the protocol and respond to the
        //  deasserted RTS by deasserting CTS and activating our UART
        //  in receive mode.

        // This low output should be acknowledged by a high. wait a moment to make sure
        //  it actually happens. If it doesn't, we're disconnected and should restart.

        Task_sleep(100000); // 1000000 us = 1000 ms

        if (!PINCC26XX_getInputValue(arm_gpio_rx)) {
            // still low, probably disconnected.
            PINCC26XX_setOutputValue(arm_gpio_tx, 1);
            arm_color(uart_id, 0,0,0);
            arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
            Semaphore_post(uart_mutex);
            set_state(uart_id, PROTO_STATE_DIS);
            break;
        }
        // cyan = still connected, rts+cts:
        arm_color(uart_id, 0,100,100);

        // it went high. yay! That means they're still connected. They've gone high to set up
        //  the ground state for the UART communications. So we need to do the same.
        //  The other side will wait a moment before sending, but we don't need to do that.
        //  It's high, which is where it's supposed to be so we can just _go_!
        PINCC26XX_setOutputValue(arm_gpio_tx, 1);

        PIN_close(arm_gpio_pin_handle);
        uart_h = UART_open(uart_id, &uart_p);
        arm_color(uart_id, 0,100,100);
        results_flag = UART_read(uart_h, &uart_rx_buf, sizeof(serial_message_t));

        if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
            // something broke:
            volatile UARTCC26XX_Object *o = (UARTCC26XX_Object *) uart_h->object;
            o->status;
            set_state(uart_id, PROTO_STATE_DIS);
            // red = failed read:
            arm_color(uart_id, 100,0,0);
            UART_close(uart_h);
            arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
            Semaphore_post(uart_mutex);
            Task_sleep(40000); // 400000 us = 400 ms
            set_state(uart_id, PROTO_STATE_DIS);
        } else { // success:
            // green = successful read:
            arm_color(uart_id, 0,100,0);
            UART_close(uart_h);
            arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
            Semaphore_post(uart_mutex);
            Task_sleep(40000); // 400000 us = 400 ms
            set_state(uart_id, PROTO_STATE_IDLE);
        }
        break;
    case PROTO_STATE_RTS_OUT:
        // We've asserted our RTS. The expected response is to see
        //  an asserted CTS within the timeout timeframe.

        if (!arm_read_in_debounced(uart_id)) {
            // We are clear to send. Acknowledge by deasserting RTS.
            PINCC26XX_setOutputValue(arm_gpio_tx, 1);
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

            UART_close(uart_h);
            arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table);
            Semaphore_post(uart_mutex);

            if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
                // something broke:
                set_state(uart_id, PROTO_STATE_DIS);
            } else {
                // sent successfully.
                set_state(uart_id, PROTO_STATE_IDLE);
                Task_sleep(100); // 1000 us = 1 ms
            }
        } // or can react to TIMEOUT:
        else if ((int32_t) (arm_timeout - Clock_getTicks()) <= 0) {
            // The other side couldn't get its UART in time.
            // Or else it had some other kind of error. Fail to
            // disconnected.
            Semaphore_post(uart_mutex);
            set_state(uart_id, PROTO_STATE_DIS); // TODO: Should this be idle?
        }
        break;
    }
}

// All four arms share the same function, even though they have separate tasks.
void serial_arm_task(UArg uart_id, UArg arg1) {
    int results_flag;
    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.

    if (uart_id == 0) {
//        PIN_close(arm_gpio_pin_handle);
//        uart_h = UART_open(uart_id, &uart_p);
    }
//
//    if (uart_id == 2) {
//        PIN_close(arm_gpio_pin_handle);
//        uart_h = UART_open(uart_id, &uart_p);
//        send_serial_handshake(uart_id);
//    }

    do {
        Task_sleep(100); // 1000 us

        if (uart_id == 0) {

            serial_handle_state_machine(0);


//            // STATE_DIS
//            while (!arm_read_in_debounced(uart_id)) {
//                // while reading low (disconnected).
//                // off = disconnected
//                arm_color(uart_id, 0,0,0);
//                Task_sleep(1000);
//            }
//
//            // STATE_PLUGGED
//            // white = connected
//            arm_color(uart_id, 100,100,100);
//
//            // Now it's high! We're connected.
//            // Give it a second to settle.
//            Task_sleep(100000);
//
//            // STATE_IDLE
//
//            // we're going to wait for the thing to go low, meaning either they want to send something
//            // or else we've been disconnected.
//
//            while (arm_read_in_debounced(uart_id)) Task_sleep(0); // wait while high.
//
//            // STATE_RTS_WAIT
//            // yellow = maybe disconnected
//            arm_color(uart_id, 100,100,0);
//
//            // now our input is low. We're going to respond by going low as well,
//            // once we have access to the UART.
//            Semaphore_pend(uart_mutex, BIOS_WAIT_FOREVER);
//            PINCC26XX_setOutputValue(arm_gpio_tx, 0);
//
//            // STATE_CTS_WAIT
//
//            // This low output should be acknowledged by a high. wait a moment to make sure
//            //  it actually happens. If it doesn't, we're disconnected and should restart.
//
//            Task_sleep(100000); // 1000000 us = 1000 ms
//
//            if (!PINCC26XX_getInputValue(arm_gpio_rx)) {
//                // still low, probably disconnected.
//                PINCC26XX_setOutputValue(arm_gpio_tx, 1);
//                arm_color(uart_id, 0,0,0);
//                continue;
//            }
//            // cyan = still connected, rts+cts:
//            arm_color(uart_id, 0,100,100);
//
//            // it went high. yay! That means they're still connected. They've gone high to set up
//            //  the ground state for the UART communications. So we need to do the same.
//            //  The other side will wait a moment before sending, but we don't need to do that.
//            //  It's high, which is where it's supposed to be so we can just _go_!
//            PINCC26XX_setOutputValue(arm_gpio_tx, 1);
//
//            PIN_close(arm_gpio_pin_handle);
//            uart_h = UART_open(uart_id, &uart_p);
//            arm_color(uart_id, 0,100,100);
//            results_flag = UART_read(uart_h, &uart_rx_buf, sizeof(serial_message_t));
//
//            if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
//                // something broke:
//                volatile UARTCC26XX_Object *o = (UARTCC26XX_Object *) uart_h->object;
//                o->status;
//                set_state(uart_id, PROTO_STATE_DIS);
//                // red = failed read:
//                arm_color(uart_id, 100,0,0);
//                UART_close(uart_h);
//                arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
//                Semaphore_post(uart_mutex);
//                Task_sleep(40000); // 400000 us = 400 ms
//            } else { // success:
//                // green = successful read:
//                arm_color(uart_id, 0,100,0);
//                UART_close(uart_h);
//                arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
//                Semaphore_post(uart_mutex);
//                Task_sleep(40000); // 400000 us = 400 ms
//            }
        }

        if (uart_id == 2) {
            while (!arm_read_in_debounced(uart_id)) {
                // while reading low (disconnected).
                // off = disconnected
                arm_color(uart_id, 0,0,0);
                Task_sleep(1000);
            }
            // white = connected
            arm_color(uart_id, 100,100,100);

            // Now it's high! We're connected.
            // Give it a few seconds to settle.
            Task_sleep(300000);

            // Now it's high! We're connected. Let's try doing a signal to send something...
            PINCC26XX_setOutputValue(arm_gpio_tx, 0); // Bring output low. Hopefully the other side goes low too...
            // cyan=rts sent
            arm_color(uart_id, 0,100,100);

            while (arm_read_in_debounced(uart_id)) Task_sleep(0); // wait while high.
            // it's low now.

            // Response received!
            // greener cyan=rts sent, cts received:
            arm_color(uart_id, 0,200,100);
            // Go high again to set up ground state for comms and wait for them to do the same:
            PINCC26XX_setOutputValue(arm_gpio_tx, 1);

            while (!arm_read_in_debounced(uart_id)) Task_sleep(0); // wait while low.
            // now it's high and we can send.
            // bright cyan=sending
            arm_color(uart_id, 0,255,255);
            Semaphore_pend(uart_mutex, BIOS_WAIT_FOREVER);
            PIN_close(arm_gpio_pin_handle);
            uart_h = UART_open(uart_id, &uart_p);

            Task_sleep(1000); // give them a moment in case they're running behind.

            results_flag = UART_write(uart_h, &uart_tx_buf, sizeof(serial_message_t));

            if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
                // something broke:
                volatile UARTCC26XX_Object *o = (UARTCC26XX_Object *) uart_h->object;
                o->status;
                arm_color(uart_id, 100,0,0);
                UART_close(uart_h);
                arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
                Semaphore_post(uart_mutex);
                Task_sleep(50000); // 500000 us = 500 ms
            } else {
                // sent successfully.
                arm_color(uart_id, 0,100,0);
                UART_close(uart_h);
                arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
                Semaphore_post(uart_mutex);
                Task_sleep(50000); // 500000 us = 500 ms
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
    uart_p.baudRate = 9600;
//    uart_p.readTimeout = RTS_TIMEOUT;
//    uart_p.writeTimeout = RTS_TIMEOUT;
    // TODO: Deal with these.
    uart_p.writeTimeout = 40000;
    uart_p.readTimeout = 40000;
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
