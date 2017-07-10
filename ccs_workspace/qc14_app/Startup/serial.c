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
            if (uart_id == 0)
                send_serial_handshake(uart_id); // TODO
        }
        if (arm_nts == SERIAL_MSG_TYPE_HANDSHAKE)
            arm_timeout = IDLE_BACKOFF_MS;
        else
            arm_timeout = 0;
        arm_color(uart_id, 255,255,255);
        break;
    case PROTO_STATE_RTS_WAIT:
//        arm_timeout = Clock_getTicks() + RTS_TIMEOUT; // TODO:
//        arm_color(uart_id, 0,0,100);
        break;
    case PROTO_STATE_CTS_WAIT:
        arm_timeout = RTS_TIMEOUT_MS;
//        arm_color(uart_id, 0,0,255);
        break;
    case PROTO_STATE_RTS_OUT:
        arm_color(uart_id, 0,255,0);
        // We're going to say we're RTS and then wait to hear a CTS back.
        PINCC26XX_setOutputValue(arm_gpio_tx, 0);
        arm_timeout = Clock_getTicks() + RTS_TIMEOUT;
        break;
    case PROTO_STATE_CTS_OUT:
        arm_color(uart_id, 0,255,0);
        PINCC26XX_setOutputValue(arm_gpio_tx, 1);
        arm_timeout = Clock_getTicks() + RTS_TIMEOUT;
        break;
    }
    arm_proto_state = dest_state;
}

void serial_handle_state_machine(UArg uart_id) {
    int results_flag;
    switch(arm_proto_state) {
    case PROTO_STATE_DIS:
        // input has pulldown, output high.
        // can react to IN going high.
        if (arm_read_in_debounced(uart_id)) {
            set_state(uart_id, PROTO_STATE_PLUGGING);
        }
        break;
    case PROTO_STATE_PLUGGING:
        arm_timeout = PLUG_TIMEOUT_MS;
        while (arm_timeout) {
            Task_sleep(100); // 1 ms
            if (!arm_read_same(uart_id))
                arm_timeout = PLUG_TIMEOUT_MS; // change, reset timeout
            else
                arm_timeout--; // no change, keep ticking
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
            // We could fall through here, but let's just loop back around and
            //  grab our section on the next iteration.
        } else if (arm_nts) { // We need to send:
            // Does the backoff apply (sending a handshake)?
            if (arm_nts == SERIAL_MSG_TYPE_HANDSHAKE && arm_timeout) {
                // Yes, backoff applies and we still need to wait.
                arm_timeout--;
                Task_sleep(100); // 1 ms.
            } else if (Semaphore_pend(uart_mutex, BIOS_NO_WAIT)) {
                // Either we don't need to wait, or we've already waited
                //  and we have the semaphore so we're ready to send.
                set_state(uart_id, PROTO_STATE_RTS_OUT);
            }
        }
        break;
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

        while (arm_timeout-- && !arm_read_in_debounced(uart_id)) {
            Task_sleep(100);
        }

        if (!arm_timeout) { // time ran out.
            // still low, probably disconnected.
            Task_sleep(RTS_TIMEOUT); // Pause for a while to give them something to think about.
            PINCC26XX_setOutputValue(arm_gpio_tx, 1);
            arm_color(uart_id, 0,0,0);
            arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
            Semaphore_post(uart_mutex);
            set_state(uart_id, PROTO_STATE_DIS);
            break;
        }

        // still connected! Yay!
        // cyan = still connected, rts+cts:
        arm_color(uart_id, 0,100,100);

        // it went high. yay! That means they're still connected. They've gone high to set up
        //  the ground state for the UART communications. So we need to do the same, by
        //  turning on the UART. The other side will watch for our idle-high output signal.

        PIN_close(arm_gpio_pin_handle);
        uart_h = UART_open(uart_id, &uart_p);
        arm_color(uart_id, 0,100,100);
        results_flag = UART_read(uart_h, &uart_rx_buf, sizeof(serial_message_t));

        if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
            // something broke:
            volatile UARTCC26XX_Object *o = (UARTCC26XX_Object *) uart_h->object;
            o->status;
            // red = failed read:
            arm_color(uart_id, 100,0,0);
            Task_sleep(40000); // 400000 us = 400 ms
            set_state(uart_id, PROTO_STATE_DIS);
        } else { // success:
            // green = successful read:
            arm_color(uart_id, 0,100,0);
            Task_sleep(40000); // 400000 us = 400 ms
            set_state(uart_id, PROTO_STATE_IDLE);
        }
        UART_close(uart_h);
        arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
        Semaphore_post(uart_mutex);
        break;
    case PROTO_STATE_RTS_OUT:
        // We've asserted our RTS (low). The expected response is to see
        //  an asserted (low) CTS within the timeout timeframe.

        if (!arm_read_in_debounced(uart_id)) {
            // We are clear to send. Acknowledge by deasserting RTS.
            set_state(uart_id, PROTO_STATE_CTS_OUT);
        } // or can react to TIMEOUT:
        else if ((int32_t) (arm_timeout - Clock_getTicks()) <= 0) {
            // The other side couldn't get its UART in time.
            // Or else it had some other kind of error. Fail to
            // disconnected.
            Semaphore_post(uart_mutex);
            set_state(uart_id, PROTO_STATE_DIS); // TODO: Should this be idle?
        }
        break;
    case PROTO_STATE_CTS_OUT:
        // We actually now need to wait for the other side to go high, too.
        //  That's our signal that their UART has been turned on.

        if (!arm_read_in_debounced(uart_id)) {
            // We are good to go. The high input is coming from the other
            //  badge's UART peripheral.

            // Tear down my GPIO and prepare to switch to the peripheral:
            PIN_close(arm_gpio_pin_handle);
            uart_h = UART_open(uart_id, &uart_p);

            // Do our TX:
            results_flag = UART_write(uart_h, &uart_tx_buf, sizeof(serial_message_t));
            arm_nts = SERIAL_MSG_TYPE_NOMSG;

            // Shut down peripheral, switch to GPIO, and release the mutex.
            UART_close(uart_h);
            arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table);
            Semaphore_post(uart_mutex);

            // Done sending, so we can cancel this flag:
            arm_nts = 0;

            if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
                // something broke:
                volatile UARTCC26XX_Object *o = (UARTCC26XX_Object *) uart_h->object;
                o->status;
                // red = failed:
                arm_color(uart_id, 100,0,0);
                Task_sleep(40000); // 400000 us = 400 ms
                set_state(uart_id, PROTO_STATE_DIS);
            } else { // success:
                // green = successful:
                arm_color(uart_id, 0,100,0);
                Task_sleep(40000); // 400000 us = 400 ms
                set_state(uart_id, PROTO_STATE_IDLE);
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

#define STATE_DIS 0
#define STATE_CON 1

// All four arms share the same function, even though they have separate tasks.
void serial_arm_task(UArg uart_id, UArg arg1) {
    uint8_t state=STATE_DIS;
    uint32_t timeout_ms = PLUG_TIMEOUT;

    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.

    do {
        Task_sleep(1);
        serial_handle_state_machine(uart_id);
    } while (1);

    do {
        // A "continue" takes us here:
        Task_sleep(0); // yield.

        if (state == STATE_DIS) { // We are disconnected.
            while (timeout_ms) {
                if (arm_read_in_debounced(uart_id))
                    timeout_ms--;
                else
                    timeout_ms = STATE_DIS;
                Task_sleep(100); // 1 ms.
            }
            // we are now connected, can fall through:
            state = STATE_CON;
        }

        if (state==STATE_CON) { // We are connected.
            if (arm_nts) { // need to send
                /*
                 *  If we DO need to send:
                 *   Get the semaphore.
                 *   Set our input LOW.
                 *   Wait for input LOW.
                 *   They have accepted, or have disconnected. Set output HIGH.
                 *   Wait with timeout for input HIGH:
                 *      If we time out, we are DISCONNECTED.
                 *      If we receive input HIGH then we need to send a message. (the HIGH came from other side's UART)
                 *      Switch to UART mode
                 *      Send.
                 *      We are now IDLE. On error, we are DISCONNECTED.
                 */
                // TODO: fill this in:
            } else { // don't need to send.
                /*
                 *  If we DON'T need to send:
                 *   Listen for input LOW.
                 *   Get the semaphore, and bring our input LOW.
                 *   Wait with timeout for input HIGH:
                 *      If we time out, we are DISCONNECTED.
                 *      If we receive input HIGH then we need to receive a message.
                 *      Switch to UART mode (idle has output high).
                 *      Receive.
                 *      Process message.
                 *      We are now connected and idle. On error, we are DISCONNECTED.
                 */
                if (arm_read_in_debounced(uart_id))
                    continue; // Back to the start.

                // We got a LOW input.
                // Get the semaphore.
                Semaphore_pend(uart_mutex, BIOS_WAIT_FOREVER);
                // Set our own low:
                PINCC26XX_setOutputValue(arm_gpio_tx, 0);
                // Wait with a timeout for a HIGH input:
                if (!wait_with_timeout(uart_id, 1)) { // TODO: write this.
                    // timed out. We are DISCONNECTED.
                    state = STATE_DIS;
                    continue; // back to the beginning.
                }
                // We got the HIGH we needed.
                // Time to set up to receive. A byproduct of this is sending a HIGH.
                PIN_close(arm_gpio_pin_handle);
                uart_h = UART_open(uart_id, &uart_p);
                results_flag = UART_read(uart_h, &uart_rx_buf, sizeof(serial_message_t));
                // Got the message. Check for errors:

                if (results_flag == UART_ERROR || results_flag<sizeof(serial_message_t)) {
                    // Error. We are now disconnected.
                    volatile UARTCC26XX_Object *o = (UARTCC26XX_Object *) uart_h->object;
                    o->status;
                    state = STATE_DIS;
                } else { // success:
                    // Successful read.
                    // Process message here.
                }
                UART_close(uart_h);
                arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
                Semaphore_post(uart_mutex);
                // Free up resources, loop again.
                continue;
            }
        }
    } while (1);



    // Let's try something different.
    /*
     *
     * While disconnected:
     *  Listen for input HIGH.
     *  Wait for it to stay HIGH for PLUG_TIMEOUT.
     *  We are now connected and idle.
     *
     * While connected and idle:
     *  If we DON'T need to send:
     *   Listen for input LOW.
     *   Get the semaphore, and bring our input LOW.
     *   Wait with timeout for input HIGH:
     *      If we time out, we are DISCONNECTED.
     *      If we receive input HIGH then we need to receive a message.
     *      Switch to UART mode (idle has output high).
     *      Receive.
     *      Process message.
     *      We are now connected and idle. On error, we are DISCONNECTED.
     *  If we DO need to send:
     *   Get the semaphore.
     *   Set our input LOW.
     *   Wait for input LOW.
     *   They have accepted, or have disconnected. Set output HIGH.
     *   Wait with timeout for input HIGH:
     *      If we time out, we are DISCONNECTED.
     *      If we receive input HIGH then we need to send a message. (the HIGH came from other side's UART)
     *      Switch to UART mode
     *      Send.
     *      We are now IDLE. On error, we are DISCONNECTED.
     *
     *
     */




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
//    uart_p.writeTimeout = 400000;
//    uart_p.readTimeout = 400000;
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
