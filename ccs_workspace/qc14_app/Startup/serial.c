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
#define BAUDRATE 57600
#define PLUG_INTERVAL 500
#define UNPLUG_INTERVAL_MS (8 * 1000 * sizeof(serial_message_t) / BAUDRATE + 1)

serial_message_t uart_tx_buf[4] = {0};
// NB: This must be protected by the semaphore:
serial_message_t arm_rx_buf;

UART_Handle uart_h;
UART_Params uart_p;
Semaphore_Handle uart_mutex;
Semaphore_Handle tx_done_sem;

volatile uint8_t active_arm = 255;

Task_Struct uart_arm_tasks[4];
char uart_arm_task_stacks[4][320];

uint32_t uart_timeout[4] = {0, 0, 0, 0};
uint8_t uart_proto_state[4] = {0, 0, 0, 0};
uint8_t uart_nts_flag[4] = {0, 0, 0, 0};
uint8_t icontile_state[] = {0, 0, 0, 0};

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
#define arm_icontile_state icontile_state[uart_id]

void setup_tx_buf_no_payload(UArg uart_id) {
    arm_tx_buf.badge_id = my_conf.badge_id;
    arm_tx_buf.current_time = my_conf.csecs_of_queercon;
    arm_tx_buf.current_time_authority = my_conf.time_is_set;
    arm_tx_buf.arm_id = uart_id;
    if (arm_tx_buf.msg_type & SERIAL_MSG_FLAG_ACK) {
        arm_tx_buf.msg_type &= 0b01111111;
        arm_tx_buf.ack = 1;
    }
    arm_tx_buf.crc = crc16((uint8_t *) &arm_rx_buf,
                           sizeof(serial_message_t) - 2);
}

inline void send_serial_handshake(UArg uart_id, uint8_t ack) {
    arm_tx_buf.msg_type = SERIAL_MSG_TYPE_HANDSHAKE;
    uint8_t* payload = arm_tx_buf.payload;
    serial_handshake_t* handshake_payload = (serial_handshake_t*) payload;
    handshake_payload->current_mode = ui_screen;
    handshake_payload->current_icon_or_tile_id = (ui_screen? my_conf.current_tile : my_conf.current_icon);
    handshake_payload->ack = ack;
    memcpy(handshake_payload->badges_mated, my_conf.badges_mated, BADGES_MATED_BYTES);
    arm_nts = SERIAL_MSG_TYPE_HANDSHAKE;
}

uint8_t rx_valid(UArg uart_id) {
    if (arm_rx_buf.badge_id > BADGES_IN_SYSTEM)
        return 0;
    if (arm_rx_buf.msg_type > SERIAL_MSG_TYPE_MAX)
        return 0;
    // TODO:
//    if (arm_rx_buf.badge_id == my_conf.badge_id)
//        return 0;
    if (arm_rx_buf.crc != crc16((uint8_t *) &arm_rx_buf,
                                sizeof(serial_message_t) - 2))
        return 0;
    return 1;
}

void connection_opened(UArg uart_id) {
    set_badge_mated(arm_rx_buf.badge_id);
    // We take our clock setting from this person if:
    //  1. They are authoritative and we are not, OR
    //  2. Neither of us are authoritative and they are ahead of us.

    if (!my_conf.time_is_set &&
            (arm_rx_buf.current_time_authority ||
             arm_rx_buf.current_time > my_conf.csecs_of_queercon)) {
        set_clock(arm_rx_buf.current_time);
        my_conf.time_is_set = arm_rx_buf.current_time_authority;
    }

    arm_icontile_state = ICONTILE_STATE_OPEN;
    arm_color(uart_id, 255,255,255);
    if (ui_screen == UI_SCREEN_GAME &&
            ((serial_handshake_t*) &arm_rx_buf.payload)->current_mode == UI_SCREEN_GAME) {
        // We're doing game things!
    } else if (ui_screen == UI_SCREEN_TILE &&
            ((serial_handshake_t*) &arm_rx_buf.payload)->current_mode == UI_SCREEN_TILE) {
        // We're doing color tile things!
    } else {
        // We're not uesful to each other.
        for (uint8_t i=255; i>0; i--) {
            arm_color(uart_id, i, 0, 0);
            Task_sleep(500); // 500 * 10 us = 50 ms // TODO: bad bad bad
        }
        arm_color(uart_id, 0, 0, 0);
    }
}

void rx_timeout(UArg uart_id) {
    // TODO: If we were waiting on an ack, resend the message it was
    //       supposed to acknowledge.

    switch(arm_icontile_state) {
    case ICONTILE_STATE_DIS:
        // Unreachable
        break;
    case ICONTILE_STATE_HS1:
        // We've sent an ACK=0 message but haven't heard back.
        //  Need to re-send.
        send_serial_handshake(uart_id, 0);
        break;
    case ICONTILE_STATE_HS2:
        // We've sent an ACK=1 message
        //  A timeout likely means they didn't hear our reply, so
        //  let's repeat it:
        send_serial_handshake(uart_id, 1);
        break;
    case ICONTILE_STATE_HS3:
        // A timeout here means we either got garbage (let's assume it was an
        //  ACK), or everything is fine and we've just waited a while.
        connection_opened(uart_id);
        break;
    case ICONTILE_STATE_OPEN:
        // Nothing to worry about - we often don't receive messages in
        //  this state.
        break;
    }
}

void rx_done(UArg uart_id) {
    switch(arm_icontile_state) {
    case ICONTILE_STATE_DIS:
        // Unreachable
        break;
    case ICONTILE_STATE_HS1:
        // We've sent an ACK=0 message.
        if (((serial_handshake_t*) &arm_rx_buf.payload)->ack) {
            // They acked ours, let's ack theirs.
            arm_icontile_state = ICONTILE_STATE_HS3;
            send_serial_handshake(uart_id, 1);
        } else {
            // They didn't ack ours, but we can ack theirs.
            send_serial_handshake(uart_id, 1);
            arm_icontile_state = ICONTILE_STATE_HS1;
        }
        break;
    case ICONTILE_STATE_HS2:
        // We've sent an ACK=1 message
        if (((serial_handshake_t*) &arm_rx_buf.payload)->ack) {
            // They acked ours, we've acked theirs. Move on...
            arm_icontile_state = ICONTILE_STATE_HS3;
        } else {
            // They didn't ack ours, need to resend:
            send_serial_handshake(uart_id, 1);
            // Stay in the same state.
        }
        break;
    case ICONTILE_STATE_HS3:
        if (((serial_handshake_t*) &arm_rx_buf.payload)->ack) {
            // OK, just got a spurious ACK. This can happen.
            //  We're going to ignore it, though.
        }
        break;
    case ICONTILE_STATE_OPEN:
        break;
    }
}

void uart_tx_done(UART_Handle h, void *buf, size_t count) {
    if (count != sizeof(serial_message_t)) {
        Semaphore_post(tx_done_sem);
        return; // TODO, broken message
    }

    Semaphore_post(tx_done_sem);
}

uint8_t serial_in_progress() {
    return uart_proto_state[0] || uart_proto_state[1] || uart_proto_state[2] || uart_proto_state[3];
}

void disconnected(UArg uart_id) {
    arm_color(uart_id, 0,0,0);
    arm_proto_state=SERIAL_PHY_STATE_DIS;
    arm_icontile_state = ICONTILE_STATE_DIS;
    arm_nts = SERIAL_MSG_TYPE_NOMSG;
    PINCC26XX_setOutputValue(arm_gpio_tx, 0);
    Task_sleep(PLUG_INTERVAL*2);
    PINCC26XX_setOutputValue(arm_gpio_tx, 1);
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

        if (arm_proto_state == SERIAL_PHY_STATE_DIS) {
            // We're physically unplugged.
            // Wait forever for our input to go high for PLUG_INTERVAL,
            // yielding execution during that loop.
            wait_with_timeout(uart_id, 1, UINT32_MAX, PLUG_INTERVAL);
            // We are physically connected, but not logically.
            // Prep a logical handshake.
            send_serial_handshake(uart_id, 0);
            arm_icontile_state = ICONTILE_STATE_HS1;
            arm_proto_state = SERIAL_PHY_STATE_CON;
        }

        // If we're physically plugged but not connected, keep
        // looping in this thread without yielding.
        // If we're disconnected, we don't need to yield because
        // wait_with_timeout does that for us.
        // If we're both physically connected AND logically coupled,
        // then it's OK to yield.

        // TODO: Deadlock is possible between these if multiple badges
        //  get plugged into each other at the same time.
        // Can we check to see if anyone is pending on the semaphore?
        //  Or if any other threads are in this state?
        if (arm_proto_state == SERIAL_PHY_STATE_CON &&
                arm_icontile_state != ICONTILE_STATE_OPEN) {
            Task_yield();
        }

        // Wait for the semaphore for the length of time it takes to send
        //  four messages. (this will yield if someone else has the sem)
        if (!Semaphore_pend(uart_mutex, 4 * UNPLUG_INTERVAL_MS))
            continue; // If we didn't get it, loop.

        // We have the semaphore.
        // Setup the UART.
        PIN_close(arm_gpio_pin_handle);
        uart_h = UART_open(uart_id, &uart_p);

        // Sending is non-blocking, so do it if we need to:
        if (arm_nts) {
            arm_color(uart_id, 100,0,0);

            // Setup the non-payload parts of the message:
            //  (the payload parts are configured by the function that sets
            //   the NTS flag.)
            setup_tx_buf_no_payload(uart_id);
            // Non-blocking, send it:
            UART_write(uart_h, &arm_tx_buf, sizeof(serial_message_t));
            // If we need to re-send, we'll re-set this flag from the
            //  tx_done function or elsewhere, so for now cancel it:
            arm_nts = SERIAL_MSG_TYPE_NOMSG;
        } else {
            // If we don't need to send, make sure that the TX semaphore is
            //  available.
            Semaphore_post(tx_done_sem);
        }

        // Listen for a message. BLOCKING:
        results_flag = UART_read(uart_h, &arm_rx_buf, sizeof(serial_message_t));

        // Wait for our write to finish, and cancel it if it's broken.
        if (!Semaphore_pend(tx_done_sem, RTS_TIMEOUT*2)) {
            // write error well past timeout. bork bork.
            UART_writeCancel(uart_h);
        }

        if (results_flag!=sizeof(serial_message_t) || !rx_valid(uart_id)) {
            // We didn't get a good message. We'll have to keep listening.
            rx_timeout(uart_id);

        } else { // success. It's valid, right size, and no error.
            // Good, back to connected.
            arm_color(uart_id, 10,10,10);
            // Process message.
            rx_done(uart_id); // determine need to ack, here.
        }

        // Yield ALL THE THINGS
        UART_close(uart_h);
        arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state,
                                       arm_gpio_init_table);
        Semaphore_post(uart_mutex);
    } while (1);
}

void serial_init() {
    Semaphore_Params uart_mutex_params;
    Semaphore_Params_init(&uart_mutex_params);
    uart_mutex_params.mode = Semaphore_Mode_BINARY;
    uart_mutex = Semaphore_create(1, &uart_mutex_params, NULL);

    Semaphore_Params tx_done_sem_params;
    Semaphore_Params_init(&tx_done_sem_params);
    tx_done_sem_params.mode = Semaphore_Mode_BINARY;
    tx_done_sem = Semaphore_create(0, &tx_done_sem_params, NULL);

    UART_Params_init(&uart_p);
    // Defaults used:
    // blocking reads and writes, no write timeout, 8N1.
    uart_p.baudRate = BAUDRATE;
    uart_p.readTimeout = RTS_TIMEOUT*4;
    uart_p.writeTimeout = RTS_TIMEOUT;
    uart_p.readMode = UART_MODE_BLOCKING;
    uart_p.writeMode = UART_MODE_CALLBACK;
    uart_p.writeCallback = uart_tx_done;
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
