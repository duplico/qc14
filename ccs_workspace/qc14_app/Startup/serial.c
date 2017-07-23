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
#define RX_TIMEOUTS_TO_IDLE 20

serial_message_t uart_tx_buf[4] = {0};
uint8_t tx_bytes[sizeof(serial_message_t)+2];
// NB: This must be protected by the semaphore:
serial_message_t arm_rx_buf;
uint8_t rx_bytes[sizeof(serial_message_t)+2];

// This, too:
uint8_t rx_timeouts_to_idle = RX_TIMEOUTS_TO_IDLE;

UART_Handle uart_h;
UART_Params uart_p;
Semaphore_Handle uart_mutex;
Semaphore_Handle rx_done_sem;

Task_Struct uart_arm_tasks[4];
char uart_arm_task_stacks[4][400];

uint32_t uart_timeout[4] = {0, 0, 0, 0};
uint8_t uart_proto_state[4] = {0, 0, 0, 0};
uint8_t uart_nts_flag[4] = {0, 0, 0, 0};
uint8_t uart_fop_flag[4] = {0, 0, 0, 0}; // "first open"
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
#define arm_fop uart_fop_flag[uart_id]
#define arm_timeout uart_timeout[uart_id]
#define arm_phy_state uart_proto_state[uart_id]
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
    arm_tx_buf.crc = crc16((uint8_t *) &arm_tx_buf,
                           sizeof(serial_message_t) - 4);
    memcpy(&tx_bytes, (uint8_t *) &arm_tx_buf, sizeof(serial_message_t));
}

inline void send_serial_handshake(UArg uart_id, uint8_t ack) {
    arm_tx_buf.msg_type = SERIAL_MSG_TYPE_HANDSHAKE;
    uint8_t* payload = arm_tx_buf.payload;
    serial_handshake_t* handshake_payload = (serial_handshake_t*) payload;
    handshake_payload->current_mode = ui_screen;
    handshake_payload->current_icon_or_tile_id = (ui_screen? my_conf.current_tile : my_conf.current_icon);
    handshake_payload->ack = ack;
    handshake_payload->in_fabric = tile_active;
    handshake_payload->fabric_offset = tile_offset;

    handshake_payload->msg_ready = 0;
    // Are we waiting on this arm to get connected?
    for (uint8_t i=0; i<4; i++) {
        if (i==uart_id) continue;
        // In other words, does there exist an arm, other than this one,
        //  which is connected (correctly), is waiting for a connection,
        //  and the connection it's waiting for is on this arm?
        if (game_arm_status[i].connected &&
                game_arm_status[i].sufficiency_info == GAME_SUFFICIENT_CONN &&
                game_curr_icon.arms[i].other_arm_id == uart_id) {
            // If so, then we need to set the payload.
            handshake_payload->msg_ready = 1;
        }
    }

    memcpy(handshake_payload->badges_mated, my_conf.badges_mated, BADGES_MATED_BYTES);
    arm_nts = SERIAL_MSG_TYPE_HANDSHAKE;
}

inline void send_serial_tile_msg(UArg uart_id) {
}

inline void send_serial_game_msg(UArg uart_id) {
    arm_tx_buf.msg_type = SERIAL_MSG_TYPE_GAME;
    uint8_t* payload_buf = arm_tx_buf.payload;
    serial_game_msg_t* payload = (serial_game_msg_t*) payload_buf;

    payload->conn_msg = 1;
    payload->conn_result = game_curr_icon.arms[uart_id].result_icon_id;

    game_arm_status[uart_id].nts_done = 1;

    arm_nts = SERIAL_MSG_TYPE_GAME;
}

uint8_t rx_valid(UArg uart_id) {
    if (arm_rx_buf.badge_id > BADGES_IN_SYSTEM)
        return 0;
    if (arm_rx_buf.msg_type > SERIAL_MSG_TYPE_MAX)
        return 0;
    if (arm_rx_buf.arm_id > 3)
        return 0;
    if (arm_rx_buf.crc != crc16((uint8_t *) &arm_rx_buf.badge_id,
                                sizeof(serial_message_t) - 4))
        return 0;
    return 1;
}

uint8_t process_game_open(UArg uart_id, uint8_t icon_id) {
    // Returns 0 if this connection is rejected.
    // Return 1 if this connection is accepted.
    serial_handshake_t* payload = (serial_handshake_t*) &arm_rx_buf.payload;

    // In general, getting here means that our line is not connectable.
    //  That could change lower down, though, if it's a good match.
    game_arm_status[uart_id].connectable = 0;

    if (icon_id != game_curr_icon.arms[uart_id].mate_icon_id)
        return 0;
    if (arm_rx_buf.arm_id != ((uart_id + 2) % 4))
        return 0; // In game mode, we only accept connections from opposite arm

    game_arm_status[uart_id].connected = 1;
    game_arm_status[uart_id].icon_id = icon_id;

    // If we're here, this was a match.
    switch(game_curr_icon.arms[uart_id].sufficient_flag) {
    case GAME_SUFFICIENT_MSG:
        // I'm a LEAF BADGE.
        //  (watch how I soar)
        // We need a message from this person in order to transition.
        //  Let us prepare our bodies.

        // It's possible that person is already connected to the badge we need.
        //  Let's find out.
        if (!payload->msg_ready) {
            // There is nothing yet to do. Yet.

            // Preempt all *other* arms
            for (uint8_t i=0; i<4; i++) {
                if (i == uart_id)
                    game_arm_status[i].connectable = 1; // Keep mine lit up.
                else
                    game_arm_status[i].connectable = 0;
            }

            game_arm_status[uart_id].sufficiency_info = GAME_SUFFICIENT_MSG;

            // Bail for now.
            return 1;
        }

        // We are ready to transform. We'll let the other badge take care
        //  of telling its neighbor, but we're going to go ahead and
        //  dooooo iiiiiit.
        //   fall through!
    case GAME_SUFFICIENT_ALONE:
        // This means that this match is good enough to do the transition.

        // Preempt all arms.
        for (uint8_t i=0; i<4; i++) {
            game_arm_status[i].connectable = 0;
        }

        // Covfefe is forever.
        if (my_conf.current_icon == ICON_COFFEE)
            do_icon_transition(ICON_COFFEE);

        game_arm_status[uart_id].sufficiency_info = GAME_SUFFICIENT_ALONE;
        do_icon_transition(game_curr_icon.arms[uart_id].result_icon_id);
        return 1;
    case GAME_SUFFICIENT_CONN:
        // I'm the MIDDLE BADGE.
        // We need another badge physically plugged into ourselves in order
        //  to transition. It may already be here, or we may need to wait
        //  for it.

        game_arm_status[uart_id].sufficiency_info = GAME_SUFFICIENT_CONN;

        if (game_arm_status[game_curr_icon.arms[uart_id].other_arm_id].connected &&
                game_arm_status[game_curr_icon.arms[uart_id].other_arm_id].sufficiency_info == GAME_SUFFICIENT_CONN) {
            // The other arm is already connected. That's pretty sweet.
            //  And! It's the one we're looking for, waiting for a connection.
            // Remember, it can only be connected if that arm is enabled. And,
            //  the sufficiency_info is set to GAME_SUFFICIENT_CONN by the
            //  other side of the else statement below.

            // Our handshake already told this newly connected badge everything
            //  it needs to know in order to transform, itself. So now we
            //  just need to tell the other one, and then transform ourselves.

            // Set the need to send the message, and the result,
            //  in our status struct:
            game_arm_status[game_curr_icon.arms[uart_id].other_arm_id].nts = 1;

            // We are now ready to transition as soon as we communicate to our
            //  other leaf badge that it's transition time.
        } else {
            // The other arm is disconnected. Bummer. But it's OK.
            //  When it plugs in, we'll automatically TRANSFORM!
            //  I know, because the code to do it is right above us, on the
            //  other side of this else.
            // Preempt all arms other than what we're waiting for.
            game_arm_status[uart_id].sufficiency_info = GAME_SUFFICIENT_CONN;
            for (uint8_t i=0; i<4; i++) {
                game_arm_status[i].connectable = (i==uart_id ||
                        i == game_curr_icon.arms[uart_id].other_arm_id);
            }
        }
        return 1;
    }
    return 0;
}

uint8_t process_tile_open(UArg uart_id) {
    serial_handshake_t* payload = (serial_handshake_t*) &arm_rx_buf.payload;

    tile_arm_status[uart_id].connected = 1;
    tile_arm_status[uart_id].remote_arm_id = arm_rx_buf.arm_id;

    if (tile_active) {
        // Already in a fabric.
        return 1;
    } else if (payload->in_fabric || my_conf.badge_id > arm_rx_buf.badge_id) {
        // We're not in a fabric, and our new mate _is_. OR, neither of us
        //  is in a fabric, but their ID is lower than ours and therefore
        //  controls.
        // Regardless, we adopt our offset based on theirs and which of
        //  THEIR arms we plug into.
        tile_offset = payload->fabric_offset + tile_frame_periods[my_conf.current_tile][arm_rx_buf.arm_id];

        // Also, since the other side controls, we need to switch our tile to
        //  theirs if it's different. This is to make things look better.

        switch_to_tile(payload->current_icon_or_tile_id, 0);

    } else {
        // Neither of us is in a fabric, and the tie-breaker has decided that
        //  I don't have to do anything to my offset. Because I am awesome.
    }

    // If we're down here, we're joining a fabric.
    tile_active = 1;


    return 1;
}

void connection_opened(UArg uart_id) {
    set_badge_mated(arm_rx_buf.badge_id);

    arm_icontile_state = ICONTILE_STATE_OPEN;
    arm_phy_state = SERIAL_PHY_STATE_PLUGGED;
    if (ui_screen == UI_SCREEN_GAME)
        outer_arm_color_rgb(uart_id, game_curr_icon.arms[uart_id].arm_color);
    else // tile:
        outer_arm_color(uart_id, 5, 5, 5);

    if (ui_screen == UI_SCREEN_GAME &&
            ((serial_handshake_t*) &arm_rx_buf.payload)->current_mode == UI_SCREEN_GAME &&
            process_game_open(uart_id, ((serial_handshake_t*) &arm_rx_buf.payload)->current_icon_or_tile_id)) {
        // We're doing game things!
    } else if (ui_screen == UI_SCREEN_TILE &&
            ((serial_handshake_t*) &arm_rx_buf.payload)->current_mode == UI_SCREEN_TILE &&
            process_tile_open(uart_id)) {
        // We're doing color tile things!
    } else {
        if (ui_screen == UI_SCREEN_GAME)
            game_arm_status[uart_id].connectable = 0;
        // We're not useful to each other.
        for (uint8_t i=255; i>0; i--) {
            arm_color(uart_id, i, 0, 0);
            Task_sleep(500); // 500 * 10 us = 50 ms
        }
        arm_color(uart_id, 0, 0, 0);
    }

}

uint8_t serial_in_progress() {
    return uart_proto_state[0] || uart_proto_state[1] || uart_proto_state[2] || uart_proto_state[3];
}

void disconnected(UArg uart_id) {
    outer_arm_color(uart_id, 0,0,0);
    arm_phy_state=SERIAL_PHY_STATE_DIS;

    if (ui_screen == UI_SCREEN_GAME) {
        // We need to clean up the game_arm_status setup.
        game_arm_status[uart_id].connected = 0;
        // Leave my own connectability unchanged.
        game_arm_status[uart_id].nts = 0;
        game_arm_status[uart_id].nts_done = 0;
        game_arm_status[uart_id].icon_id = 255;
        game_arm_status[uart_id].sufficiency_info = 0;
        if (serial_in_progress()) {
            // If we're expecting someone to plug in here, go ahead and make
            //  it connectable again:

            for (uint8_t i=0; i<4; i++) {
                if (i == uart_id) continue;
                if (game_arm_status[i].connected &&
                        game_arm_status[i].sufficiency_info &&
                        game_curr_icon.arms[i].sufficient_flag != GAME_SUFFICIENT_ALONE &&
                        game_curr_icon.arms[i].other_arm_id == uart_id)
                    game_arm_status[uart_id].connectable = 1;
            }
        } else {
            // We're the last to disconnect.
            // Make everybody else connectable.
            for (uint8_t i=0; i<4; i++) {
                game_arm_status[i].connectable = 1;
            }
        }
    } else {
        // Handle tile cleanup.
        tile_arm_status[uart_id].connected = 0;
        tile_arm_status[uart_id].neighbor_fully_connected = 0;
        tile_arm_status[uart_id].nts = 0;
        tile_arm_status[uart_id].nts_done = 0;

        arm_color(uart_id, 0, 0, 0);

        if (!serial_in_progress()) {
            tile_active = 0;
            tile_offset = 0;
        }

    }

    arm_icontile_state = ICONTILE_STATE_DIS;
    PINCC26XX_setOutputValue(arm_gpio_tx, 0);
    Task_sleep(RTS_TIMEOUT*1.2);
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

uint8_t arm_connectable(UArg uart_id) {
    if (ui_screen != UI_SCREEN_GAME && ui_screen != UI_SCREEN_TILE)
        return 0;

    // We're not connectable if we're in game mode and this arm has been
    //  turned off by another arm.
    if (ui_screen == UI_SCREEN_GAME && !game_arm_status[uart_id].connectable)
        return 0;

    return 1;
}

void block_until_plugged(UArg uart_id) {
    uint32_t plugin_timeout_ms = PLUG_TIMEOUT_MS;
    if (arm_phy_state == SERIAL_PHY_STATE_DIS) { // We are disconnected.
        /*
         * While disconnected:
         *  Listen for input HIGH.
         *  Wait for it to stay HIGH for PLUG_TIMEOUT.
         *  We are now plugged and idle.
         */

        PINCC26XX_setOutputValue(arm_gpio_tx, 1);

        while (plugin_timeout_ms) {
            // This is the only place we can get connected,
            // so there's no need to leave this loop. We do,
            // however, need to yield so that other threads
            // (including other arms) can actually function.

            if (arm_connectable(uart_id)) {
                PINCC26XX_setOutputValue(arm_gpio_tx, 1);
            } else {
                PINCC26XX_setOutputValue(arm_gpio_tx, 0);
            }

            if (PINCC26XX_getInputValue(arm_gpio_rx) && arm_connectable(uart_id)) {
                plugin_timeout_ms--;
            } else {
                plugin_timeout_ms = PLUG_TIMEOUT_MS;
            }
            // We're only allowed to get connected if we're in the tile or
            //  game mode.
            // So if we're in those modes we'll go ahead and do the normal
            //  sleeping thing. Otherwise start the whole business over.

            Task_sleep(100); // 1 ms.
        }

        // we are now plugged, can fall through:
        arm_phy_state = SERIAL_PHY_STATE_PLUGGED;
        arm_fop = 1;

        // We're going to take this opportunity to set all our other arms
        //  non-connectable temporarily. (if we're playing the game.)
        if (ui_screen == UI_SCREEN_GAME) {
            for (uint8_t i=0; i<4; i++) {
                if (game_arm_status[i].connected || i==uart_id)
                    continue;
                game_arm_status[i].connectable = 0;
            }
        }

    }
}

uint8_t do_phy_handshake_rx(UArg uart_id) {
    if (wait_with_timeout(uart_id, 0, 500, 1)) { // this blocks for 500 ms
        // Secure our own UART before coming to the assistance of
        //  others.
        Semaphore_pend(uart_mutex, BIOS_WAIT_FOREVER);

        // Signal we heard it by also sending a low.
        PINCC26XX_setOutputValue(arm_gpio_tx, 0);
        // The response to our LOW should be a HIGH. If we got it,
        //  great, it was a request to take this serial port active.
        //  Otherwise it was a disconnect.
        if (!wait_with_timeout(uart_id, 1, RTS_TIMEOUT_MS, SERIAL_SETTLE_TIME_MS)) {
            // timed out. We are DISCONNECTED.
            Semaphore_post(uart_mutex);
            disconnected(uart_id);
            return 0;
        }
        return 1; // leave with our output LOW.
    }
    return 0; // no handshake incoming
}

uint8_t do_phy_handshake_tx(UArg uart_id) {
    /*
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
    if (!Semaphore_pend(uart_mutex, BIOS_NO_WAIT))
        return 0; // didn't get the semaphore, yield. We don't wait here in case the other side signals.

    // Output low to signal RTS.
    PINCC26XX_setOutputValue(arm_gpio_tx, 0);
    // Wait to get a low input (means CTS):
    if (!wait_with_timeout(uart_id, 0, RTS_TIMEOUT_MS, SERIAL_SETTLE_TIME_MS)) {
        // If they never go low, it means they either missed our message
        //  somehow, or they were unable to get their semaphore in time.
        //  Regardless, let's time out and try again later.
        Semaphore_post(uart_mutex);
        return 0;
    }

    // Got low input. They've either accepted, or disconnected.
    // Set output high and wait with timeout for high input.
    PINCC26XX_setOutputValue(arm_gpio_tx, 1);
    if (!wait_with_timeout(uart_id, 1, RTS_TIMEOUT_MS, SERIAL_SETTLE_TIME_MS)) {
        // It's conceivable they disconnected at the same time we signaled our
        //  desire to send. Whoops. Clean up.
        Semaphore_post(uart_mutex);
        disconnected(uart_id);
        return 0;
    }
    // We got everything we needed and we're now active and ready to go.
    return 1; // Leave with our output HIGH.
}

void uart_rx_done(UART_Handle h, void *buf, size_t count) {

    if (count != sizeof(serial_message_t)) {
        Semaphore_post(rx_done_sem);
        return;
    }

    Semaphore_post(rx_done_sem);
}

void arm_disp(UArg uart_id) {
    switch (arm_icontile_state) {
    case ICONTILE_STATE_DIS:
        outer_arm_color(uart_id, 255,0,0);
        break;
    case ICONTILE_STATE_HS0:
        outer_arm_color(uart_id, 255,255,0);
        break;
    case ICONTILE_STATE_HS1:
        outer_arm_color(uart_id, 0,255,0);
        break;
    case ICONTILE_STATE_HS2:
        outer_arm_color(uart_id, 255,0,255);
        break;
    case ICONTILE_STATE_OPEN_WAIT:
        outer_arm_color(uart_id, 0,0,255);
        break;
    case ICONTILE_STATE_OPEN:
        break;
    }
}

void new_plug(UArg uart_id) {
    switch(arm_icontile_state) {
    case ICONTILE_STATE_DIS:
        arm_icontile_state = ICONTILE_STATE_HS0;
        // Wait for an RX timeout to send.
        break;
    default:
        break;
    }
    arm_disp(uart_id);
}

void rx_timeout(UArg uart_id) {
    switch (arm_icontile_state) {
    case ICONTILE_STATE_DIS:
        break;
    case ICONTILE_STATE_HS0:
        send_serial_handshake(uart_id, 0);
        break;
    case ICONTILE_STATE_HS1:
        send_serial_handshake(uart_id, 1);
        break;
    case ICONTILE_STATE_HS2:
        arm_icontile_state = ICONTILE_STATE_OPEN_WAIT;
        rx_timeouts_to_idle = RX_TIMEOUTS_TO_IDLE;
        break;
    case ICONTILE_STATE_OPEN_WAIT:
        arm_icontile_state = ICONTILE_STATE_OPEN_WAIT2;
        rx_timeouts_to_idle = RX_TIMEOUTS_TO_IDLE;
        break;
    case ICONTILE_STATE_OPEN:
        if (ui_screen == UI_SCREEN_GAME && game_arm_status[uart_id].nts)
            send_serial_game_msg(uart_id);
        else if (ui_screen == UI_SCREEN_TILE && tile_arm_status[uart_id].nts)
            send_serial_tile_msg(uart_id);
        break;
    case ICONTILE_STATE_SEND_TILE_MSG:
        break;
    }
    arm_disp(uart_id);
}

void rx_done(UArg uart_id) {
    // RX Timeouts are automatically reset here.
    switch (arm_icontile_state) {
    case ICONTILE_STATE_DIS:
        // Should not be reachable.
        break;
    case ICONTILE_STATE_HS0:
        // We take our clock setting from this person if:
        //  1. They are authoritative and we are not, OR
        //  2. Neither of us are authoritative and they are ahead of us.

        if (!my_conf.time_is_set &&
                (arm_rx_buf.current_time_authority ||
                 arm_rx_buf.current_time > my_conf.csecs_of_queercon)) {
            set_clock(arm_rx_buf.current_time);
            my_conf.time_is_set = arm_rx_buf.current_time_authority;
        }

        send_serial_handshake(uart_id, 1); // ack this

        if (((serial_handshake_t*) &arm_rx_buf.payload)->ack) {
            arm_icontile_state = ICONTILE_STATE_HS2;
        } else {
            arm_icontile_state = ICONTILE_STATE_HS1;
        }
        break;
    case ICONTILE_STATE_HS1:
        if (((serial_handshake_t*) &arm_rx_buf.payload)->ack) {
            // They got our ACK. We should be good to go.
            arm_icontile_state = ICONTILE_STATE_OPEN_WAIT;
        } else {
            // They didn't get our ACK and are still sending.
            // Retry.
            send_serial_handshake(uart_id, 1); // ack this
        }
        break;
    case ICONTILE_STATE_HS2:
        // They've acked our handshake. Ack their ack. Lol.
        if (((serial_handshake_t*) &arm_rx_buf.payload)->ack == 1) {
            send_serial_handshake(uart_id, 2);
        } else if (!((serial_handshake_t*) &arm_rx_buf.payload)->ack) {
            send_serial_handshake(uart_id, 2);
        }
        break;
    case ICONTILE_STATE_OPEN_WAIT:
        if (arm_rx_buf.msg_type == SERIAL_MSG_TYPE_HANDSHAKE) {
            send_serial_handshake(uart_id, 2);
            break;
        }
        // If it's not a handshake, fall through. The other side may be
        //  all the way open.
    case ICONTILE_STATE_OPEN:
        if (arm_rx_buf.msg_type == SERIAL_MSG_TYPE_HANDSHAKE) {
            send_serial_handshake(uart_id, 2);
            break;
        }

        if (arm_rx_buf.msg_type == SERIAL_MSG_TYPE_TILE) {
            // This is a topology update.
        } else if (arm_rx_buf.msg_type == SERIAL_MSG_TYPE_GAME) {
            // This is telling us it's time to TRANSFORM.
            if (((serial_game_msg_t*) &arm_rx_buf.payload)->conn_msg &&
                    game_arm_status[uart_id].sufficiency_info == GAME_SUFFICIENT_MSG) {
                game_arm_status[uart_id].sufficiency_info = GAME_SUFFICIENT_ALONE;
                do_icon_transition(game_curr_icon.arms[uart_id].result_icon_id);
            }
        }
        // Process more interesting messages here.
        break;
    }
    arm_disp(uart_id);
}

void serial_arm_task(UArg uart_id, UArg arg1) {
    arm_phy_state=SERIAL_PHY_STATE_DIS;

    // This table holds the correct disconnected values:
    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table);

    do {
        // A "continue" takes us here:
        Task_yield();

        block_until_plugged(uart_id);

        // We are plugged. Now, there's two different things that can happen.
        //  We can either just hang out like this, or we can mutually secure
        //  our UARTs and start chatting.

        switch (arm_phy_state) {
        case SERIAL_PHY_STATE_PLUGGED:
//            outer_arm_color(uart_id, 25, 25, 25);
            // We are just hanging out plugged into each other. Two things
            //  can cause us to want to change that. Either we need to send
            //  or we've gotten a RTS (or disconnect) from our peer.

            if (do_phy_handshake_rx(uart_id) || ((arm_fop || arm_nts || game_arm_status[uart_id].nts) && do_phy_handshake_tx(uart_id))) {
                // If we got here, then we know it was a signal to connect.
                //  Or we just plugged in.
                //  We can assert our side high by switching on our UART.

                PIN_setOutputValue(arm_gpio_pin_handle, arm_gpio_tx, 1);
                PIN_close(arm_gpio_pin_handle);
                uart_h = UART_open(uart_id, &uart_p);

                // We are active and ready to chat.
                arm_phy_state = SERIAL_PHY_STATE_ACTIVE;
                rx_timeouts_to_idle = RX_TIMEOUTS_TO_IDLE;

                // I'm going to pause for a moment here to try to make sure
                //  that both sides are consistent. Maybe this will help
                //  with the shifting problem.
                Task_sleep(100);

                // So let's set up an asynchronous read, and then if we need to,
                //  make a blocking write.
                // Reads do not time out in callback mode.
                UART_read(uart_h, rx_bytes, sizeof(serial_message_t));

                if (arm_fop) {
                    new_plug(uart_id);
                    arm_fop = 0;
                }
            }
            break;
        case SERIAL_PHY_STATE_ACTIVE:
            // If we're here, that means the other side signaled to us, or
            // that we have something to say.

            // If we're here, we're also already listening.
            if (arm_nts) {
                outer_arm_color(uart_id, 15,15,15);
                setup_tx_buf_no_payload(uart_id);
                UART_write(uart_h, tx_bytes, sizeof(serial_message_t));
                // Done sending, so we can cancel this flag:
                arm_nts = SERIAL_MSG_TYPE_NOMSG;
                arm_disp(uart_id);
            }

            if (Semaphore_pend(rx_done_sem, RTS_TIMEOUT*2)) {
                // Look at this...

                if (((UARTCC26XX_Object *) uart_h->object)->status == UART_OK)
                    memcpy(&arm_rx_buf, &rx_bytes, sizeof(serial_message_t));
                else
                    __nop();

                UART_read(uart_h, rx_bytes, sizeof(serial_message_t));

                if (((UARTCC26XX_Object *) uart_h->object)->status == UART_OK &&
                        arm_rx_buf.current_time && rx_valid(uart_id)) { // nobody will ever have 0 time, and sometimes this just gets zeroes.
                    rx_done(uart_id);
                    // Invalidate already received message, just in case.
                    arm_rx_buf.crc++;
                    rx_timeouts_to_idle = RX_TIMEOUTS_TO_IDLE;
                }
            } else {
                rx_timeouts_to_idle--;
                if (!rx_timeouts_to_idle) {
                    if (arm_icontile_state == ICONTILE_STATE_OPEN_WAIT2) {
                        rx_timeouts_to_idle = RX_TIMEOUTS_TO_IDLE;
                        connection_opened(uart_id);
                    }

                    if (game_arm_status[uart_id].nts_done) {
                        game_arm_status[uart_id].nts = 0;
                        game_arm_status[uart_id].nts_done = 0;

                        game_arm_status[uart_id].sufficiency_info = GAME_SUFFICIENT_ALONE;
                        do_icon_transition(game_curr_icon.arms[uart_id].result_icon_id);

                    }

                    arm_phy_state = SERIAL_PHY_STATE_PLUGGED;
                    arm_nts = SERIAL_MSG_TYPE_NOMSG;
                } else {
                    rx_timeout(uart_id); // We didn't get a message during our timeout window. Maybe we will later.
                }
            }
            if (arm_phy_state == SERIAL_PHY_STATE_PLUGGED) {
                // Clean up, we're going inactive on this line:
                UART_readCancel(uart_h);
                UART_close(uart_h);
                arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state,
                                               arm_gpio_init_table);
                Semaphore_post(uart_mutex);
            }

            break;
        }
    } while (1);
}

void serial_init() {
    Semaphore_Params uart_mutex_params;
    Semaphore_Params_init(&uart_mutex_params);
    uart_mutex_params.mode = Semaphore_Mode_BINARY;
    uart_mutex = Semaphore_create(1, &uart_mutex_params, NULL);

    Semaphore_Params rx_done_sem_params;
    Semaphore_Params_init(&rx_done_sem_params);
    rx_done_sem_params.mode = Semaphore_Mode_BINARY;
    rx_done_sem = Semaphore_create(0, &rx_done_sem_params, NULL);

    UART_Params_init(&uart_p);
    // Defaults used:
    // blocking reads and writes, no write timeout, 8N1.
    uart_p.baudRate = 115200;
    uart_p.parityType = UART_PAR_EVEN;
    // No read timeout because it's on a callback.
    // No write timeout because there's no flow control.
    uart_p.readMode = UART_MODE_CALLBACK;
    uart_p.readCallback = uart_rx_done;
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
        game_arm_status[task_arm_id].connectable = 1;
        game_arm_status[task_arm_id].icon_id = 255;
    }
}
