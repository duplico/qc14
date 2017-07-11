/*
 * qc14.h
 *
 *  Created on: Jun 7, 2017
 *      Author: glouthan
 */

#ifndef STARTUP_QC14_H_
#define STARTUP_QC14_H_

#include "board.h"

#define LED_MP_RATE_MIN 16777215
#define LED_MP_RATE_BEST 48000

// TLC LED DRIVER CONFIGURATION
#define LED_MP_RATE LED_MP_RATE_BEST
#define LED_BRIGHTNESS_INTERVAL 12500

// Switch signals
#define SW_SIGNAL_NONE 0
#define SW_SIGNAL_OPEN 0b1000
#define SW_SIGNAL_L 0b01
#define SW_SIGNAL_R 0b10
#define SW_SIGNAL_DIR_MASK 0x11
#define SW_SIGNAL_C 0b100

// Serial timeout configuration
#define RTS_TIMEOUT_MS 50
#define SERIAL_SETTLE_TIME_MS 25
#define PLUG_TIMEOUT_MS 2500
#define IDLE_BACKOFF_MS 1000

// UI constant configuration
#define UI_CLOCK_MS 10
#define UI_SEL_TIMEOUT_SEC 10
#define UI_MAIN_TIMEOUT_SEC 600

// UART Protocol states:
extern uint8_t uart_proto_state[4];

unsigned short crc16(volatile unsigned char *sbuf,unsigned char len);

typedef struct {
    uint16_t badge_id;
    uint8_t badges_mated[36];
    uint8_t current_icon;
    uint8_t icons_unlocked;
    uint8_t icons_been[5];
    uint8_t current_tile;
    uint8_t avail_tiles;
    uint16_t crc;
} qc14_badge_conf_t;

extern qc14_badge_conf_t my_conf;

#endif /* STARTUP_QC14_H_ */
