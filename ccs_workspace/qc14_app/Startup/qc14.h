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
#define SW_SIGNAL_DIR_MASK 0b11
#define SW_SIGNAL_C 0b100
#define SW_SIGNAL_TIMEOUT 0b10000

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

#define FLASH_LOC_RESERVED          0x000000
#define FLASH_ID_LOC                0x001000
#define FLASH_CONF_LOC              0x002000
#define FLASH_BOOT_ANIM_LOC         0x003000
#define FLASH_TILE_ANIM_LOC         0x004000
#define FLASH_CONF_BACKUP_LOC       0x008000
#define FLASH_ID_LOC2               0x009000
#define FLASH_GAME_ANIM_LOC         0x00a000
#define FLASH_SCREEN_FRAMES_STARTPT 0x010000

#define BADGES_MATED_BYTES 36

#define ICON_COFFEE_ID 42
#define ICON_COUNT 44
#define TILE_COUNT 12

// 285 badges, plus #285 (the fallback ID)
// I am #5 because reasons.
// There are 14 ubers and 11 handlers, with 7 overlapping. So:
// 0..4   not me (qty 5)
// 5      me     (qty 1)
// 6      not me (qty 1)
// 7..13  uber handlers (qty 7)
// 14..17 handlers (qty 4)
// 18..27 sponsors (qty 10)

#define BADGES_IN_SYSTEM 286
#define BADGE_ID_DUPLICO 5
#define BADGE_UBER_CUTOFF 14
#define BADGE_HANDLER_START 4
#define BADGE_HANDLER_CUTOFF 18
#define BADGE_SPONSOR_START 18
#define BADGE_SPONSOR_CUTOFF 28

unsigned short crc16(volatile unsigned char *sbuf,unsigned char len);

typedef struct {
    uint16_t badge_id;
    uint8_t badges_mated[BADGES_MATED_BYTES];
    uint8_t icons_been[6];
    char handle[10];
    uint8_t current_icon;
    uint8_t icons_unlocked;
    uint8_t current_tile;
    uint8_t time_is_set;
    uint16_t avail_tiles;
    uint32_t csecs_of_queercon;
    uint16_t crc;
} qc14_badge_conf_t;

extern qc14_badge_conf_t my_conf;

void start_badge();
void qc14conf_save();
void set_badge_mated(uint16_t badge_id);
uint8_t game_been_icon(uint8_t icon_id);
void game_set_icon(uint8_t icon_id);
uint8_t game_starting_icon();
uint8_t is_uber(uint16_t id);
uint8_t is_handler(uint16_t id);
uint8_t is_sponsor(uint16_t id);

#endif /* STARTUP_QC14_H_ */
