/*
 * serial.h
 *
 *  Created on: Jul 8, 2017
 *      Author: George
 */

#ifndef STARTUP_SERIAL_H_
#define STARTUP_SERIAL_H_

#include "qc14.h"

#define SERIAL_PHY_STATE_DIS 0
#define SERIAL_PHY_STATE_PLUGGED 1
#define SERIAL_PHY_STATE_ACTIVE 2

#define ICONTILE_STATE_DIS 0
#define ICONTILE_STATE_HS0 1
#define ICONTILE_STATE_HS1 2
#define ICONTILE_STATE_HS2 3
#define ICONTILE_STATE_OPEN_WAIT 4
#define ICONTILE_STATE_OPEN_WAIT2 5
#define ICONTILE_STATE_OPEN 6
#define ICONTILE_STATE_SEND_GAME_MSG 7
#define ICONTILE_STATE_SEND_TILE_MSG 7

#define SERIAL_MSG_TYPE_NOMSG 0
#define SERIAL_MSG_TYPE_HANDSHAKE 1
#define SERIAL_MSG_TYPE_GAME 2
#define SERIAL_MSG_TYPE_TILE 3
#define SERIAL_MSG_TYPE_CONF 4
#define SERIAL_MSG_TYPE_MAX 4

#define GAME_SUFFICIENT_ALONE 3
#define GAME_SUFFICIENT_CONN 2
#define GAME_SUFFICIENT_MSG 1

void serial_init();
uint8_t serial_in_progress();

typedef struct {
    uint16_t badge_id;
    uint8_t msg_type;
    uint32_t current_time;
    uint16_t current_time_authority;
    uint8_t arm_id;
    uint8_t payload[52];
    uint32_t crc; // word align this bitch.
} serial_message_t;

typedef struct {
    char handle[9];
    uint8_t badges_mated[BADGES_MATED_BYTES];
    uint8_t current_mode;
    uint8_t current_icon_or_tile_id;
    uint8_t ack;
    uint8_t in_fabric;
    int8_t fabric_offset;
    uint8_t fully_connected;
    uint8_t msg_ready; // also encodes result.
} serial_handshake_t; // Initialization handshake

typedef struct {
    uint8_t conn_msg;
    uint8_t conn_result;
    uint8_t pad[50];
} serial_game_msg_t; // Game update

typedef struct {
    uint8_t fully_connected;
} serial_tile_msg; // Tile topology/info

typedef struct {
    uint8_t update_handle;
    char new_handle[9];
} serial_conf_msg; // INBOUND FROM CONSOLE

#endif /* STARTUP_SERIAL_H_ */
