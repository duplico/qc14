/*
 * tlc_driver.h
 *
 *  Created on: Jun 7, 2017
 *      Author: glouthan
 */

#ifndef STARTUP_TLC_DRIVER_H_
#define STARTUP_TLC_DRIVER_H_

void led_init();
void led_start();

extern uint8_t led_buf[11][7][3];

#endif /* STARTUP_TLC_DRIVER_H_ */
