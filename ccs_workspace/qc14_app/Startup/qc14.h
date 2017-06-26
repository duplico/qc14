/*
 * qc14.h
 *
 *  Created on: Jun 7, 2017
 *      Author: glouthan
 */

#ifndef STARTUP_QC14_H_
#define STARTUP_QC14_H_

#define LED_MP_RATE_MIN 16777215
#define LED_MP_RATE_BEST 48000

// TLC LED DRIVER CONFIGURATION
#define LED_MP_RATE LED_MP_RATE_BEST
#define LED_BRIGHTNESS_INTERVAL 12500

// Switch signals
#define SW_SIGNAL_OPEN 0
#define SW_SIGNAL_L 0b1
#define SW_SIGNAL_R 0b10
#define SW_SIGNAL_DIR_MASK 0x11
#define SW_SIGNAL_C 0b100

#endif /* STARTUP_QC14_H_ */
