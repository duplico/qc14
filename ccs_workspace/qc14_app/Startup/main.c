// Standard library includes
#include <stdio.h>
#include <string.h>

// TI runtime

// SYS/BIOS primitives
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

// Instruction fetch cache dependencies:
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

// TI-RTOS drivers
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/PWM.h>

// TI-BLE Stack
#include "icall.h"
#include "hal_assert.h"
#include "bcomdef.h"
#include "broadcaster.h"
#include "ble_user_config.h"
bleUserCfg_t user0Cfg = BLE_USER_CFG; // BLE user defined configuration

// Application includes
#include "simple_broadcaster.h"

// QC14 Drivers
#include "ExtFlash.h"
#include "tlc_driver.h"
#include "ui.h"
#include "serial.h"
#include "qc14.h"

qc14_badge_conf_t my_conf;

unsigned short crc16(volatile unsigned char *sbuf,unsigned char len){
    unsigned short crc=0xB8F6;

    while(len){
        crc=(unsigned char)(crc >> 8) | (crc << 8);
        crc^=(unsigned char) *sbuf;
        crc^=(unsigned char)(crc & 0xff) >> 4;
        crc^=(crc << 8) << 4;
        crc^=((crc & 0xff) << 4) << 1;
        len--;
        sbuf++;
    }
    return crc;
}//crc16()

void init_ble() {
    // Initialize the BLE stack Indirect Call (ICall) module
    ICall_init();
    ICall_createRemoteTasks(); // (Priority 5)
    GAPRole_createTask(); // (Priority 3)
    SimpleBLEBroadcaster_createTask(); // (Priority 1)
}

void init_badge_peripherals() {
    led_init();
    screen_init();
}

void qc14conf_save() {
    qc14_badge_conf_t readback_conf;
    uint8_t curr_time_set_flag = my_conf.time_is_set;
    my_conf.time_is_set = 0;

    // Compute the CRC to save.
    my_conf.crc = crc16((uint8_t*) &my_conf, sizeof(qc14_badge_conf_t)-2);

    // Wait until the flash chip is available to access.
    Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
    ExtFlash_open();

    // Write the main conf to flash, then read it back. If it didn't work,
    //  keep trying.
    do {
        ExtFlash_erase(FLASH_CONF_LOC, sizeof(qc14_badge_conf_t));
        ExtFlash_write_skipodd(FLASH_CONF_LOC,
                              sizeof(qc14_badge_conf_t),
                              (uint8_t *) &my_conf);
        ExtFlash_read_skipodd(FLASH_CONF_LOC,
                              sizeof(qc14_badge_conf_t),
                              (uint8_t *) &readback_conf);
    } while (memcmp(&my_conf, &readback_conf, sizeof(qc14_badge_conf_t)));

    // Now write the backup conf to flash, then read it back. If it didn't
    //  work, keep trying.
    do {
        ExtFlash_erase(FLASH_CONF_BACKUP_LOC, sizeof(qc14_badge_conf_t));
        ExtFlash_write_skipodd(FLASH_CONF_BACKUP_LOC,
                               sizeof(qc14_badge_conf_t),
                               (uint8_t *) &my_conf);
        ExtFlash_read_skipodd(FLASH_CONF_BACKUP_LOC,
                              sizeof(qc14_badge_conf_t),
                              (uint8_t *) &readback_conf);
    } while (memcmp(&my_conf, &readback_conf, sizeof(qc14_badge_conf_t)));

    ExtFlash_close();
    Semaphore_post(flash_sem);

    my_conf.time_is_set = curr_time_set_flag;
}

uint8_t game_starting_icon() {
    return my_conf.badge_id % 4; // TODO: Set to the real one.
}

uint8_t game_been_icon(uint8_t icon_id) {
    uint8_t byte_number = icon_id / 8;
    uint8_t bit_number = icon_id % 8;
    return my_conf.icons_been[byte_number] & (1 << bit_number);
}

void game_set_icon(uint8_t icon_id) {
    if (game_been_icon(icon_id))
        return;
    uint8_t byte_number = icon_id / 8;
    uint8_t bit_number = icon_id % 8;
    my_conf.icons_been[byte_number] |= (1 << bit_number);
    my_conf.current_icon = icon_id;
    qc14conf_save();
}

uint8_t has_badge_mated(uint16_t badge_id) {
    uint8_t byte_number = badge_id / 8;
    uint8_t bit_number = badge_id % 8;
    return (my_conf.badges_mated[byte_number] & (1 << bit_number))? 1:0;
}

void set_badge_mated(uint16_t badge_id) {
    if (has_badge_mated(badge_id))
        return;

    uint8_t byte_number = badge_id / 8;
    uint8_t bit_number = badge_id % 8;
    my_conf.badges_mated[byte_number] |= (1 << bit_number);
    qc14conf_save();
}

void qc14conf_init() {
    Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
    ExtFlash_open();
    // Load up the animation from base and index.
    ExtFlash_read_skipodd(FLASH_CONF_LOC,
                          sizeof(qc14_badge_conf_t),
                          (uint8_t *) &my_conf);
    ExtFlash_close();
    Semaphore_post(flash_sem);

    if (crc16((uint8_t*) &my_conf, sizeof(qc14_badge_conf_t)-2) != my_conf.crc) {
        // Invalid CRC. Check backup:

        Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
        ExtFlash_open();
        ExtFlash_read_skipodd(FLASH_CONF_BACKUP_LOC,
                              sizeof(qc14_badge_conf_t),
                              (uint8_t *) &my_conf);
        ExtFlash_close();
        Semaphore_post(flash_sem);

        if (crc16((uint8_t*) &my_conf,
                  sizeof(qc14_badge_conf_t)-2) != my_conf.crc) {
            // Backup also invalid:

            // Check ID locations:
            uint16_t badge_id1 = 0;
            uint16_t badge_id2 = 0;
            Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
            ExtFlash_open();
            ExtFlash_read_skipodd(FLASH_ID_LOC, 2, (uint8_t *) &badge_id1);
            ExtFlash_read_skipodd(FLASH_ID_LOC2, 2, (uint8_t *) &badge_id2);
            ExtFlash_close();
            Semaphore_post(flash_sem);

            if (badge_id1 != badge_id2 || badge_id1 == 0xffff) {
                // PROBLEM. We'll have to make one up.
                badge_id1 = 285; // 285 is the backup number.
            }

            // Zero it out and start from scratch:
            memset((uint8_t *) &my_conf, 0x00, sizeof(qc14_badge_conf_t));

            my_conf.badge_id = badge_id1;
            game_set_icon(game_starting_icon());
            set_badge_mated(my_conf.badge_id);
            // The two function calls above will set the CRC and write to flash.
        }
    }

    // Config is loaded or created.
    if (my_conf.badge_id == BADGE_ID_DUPLICO &&
            my_conf.csecs_of_queercon < 180000) {
        // If it's MEEEEEE
        //  and I've had less than 30 minutes of uptime:
        my_conf.time_is_set = 1;
    }

}

// Called only once, from the screen.
void start_badge() {
    qc14conf_init();
    ui_init();
    serial_init();
    init_ble();
}

int main()
{
    // TI-RTOS driver initializations:
    PIN_init(BoardGpioInitTable);
    PWM_init();
    SPI_init();
    ADC_init();
    UART_init();

    init_badge_peripherals();

    // Signal that we need the external HF oscillator.
    Power_setDependency(PowerCC26XX_XOSC_HF);

    // And we're off to see the wizard!
    BIOS_start();

    return 0;
}
