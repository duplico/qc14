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

extern uint8 scanRspData[22];
extern uint8 advertData[23];

qc14_badge_conf_t my_conf;

ICall_Semaphore ble_sem;
volatile uint8_t update_ble = 0;

uint32_t tile_offsets[TILE_COUNT] = {
                         202-28, // rainbow
                         146, // bi
                         103-32, // trans // len is 103 (0 to 102)
                         33, // bear
                         144, // leather (was 148)
                         0, // cubeheart
                         8, // rainbowboom (18 to sync)
                         3 // heartonblack
};

int8_t tile_frame_periods[TILE_COUNT][4] = {
                                  // UP, LEFT, DOWN, RIGHT
                                  {0,-1,0,1}, // rainbow // revved
                                  {-1,-1,1,1}, // bi
                                  {0,-1,0,1}, // trans // revved
                                  {-1,2,-3,4}, // bear
                                  {-1,-1,1,1}, // leather // correct
                                  {0,0,0,0}, // cubeheart
                                  {1,1,1,1}, // rainbowboom
                                  {-1,-1,-1,-1} // heartonblack
};

unsigned short crc16(volatile unsigned char *sbuf,unsigned char len) {
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

// Callable from a TASK context:
void qc14conf_save() {
    qc14_badge_conf_t readback_conf;
    qc14_badge_conf_t save_conf;

    // We're making a copy here, because my_conf can actually change
    //  due to the time thing.
    memcpy(&save_conf, &my_conf, sizeof(qc14_badge_conf_t));

    save_conf.time_is_set = 0;

    // Compute the CRC to save.
    save_conf.crc = crc16((uint8_t*) &save_conf, sizeof(qc14_badge_conf_t)-4);

    // Wait until the flash chip is available to access.
    Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
    while (!ExtFlash_open());

    // Write the main conf to flash, then read it back. If it didn't work,
    //  keep trying.
    do {
        ExtFlash_erase(FLASH_CONF_LOC, sizeof(qc14_badge_conf_t));
        ExtFlash_write(FLASH_CONF_LOC,
                       sizeof(qc14_badge_conf_t),
                       (uint8_t *) &save_conf);
        ExtFlash_read(FLASH_CONF_LOC,
                      sizeof(qc14_badge_conf_t),
                      (uint8_t *) &readback_conf);
    } while (memcmp(&save_conf, &readback_conf, sizeof(qc14_badge_conf_t)));

    // Now write the backup conf to flash, then read it back. If it didn't
    //  work, keep trying.
    do {
        ExtFlash_erase(FLASH_CONF_BACKUP_LOC, sizeof(qc14_badge_conf_t));
        ExtFlash_write(FLASH_CONF_BACKUP_LOC,
                               sizeof(qc14_badge_conf_t),
                               (uint8_t *) &save_conf);
        ExtFlash_read(FLASH_CONF_BACKUP_LOC,
                              sizeof(qc14_badge_conf_t),
                              (uint8_t *) &readback_conf);
    } while (memcmp(&save_conf, &readback_conf, sizeof(qc14_badge_conf_t)));

    ExtFlash_close();
    Semaphore_post(flash_sem);
    update_ble = 1;
    sprintf((char *) &advertData[15], "%d", my_conf.badge_id);
    ICall_signal(ble_sem);
}

uint8_t game_starting_icon(uint16_t badge_id) {
    return 16 + (badge_id % 4);
}

uint8_t is_uber(uint16_t id) {
    return my_conf.badge_id < BADGE_UBER_CUTOFF;
}

uint8_t is_handler(uint16_t id) {
    return my_conf.badge_id >= BADGE_HANDLER_START &&
            my_conf.badge_id < BADGE_HANDLER_CUTOFF;
}

uint8_t is_sponsor(uint16_t id) {
    return my_conf.badge_id >= BADGE_SPONSOR_START &&
            my_conf.badge_id < BADGE_SPONSOR_CUTOFF;
}

uint8_t game_been_icon(uint8_t icon_id) {
    if (icon_id >= ICON_COUNT)
        return 0;
    uint8_t byte_number = ICONS_BEEN_BYTES - 1 - icon_id / 8;
    uint8_t bit_number = icon_id % 8;
    return (my_conf.icons_been[byte_number] & (1 << bit_number)) ? 1 : 0;
}

void set_radio_crc() {
    uint16_t c = crc16(&scanRspData[4], 9);
    scanRspData[14] = c & 0xff;
    scanRspData[13] = (c >> 8) & 0xff;
}

void game_set_icon(uint8_t icon_id) {
    if (icon_id >= ICON_COUNT)
        return;
    if (icon_id != game_starting_icon(my_conf.badge_id) &&
            icon_id != ICON_COFFEE)
        my_conf.earned_icon = icon_id;
    my_conf.current_icon = icon_id;
    scanRspData[6] = icon_id;
    if (!game_been_icon(icon_id)) {
        uint8_t byte_number = ICONS_BEEN_BYTES - 1 - icon_id / 8;
        uint8_t bit_number = icon_id % 8;
        my_conf.icons_been[byte_number] |= (1 << bit_number);
    }
    memcpy(&scanRspData[7], my_conf.icons_been, 6);
    set_radio_crc();
    Semaphore_post(save_sem);
}

void unlock_tile(uint8_t index) {
    if (index > TILE_COUNT)
        return;
    my_conf.avail_tiles |= (1 << index);
    Semaphore_post(save_sem);
}

uint8_t has_badge_mated(uint16_t badge_id) {
    if (badge_id > BADGES_IN_SYSTEM)
        return 0;
    uint8_t byte_number = badge_id / 8;
    uint8_t bit_number = badge_id % 8;
    return (my_conf.badges_mated[byte_number] & (1 << bit_number))? 1:0;
}

void set_badge_mated(uint16_t badge_id) {
    if (has_badge_mated(badge_id))
        return;
    if (badge_id > BADGES_IN_SYSTEM)
        return;

    uint8_t byte_number = badge_id / 8;
    uint8_t bit_number = badge_id % 8;
    my_conf.badges_mated[byte_number] |= (1 << bit_number);
    Semaphore_post(save_sem);
}

void set_clock(uint32_t csecs) {
    uint8_t restore = Clock_isActive(csecs_clock_h);
    if (restore)
        Clock_stop(csecs_clock_h);

    my_conf.csecs_of_queercon = csecs;
    if (my_conf.csecs_of_queercon >= POOL_TILE_TIME) {
        unlock_tile(TILE_RAINBOWBOOM);
    } else if (my_conf.csecs_of_queercon >= CLUB_TILE_TIME) {
        unlock_tile(TILE_HEARTONBLACK);
    } else if (my_conf.csecs_of_queercon >= UNLOCK_TIME) {
        my_conf.icons_unlocked = 1;
        Semaphore_post(save_sem);
    }

    if (restore)
        Clock_start(csecs_clock_h);
}

void qc14conf_init() {
    Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
    while (!ExtFlash_open());
    // Load up the animation from base and index.

    qc14_badge_conf_t load_conf;

    ExtFlash_read(FLASH_CONF_LOC,
                          sizeof(qc14_badge_conf_t),
                          (uint8_t *) &load_conf);
    ExtFlash_close();
    Semaphore_post(flash_sem);

    volatile uint16_t load_crc = crc16((uint8_t*) &load_conf, sizeof(qc14_badge_conf_t)-4);

    // TODO
    if (1 || load_crc != load_conf.crc) {
        // Invalid CRC. Check backup:

        Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
        while (!ExtFlash_open());
        ExtFlash_read(FLASH_CONF_BACKUP_LOC,
                              sizeof(qc14_badge_conf_t),
                              (uint8_t *) &load_conf);
        ExtFlash_close();
        Semaphore_post(flash_sem);

        if (crc16((uint8_t*) &load_conf,
                  sizeof(qc14_badge_conf_t)-2) != load_conf.crc) {
            // Backup also invalid:

            // Check ID locations:
            uint16_t badge_id1 = 0;
            uint16_t badge_id2 = 0;
            Semaphore_pend(flash_sem, BIOS_WAIT_FOREVER);
            while (!ExtFlash_open());
            ExtFlash_read(FLASH_ID_LOC, 2, (uint8_t *) &badge_id1);
            ExtFlash_read(FLASH_ID_LOC2, 2, (uint8_t *) &badge_id2);
            ExtFlash_close();
            Semaphore_post(flash_sem);

            if (badge_id1 != badge_id2 || badge_id1 == 0xffff) {
                // PROBLEM. We'll have to make one up.
                badge_id1 = 285; // 285 is the backup number.
            }

            // Zero it out and start from scratch:
            memset((uint8_t *) &load_conf, 0x00, sizeof(qc14_badge_conf_t));

            load_conf.badge_id = badge_id1;
            load_conf.avail_tiles = 0x001f;
            load_conf.earned_icon = game_starting_icon(load_conf.badge_id);
            load_conf.current_icon = game_starting_icon(load_conf.badge_id);
            sprintf(load_conf.handle, "%d", load_conf.badge_id);

            set_badge_mated(load_conf.badge_id);
            if (load_conf.badge_id == BADGE_ID_DUPLICO)
                load_conf.csecs_of_queercon = START_TIME_GEORGE;
            else if (is_uber(load_conf.badge_id))
                load_conf.csecs_of_queercon = START_TIME_UBER;
            else
                load_conf.csecs_of_queercon = START_TIME_OTHERS;

            // The two function calls above will set the CRC and signal a
            //  write to the flash.
        }
    }

    memcpy(&my_conf, &load_conf, sizeof(qc14_badge_conf_t));

    // Config is loaded or created.
    set_clock(my_conf.csecs_of_queercon); // Make SURE we get the unlocks done.


    scanRspData[5] = my_conf.badge_id & 0xff;
    scanRspData[4] = (my_conf.badge_id >> 8) & 0xff;

    game_set_icon(my_conf.current_icon);

    if (my_conf.badge_id == BADGE_ID_DUPLICO &&
            my_conf.csecs_of_queercon < START_TIME_GEORGE + 3*CSECS_PER_HOUR) {
        // If it's MEEEEEE
        //  and I've had less than 3 hours of uptime:
        //  (enough time to have infected others with the time virus)
        my_conf.time_is_set = 1;
    }
}

// Called only once, from the screen.
void start_badge() {
    qc14conf_init();
    ui_init();
    serial_init();
    init_ble();
    Clock_start(csecs_clock_h);
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
