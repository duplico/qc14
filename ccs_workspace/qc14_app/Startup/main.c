// Standard library includes
#include <stdio.h>

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
#include "screen.h"
#include "ui.h"
#include "serial.h"
#include "qc14.h"

void init_ble() {
    // Initialize the BLE stack Indirect Call (ICall) module
    ICall_init();
    ICall_createRemoteTasks(); // (Priority 5)
    GAPRole_createTask(); // (Priority 3)
    SimpleBLEBroadcaster_createTask(); // (Priority 1)
}

void init_badge_peripherals() {
    init_ble();
    led_init();
    screen_init();
    ui_init();
    serial_init();
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
