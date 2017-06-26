// Standard library includes
#include <stdio.h>

// TI runtime
#include <xdc/runtime/Error.h>

// SYS/BIOS primitives
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

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

// Application includes
#include "simple_broadcaster.h"

// QC14 Drivers
#include "ExtFlash.h"
#include "tlc_driver.h"
#include "screen.h"
#include "ui.h"

// Configuration includes:
#include "ble_user_config.h"
    // BLE user defined configuration
    bleUserCfg_t user0Cfg = BLE_USER_CFG;
#include "qc14.h"


static PIN_State sw_pin_state;
PIN_Handle sw_pin_h;

PIN_Config sw_pin_table[] = {
    // Rocker switch:
    SW_L            | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    SW_R            | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    SW_CLICK        | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

void init_switch() {
    sw_pin_h = PIN_open(&sw_pin_state, sw_pin_table);
}


//    UART_Handle uh;
//    UART_Params p;
//    UART_Params_init(&p);
//    p.baudRate = 9600;
//
//    uint8_t uart_id = 0;
//    char buf[12] = "test";
//    do {
//
//        sprintf(buf, "%d\r\n", adc_value);
//
//        uh = UART_open(uart_id, &p);
//        UART_write(uh, buf, strlen(buf));
//        UART_close(uh);
//        uart_id = (uart_id+1) % QC14BOARD_UARTCOUNT;
//        Task_sleep(5000);
//    } while (1);

//
//    while (1)
//    {
//        Task_sleep(BIOS_WAIT_FOREVER);
//    }

void init_ble() {
    /* Initialize ICall module */
    ICall_init();
    ICall_createRemoteTasks(); // Priority 5
    GAPRole_createTask(); // Priority 3
    SimpleBLEBroadcaster_createTask(); // Priority 1
}

void init_badge_peripherals() {
    init_ble();
    init_switch();
    led_init();
    screen_init();
    ui_init();
}

int main()
{
    // TI-RTOS driver initializations:
    PIN_init(BoardGpioInitTable);
    PWM_init();
    SPI_init();
    ADC_init();
    UART_init();

    // Keep the external HF oscillator on at all times:
    // TODO: May not be needed if we're always writing SPI from RAM.
    Power_setDependency(PowerCC26XX_XOSC_HF);

    // Initialize badge peripherals:
    init_badge_peripherals();
    // And we're off to see the wizard!
    BIOS_start();

  return 0;
}
