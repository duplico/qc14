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

uint8_t sw_l_clicked = 0;
uint8_t sw_r_clicked = 0;
uint8_t sw_c_clicked = 0;

void sw_clock_f() {
    static uint8_t sw_l_last = 1;
    static uint8_t sw_r_last = 1;
    static uint8_t sw_c_last = 1;

    uint8_t sw_l_curr = PIN_getInputValue(SW_L);
    uint8_t sw_r_curr = PIN_getInputValue(SW_R);
    uint8_t sw_c_curr = PIN_getInputValue(SW_CLICK);

    if (!sw_l_curr && !sw_l_last && !sw_l_clicked) {
        // left clicked
        sw_l_clicked = 1;
        // do stuff
    } else if (sw_l_curr && sw_l_last && sw_l_clicked) {
        sw_l_clicked = 0;
        // unclicked.
    }
    sw_l_last = sw_l_curr;

    if (!sw_r_curr && !sw_r_last && !sw_r_clicked) {
        // left clicked
        sw_r_clicked = 1;
        // do stuff
    } else if (sw_r_curr && sw_r_last && sw_r_clicked) {
        sw_r_clicked = 0;
        // unclicked.
    }
    sw_r_last = sw_r_curr;

    if (!sw_c_curr && !sw_c_last && !sw_c_clicked) {
        // left clicked
        sw_c_clicked = 1;
        // do stuff
    } else if (sw_c_curr && sw_c_last && sw_c_clicked) {
        sw_c_clicked = 0;
        // unclicked.
    }
    sw_c_last = sw_c_curr;
}

Clock_Handle sw_clock;

void init_switch() {
    sw_pin_h = PIN_open(&sw_pin_state, sw_pin_table);

    Clock_Params clockParams;
    Error_Block eb;
    Error_init(&eb);
    Clock_Params_init(&clockParams);
    clockParams.period = 100;
    clockParams.startFlag = TRUE;
    sw_clock = Clock_create(sw_clock_f, 2, &clockParams, &eb);
}

Task_Struct qcTask;
char qcTaskStack[2000];

void qc_task_fn(UArg a0, UArg a1)
{
    ADC_Handle adc;
    ADC_Params adcp;
    ADC_Params_init(&adcp);
    adc = ADC_open(QC14BOARD_ADC7_LIGHT, &adcp);

    UART_Handle uh;
    UART_Params p;
    UART_Params_init(&p);
    p.baudRate = 9600;

    uint8_t uart_id = 0;
    char buf[12] = "test";
    uint_fast16_t adc_value = 0;
    do {
        ADC_convert(adc, &adc_value);

        sprintf(buf, "%d\r\n", adc_value);

        uh = UART_open(uart_id, &p);
        UART_write(uh, buf, strlen(buf));
        UART_close(uh);
        uart_id = (uart_id+1) % QC14BOARD_UARTCOUNT;
        Task_sleep(5000);
    } while (1);

//
//    while (1)
//    {
//        Task_sleep(BIOS_WAIT_FOREVER);
//    }
}

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
    led_init_from_task();
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

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = qcTaskStack;
    taskParams.stackSize = sizeof(qcTaskStack);
    taskParams.priority = 1;
    Task_construct(&qcTask, qc_task_fn, &taskParams, NULL);

    // And we're off to see the wizard!
    BIOS_start();

  return 0;
}
