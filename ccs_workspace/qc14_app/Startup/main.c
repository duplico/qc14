// Standard library includes
#include <stdio.h>
#include <string.h>

// TI runtime
#include <xdc/runtime/Error.h>

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

void sw_clock_f(UArg a0) {
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

#define PROTO_STATE_DIS 0
#define PROTO_STATE_IDLE 1
#define PROTO_STATE_RTS_WAIT 2
#define PROTO_STATE_CTS_WAIT 3
#define PROTO_STATE_SERIAL_PLACEHOLDER 4

#define RTS_TIMEOUT 50000

UART_Handle uart_h;
UART_Params uart_p;
Semaphore_Handle uart_mutex;

Task_Struct uart_arm_tasks[4];
char uart_arm_task_stacks[4][100];

uint32_t uart_rts_timeout[4] = {0, 0, 0, 0};
uint8_t uart_rts_old_val[4] = {0, 0, 0, 0};
uint8_t uart_rts_cur_val[4] = {0, 0, 0, 0};
uint8_t uart_proto_state[4] = {0, 0, 0, 0};
uint8_t uart_hold_sem[4] = {0, 0, 0, 0};

PIN_Config arm_gpio_init_tables[4][3] = {
    {
         P1_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
         P1_RX           | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_DIS,
         PIN_TERMINATE
    }, {
        P2_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P2_RX           | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }, {
        P3_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P3_RX           | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }, {
        P4_TX           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        P4_RX           | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_DIS,
        PIN_TERMINATE
    }
};
PIN_State arm_gpio_pin_states[4];
PIN_Handle arm_gpio_pin_handles[4];


uint64_t arm_gpio_txs[4] = {P1_TX, P2_TX, P3_TX, P4_TX};
uint64_t arm_gpio_rxs[4] = {P1_RX, P2_RX, P3_RX, P4_RX};

#define arm_timeout uart_rts_timeout[uart_id]
#define arm_old_val uart_rts_old_val[uart_id]
#define arm_cur_val uart_rts_cur_val[uart_id]
#define arm_proto_state uart_proto_state[uart_id]
#define arm_gpio_init_table arm_gpio_init_tables[uart_id]
#define arm_gpio_pin_state arm_gpio_pin_states[uart_id]
#define arm_gpio_pin_handle arm_gpio_pin_handles[uart_id]
#define arm_gpio_tx arm_gpio_txs[uart_id]
#define arm_gpio_rx arm_gpio_rxs[uart_id]
#define arm_i_have_uart uart_hold_sem[uart_id]

uint8_t arm_read_in_debounced(uint8_t uart_id) {
    uint8_t read_val = PINCC26XX_getInputValue(arm_gpio_rx);

    if (arm_old_val == read_val)
        arm_cur_val = read_val;
    arm_old_val = read_val;

    return arm_cur_val;
}

void serial_do_stuff(uint8_t uart_id) {
    char buf[12] = "test";

    // Tear down my GPIO and prepare to switch to the peripheral:
    PIN_close(arm_gpio_pin_handle);

    // Secure control of the global UART:
    Semaphore_pend(uart_mutex, BIOS_WAIT_FOREVER);
    uart_h = UART_open(uart_id, &uart_p);
    UART_write(uart_h, buf, strlen(buf));
    // Release the global UART:
    UART_close(uart_h);
    Semaphore_post(uart_mutex);
    // Reallocate our pins.
    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct *disconnected* values.
}

// All four arms share the same function, even though they have separate tasks.
void serial_arm_task(UArg uart_id, UArg arg1) {
    arm_gpio_pin_handle = PIN_open(&arm_gpio_pin_state, arm_gpio_init_table); // This table holds the correct disconnected values.
    do {
        // TODO: Perhaps implement this using EVENTS instead of busy-waiting.
        // Possible "events":
        //  IN Low (debounced)
        //  IN High (debounced)

        switch(arm_proto_state) {
        case PROTO_STATE_DIS:
            // input has pullup, output low.
            // can react to IN going low.
            if (!arm_read_in_debounced(uart_id)) {
                arm_proto_state = PROTO_STATE_IDLE;
            }
            break;
        case PROTO_STATE_IDLE:
            // input has pullup, output low.
            // can react to IN going high.
            if (arm_read_in_debounced(uart_id)) {
                // debounced input has gone high.
                // this is either a disconnect or a RTS signal.
                // Either way, we wait until we have control of our UART, and
                // attempt to respond.
                arm_proto_state = PROTO_STATE_RTS_WAIT;
                arm_timeout = Clock_getTicks() + RTS_TIMEOUT; // TODO: guard for overflows.
                // fall through...
            } else {
                break;
            }
        case PROTO_STATE_RTS_WAIT:
            if (Semaphore_pend(uart_mutex, RTS_TIMEOUT)) {
                // we have the UART.
                // we respond to an RTS signal by taking our output high, too.
                arm_i_have_uart = 1;
                // We are clear to send. Signal CTS.
                PINCC26XX_setOutputValue(arm_gpio_tx, 1);
                arm_proto_state = PROTO_STATE_CTS_WAIT;
            } else {
                // We didn't get the UART in time.
                // TODO: Do we even need to *have* a timeout here?
                //       I'm not convinced we do.
                // arm_proto_state = PROTO_STATE_DIS;
            }
            break;
        case PROTO_STATE_CTS_WAIT:
            // in is high, out set high.
            // can react to IN going low (do serial stuff)

            if (!arm_read_in_debounced(uart_id)) {
                // Our CTS has been acknowledged.
            }
            // can react to TIMEOUT.
            if (Clock_getTicks() > arm_timeout) { // TODO: system clock ticks wrap every 11-12 hours or so.
                // DISCONNECTED!!!
                arm_proto_state = PROTO_STATE_DIS;
                PINCC26XX_setOutputValue(arm_gpio_tx, 0); // Bring output low.
            }

        }

        Task_sleep(100); // 1000 us
    } while (1);
}

void serial_init() {
    Semaphore_Params uart_mutex_params;
    Semaphore_Params_init(&uart_mutex_params);

    uart_mutex_params.mode = Semaphore_Mode_BINARY;
    uart_mutex = Semaphore_create(1, &uart_mutex_params, NULL);

    UART_Params_init(&uart_p);
    uart_p.baudRate = 9600; // TODO: SPEEEEEED

    // Create the 4 tasks:
    for (uint8_t task_arm_id=0; task_arm_id<4; task_arm_id++) {
        Task_Params taskParams;
        Task_Params_init(&taskParams);
        taskParams.stack = uart_arm_task_stacks[task_arm_id];
        taskParams.stackSize = sizeof(uart_arm_task_stacks[0]);
        taskParams.priority = 1;
        taskParams.arg0 = task_arm_id;
        Task_construct(&uart_arm_tasks[task_arm_id], serial_arm_task, &taskParams, NULL);
    }
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
    led_init();
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
