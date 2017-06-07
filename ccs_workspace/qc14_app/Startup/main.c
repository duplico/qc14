#include <xdc/runtime/Error.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/pwm/PWMTimerCC26XX.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/ADC.h>

#include <stdio.h>

#include <ti/sysbios/BIOS.h>

#include "icall.h"
#include "hal_assert.h"
#include "bcomdef.h"
#include "broadcaster.h"
#include "simple_broadcaster.h"

#include "ExtFlash.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
bleUserCfg_t user0Cfg = BLE_USER_CFG;

#endif // USE_DEFAULT_USER_CFG

#include <ti/mw/display/Display.h>

// 45-50k best so far.
#define LED_MP_RATE 45000

extern Display_Handle dispHandle;

static PIN_State sw_pin_state;
PIN_Handle sw_pin_h;

PIN_Config sw_pin_table[] = {
    // Rocker switch:
    SW_L            | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    SW_R            | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    SW_CLICK        | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

static PIN_State mbi_pin_state;
PIN_Handle mbi_pin_h;

PIN_Config mbi_pin_table[] = {
    // LED controller:
//    LED_GSCLK       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED_LE         | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    // LED Multiplexer (bit-banged)
    MP_CTR_CLK      | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    MP0_OUT         | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    MP0_CLR         | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    MP1_CLR         | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    MP1_OUT         | PIN_INPUT_EN | PIN_NOPULL,
    PIN_TERMINATE
};

SPI_Handle tlc_spi;

#define TLC_THISISGS    0x00
#define TLC_THISISFUN   0x01

uint8_t fun_base[] = {
        TLC_THISISFUN, // This is a command.
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // ...reserved...
        // B135 / PSM(D1)       0
        // B134 / PSM(D0)       0
        // B133 / OLDENA        1
        // B132 / IDMCUR(D1)    0
        // B131 / IDMCUR(D0)    0
        // B130 / IDMRPT(D0)    0
        // B129 / IDMENA        0
        // B128 / LATTMG(D1)    1:
        0b00100001,
        // B127 / LATTMG(D0)    1
        // B126 / LSDVLT(D1)    0
        // B125 / LSDVLT(D0)    0
        // B124 / LODVLT(D1)    0
        // B123 / LODVLT(D0)    0
        // B122 / ESPWM         1
        // B121 / TMGRST        1
        // B120 / DSPRPT        1:
        0b10000111,
        // B119 / BLANK
        // and 7 bits of global brightness correction:
//        0xff, // blank on.
        0x00, // after this there are 16 7-tets of dot correction. // blank off
        //We'll send it as 14 octets.
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

uint8_t rx_buf[33];

uint8_t all_on[] = {
        TLC_THISISGS, // This is a command.
        0x00, 0x00,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
};

uint8_t tlc_gs_buf[] = {
        TLC_THISISGS, // This is a command.
        0x00, 0x00,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
        0xff, 0xff,
};

volatile uint16_t scan_line = 0;
volatile uint16_t gclk = 0;
Clock_Handle gclk_clock;
Clock_Handle sw_clock;

Semaphore_Handle led_tick_sem;

#define LED_MP_CYCLES 64

void init_ble() {
    /* Initialize ICall module */
    ICall_init();

    /* Start tasks of external images - Priority 5 */
    ICall_createRemoteTasks();

    /* Kick off profile - Priority 3 */
    GAPRole_createTask();

    /* Kick off application - Priority 1 */
    SimpleBLEBroadcaster_createTask();

}

inline void mp_shift() {
    scan_line++;
    scan_line %= 15;
    PIN_setOutputValue(mbi_pin_h, MP0_OUT, scan_line != 0);
    PIN_setOutputValue(mbi_pin_h, MP_CTR_CLK, 1); // pulse clock
    PIN_setOutputValue(mbi_pin_h, MP_CTR_CLK, 0);
    if (scan_line == 7) {
        PIN_setOutputValue(mbi_pin_h, MP_CTR_CLK, 1); // pulse clock
        PIN_setOutputValue(mbi_pin_h, MP_CTR_CLK, 0);
    }
}

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

volatile uint8_t gclk_state = 0;

GPTimerCC26XX_Handle gclk_timer_h;

void timer_init_task();

void init_mbi() {
    // Set up our GPIO:
    mbi_pin_h = PIN_open(&mbi_pin_state, mbi_pin_table);
    // Pulse shift register inputs low to clear:
    PIN_setOutputValue(mbi_pin_h, MP0_CLR, 0);
    PIN_setOutputValue(mbi_pin_h, MP1_CLR, 0);
    PIN_setOutputValue(mbi_pin_h, MP0_CLR, 1);
    PIN_setOutputValue(mbi_pin_h, MP1_CLR, 1);

    do { // Clear out & prime the shift register:
        mp_shift();
    } while (scan_line);
    do { // Clear out & prime the shift register:
        mp_shift();
    } while (scan_line);
}

void init_switch() {
    sw_pin_h = PIN_open(&sw_pin_state, sw_pin_table);
//    PIN_registerIntCb(sw_pin_h, &f_button_cb);

    Clock_Params clockParams;
    Error_Block eb;
    Error_init(&eb);
    Clock_Params_init(&clockParams);
    clockParams.period = 100;
    clockParams.startFlag = TRUE;
    sw_clock = Clock_create(sw_clock_f, 2, &clockParams, &eb);

}

uint8_t led_buf[11][7][3] = {
    {{0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}},
    {{0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}},
    {{0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}, {0xff, 0x00, 0x00}},
    {{0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}},
    {{0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}, {0x00, 0xff, 0x00}},
    {{0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}},
    {{0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}, {0x00, 0x00, 0xff}},
    {{0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0,0,0}},
    {{0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0,0,0}},
    {{0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0,0,0}},
    {{0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0xff, 0xff, 0xff}, {0,0,0}},
};


uint8_t tlc_led_map[15][15][3] = {
    { {0, 0, 0}, {0, 0, 1}, {0, 0, 2}, {1, 0, 0}, {1, 0, 1}, {1, 0, 2}, {2, 0, 0}, {2, 0, 1}, {2, 0, 2}, {3, 0, 0}, {3, 0, 1}, {3, 0, 2}, {4, 0, 0}, {4, 0, 1}, {4, 0, 2}},
    { {0, 1, 0}, {0, 1, 1}, {0, 1, 2}, {1, 1, 0}, {1, 1, 1}, {1, 1, 2}, {2, 1, 0}, {2, 1, 1}, {2, 1, 2}, {3, 1, 0}, {3, 1, 1}, {3, 1, 2}, {4, 1, 0}, {4, 1, 1}, {4, 1, 2}},
    { {0, 2, 0}, {0, 2, 1}, {0, 2, 2}, {1, 2, 0}, {1, 2, 1}, {1, 2, 2}, {2, 2, 0}, {2, 2, 1}, {2, 2, 2}, {3, 2, 0}, {3, 2, 1}, {3, 2, 2}, {4, 2, 0}, {4, 2, 1}, {4, 2, 2}},
    { {0, 3, 0}, {0, 3, 1}, {0, 3, 2}, {1, 3, 0}, {1, 3, 1}, {1, 3, 2}, {2, 3, 0}, {2, 3, 1}, {2, 3, 2}, {3, 3, 0}, {3, 3, 1}, {3, 3, 2}, {4, 3, 0}, {4, 3, 1}, {4, 3, 2}},
    { {0, 4, 0}, {0, 4, 1}, {0, 4, 2}, {1, 4, 0}, {1, 4, 1}, {1, 4, 2}, {2, 4, 0}, {2, 4, 1}, {2, 4, 2}, {3, 4, 0}, {3, 4, 1}, {3, 4, 2}, {4, 4, 0}, {4, 4, 1}, {4, 4, 2}},
    { {0, 5, 0}, {0, 5, 1}, {0, 5, 2}, {1, 5, 0}, {1, 5, 1}, {1, 5, 2}, {2, 5, 0}, {2, 5, 1}, {2, 5, 2}, {3, 5, 0}, {3, 5, 1}, {3, 5, 2}, {4, 5, 0}, {4, 5, 1}, {4, 5, 2}},
    { {0, 6, 0}, {0, 6, 1}, {0, 6, 2}, {1, 6, 0}, {1, 6, 1}, {1, 6, 2}, {2, 6, 0}, {2, 6, 1}, {2, 6, 2}, {3, 6, 0}, {3, 6, 1}, {3, 6, 2}, {4, 6, 0}, {4, 6, 1}, {4, 6, 2}},
    { {5, 1, 0}, {5, 1, 1}, {5, 1, 2}, {5, 2, 0}, {5, 2, 1}, {5, 2, 2}, {5, 3, 0}, {5, 3, 1}, {5, 3, 2}, {5, 4, 0}, {5, 4, 1}, {5, 4, 2}, {5, 5, 0}, {5, 5, 1}, {5, 5, 2}},
    { {6, 1, 0}, {6, 1, 1}, {6, 1, 2}, {6, 2, 0}, {6, 2, 1}, {6, 2, 2}, {6, 3, 0}, {6, 3, 1}, {6, 3, 2}, {6, 4, 0}, {6, 4, 1}, {6, 4, 2}, {6, 5, 0}, {6, 5, 1}, {6, 5, 2}},
    { {5, 0, 0}, {5, 0, 1}, {5, 0, 2}, {6, 0, 0}, {6, 0, 1}, {6, 0, 2}, {5, 6, 0}, {5, 6, 1}, {5, 6, 2}, {6, 6, 0}, {6, 6, 1}, {6, 6, 2}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
    { {7, 1, 0}, {7, 1, 1}, {7, 1, 2}, {7, 2, 0}, {7, 2, 1}, {7, 2, 2}, {7, 3, 0}, {7, 3, 1}, {7, 3, 2}, {7, 4, 0}, {7, 4, 1}, {7, 4, 2}, {7, 5, 0}, {7, 5, 1}, {7, 5, 2}},
    { {10, 5, 0}, {10, 5, 1}, {10, 5, 2}, {10, 2, 0}, {10, 2, 1}, {10, 2, 2}, {10, 1, 0}, {10, 1, 1}, {10, 1, 2}, {10, 0, 0}, {10, 0, 1}, {10, 0, 2}, {10, 3, 0}, {10, 3, 1}, {10, 3, 2}},
    { {9, 5, 0}, {9, 5, 1}, {9, 5, 2}, {9, 2, 0}, {9, 2, 1}, {9, 2, 2}, {9, 1, 0}, {9, 1, 1}, {9, 1, 2}, {9, 4, 0}, {9, 4, 1}, {9, 4, 2}, {9, 3, 0}, {9, 3, 1}, {9, 3, 2}},
    { {8, 3, 0}, {8, 3, 1}, {8, 3, 2}, {8, 4, 0}, {8, 4, 1}, {8, 4, 2}, {8, 2, 0}, {8, 2, 1}, {8, 2, 2}, {8, 1, 0}, {8, 1, 1}, {8, 1, 2}, {8, 5, 0}, {8, 5, 1}, {8, 5, 2}},
    { {7, 0, 0}, {7, 0, 1}, {7, 0, 2}, {10, 4, 0}, {10, 4, 1}, {10, 4, 2}, {9, 0, 0}, {9, 0, 1}, {9, 0, 2}, {8, 0, 0}, {8, 0, 1}, {8, 0, 2}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
};

uint8_t screen2[3][3] = {{255, 255, 255,}, {255, 255, 255,}, {255, 255, 255,},};

SPI_Transaction gs_data_transaction;

void mp_tick() {
    for (uint8_t i=0; i<15; i++) {
        tlc_gs_buf[2 + i*2] = 0xff; //*tlc_led_map[scan_line][i];
        tlc_gs_buf[2 + i*2 + 1] = 0xff; // MSB
//        tlc_gs_buf[2 + i*2 + 1] = led_buf[tlc_led_map[scan_line][i][0]][tlc_led_map[scan_line][i][1]][tlc_led_map[scan_line][i][2]];
    }

    gs_data_transaction.txBuf = tlc_gs_buf;
    gs_data_transaction.rxBuf = NULL;
    gs_data_transaction.count = sizeof all_on;
    SPI_transfer(tlc_spi, &gs_data_transaction);
}

void tlc_spi_cb(SPI_Handle handle, SPI_Transaction *transaction) {
    // We only have the one SPI, so we can pretty well ignore these parameters.
    // TODO: We may want to check whether transaction.status == SPI_TRANSFER_COMPLETED???
    // LATCH!
    static uint16_t hwiKey;
    // Disable hardware interrupts.
    hwiKey  = (uint16_t) Hwi_disable();

    PIN_setOutputValue(mbi_pin_h, LED_LE, 1);
    if (((uint8_t *)transaction->txBuf)[0] == TLC_THISISGS) mp_shift();
    PIN_setOutputValue(mbi_pin_h, LED_LE, 0);

    Hwi_restore(hwiKey);
}

Task_Struct qcTask;
char qcTaskStack[2000];

void qc_task_fn(UArg a0, UArg a1)
{
    GPTimerCC26XX_Params gpt_params;
    GPTimerCC26XX_Params_init(&gpt_params);
    gpt_params.mode  = GPT_MODE_PERIODIC_UP;
    gpt_params.width = GPT_CONFIG_16BIT;
    gclk_timer_h = GPTimerCC26XX_open(QC14BOARD_GPTIMER0A, &gpt_params);

    GPTimerCC26XX_setLoadValue(gclk_timer_h, LED_MP_RATE);
    GPTimerCC26XX_registerInterrupt(gclk_timer_h, mp_tick, GPT_INT_TIMEOUT);

    PWM_Handle gclk_pwm_h;
    PWM_Params pwm_params;

    PWM_init();

    PWM_Params_init(&pwm_params);
    pwm_params.idleLevel = PWM_IDLE_LOW;
    pwm_params.periodUnits = PWM_PERIOD_HZ;
    pwm_params.periodValue = 24000000;
    pwm_params.dutyUnits = PWM_DUTY_FRACTION;
    pwm_params.dutyValue = PWM_DUTY_FRACTION_MAX/2;

    gclk_pwm_h = PWM_open(QC14BOARD_PWM_GSCLK, &pwm_params);

    PWM_start(gclk_pwm_h);

    Task_sleep(10); // 10 ticks. (100 us)

    Error_Block eb;

    ADC_Handle adc;
    ADC_Params adcp;
    ADC_init();

    ADC_Params_init(&adcp);
    adc = ADC_open(QC14BOARD_ADC7_LIGHT, &adcp);

    SPI_Params tlc_spi_params;

    SPI_Params_init(&tlc_spi_params);
    tlc_spi_params.transferMode = SPI_MODE_CALLBACK;
    tlc_spi_params.mode = SPI_MASTER;
    tlc_spi_params.transferCallbackFxn = tlc_spi_cb;
    tlc_spi_params.bitRate = 12000000;
    tlc_spi = SPI_open(QC14BOARD_TLC_SPI, &tlc_spi_params);

    SPI_Transaction fun_data_transaction;
    fun_data_transaction.txBuf = fun_base;
    fun_data_transaction.rxBuf = NULL;
    fun_data_transaction.count = sizeof fun_base;

    SPI_transfer(tlc_spi, &fun_data_transaction);

    while (fun_data_transaction.status != SPI_TRANSFER_COMPLETED);

    GPTimerCC26XX_start(gclk_timer_h);

    UART_init();

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

int main()
{

  PIN_init(BoardGpioInitTable);

  Power_setDependency(PowerCC26XX_XOSC_HF);

#ifndef POWER_SAVING
  /* Set constraints for Standby, powerdown and idle mode */
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif // POWER_SAVING

  SPI_init();

  init_ble();
  init_switch();
  init_mbi();

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
