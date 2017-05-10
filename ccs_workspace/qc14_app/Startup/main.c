#include <xdc/runtime/Error.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>
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

#define MBI_CMD_VSYNC 2
#define MBI_CMD_LATCH 1
#define MBI_CMD_PREACT 14
#define MBI_CMD_READCFG1 5
#define MBI_CMD_CFG1 4
#define MBI_CMD_CFG2 8
#define MBI_CMD_RST 10
volatile uint16_t scan_line = 0;
volatile uint16_t gclk = 0;
volatile uint8_t mbi_needs_vsync = 0;
volatile uint8_t mbi_writing = 0;
volatile uint8_t mbi_wr_sl = 0;
volatile uint8_t mbi_wr_ch = 0;
volatile uint8_t mbi_wr_bit = 0;
volatile uint8_t mbi_gclk_hold = 0;
volatile uint8_t mbi_vsync_ready = 0;
volatile uint8_t mbi_vsync_wait = 0;
Clock_Handle gclk_clock;
Clock_Handle sw_clock;

Semaphore_Handle led_load_sem;

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


uint16_t mbi_r_dat(uint16_t dat) {
    uint16_t bit = 16;
    uint16_t val = 0x0000;

    while (bit) {
        bit--;
        PIN_setOutputValue(mbi_pin_h, LED_DOUT, (dat>>bit) & 0x0001);
        val |= (PIN_getInputValue(LED_DIN) << bit);
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 1);
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 0);
    }

    return val;
}

void mp_shift() {
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
        Semaphore_post(led_load_sem);
    } else if (sw_c_curr && sw_c_last && sw_c_clicked) {
        sw_c_clicked = 0;
        // unclicked.
    }
    sw_c_last = sw_c_curr;
}

volatile uint8_t gclk_state = 0;

GPTimerCC26XX_Handle gclk_timer_h;

void mp_tick() {
    mp_shift();
}

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

Task_Struct qcLoadTask;
char qcLoadTaskStack[400];

void qc_load_led_fn(UArg a0, UArg a1) {
    while (1) {
        Semaphore_pend(led_load_sem, BIOS_WAIT_FOREVER);
    }
}

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
        0x00,
//        0x7f, // after this there are 16 7-tets of dot correction. // blank off
        //We'll send it as 14 octets.
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

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

uint8_t rx_buf[33];

Task_Struct qcTask;
char qcTaskStack[2000];

void qc_task_fn(UArg a0, UArg a1)
{
    GPTimerCC26XX_Params gpt_params;
    GPTimerCC26XX_Params_init(&gpt_params);
    gpt_params.mode  = GPT_MODE_PERIODIC_UP;
    gpt_params.width = GPT_CONFIG_16BIT;
    gclk_timer_h = GPTimerCC26XX_open(QC14BOARD_GPTIMER0A, &gpt_params);

    GPTimerCC26XX_setLoadValue(gclk_timer_h, 24000);
    GPTimerCC26XX_registerInterrupt(gclk_timer_h, mp_tick, GPT_INT_TIMEOUT);
    GPTimerCC26XX_start(gclk_timer_h);

    PWM_Handle gclk_pwm_h;
    PWM_Params pwm_params;

    PWM_init();

    PWM_Params_init(&pwm_params);
    pwm_params.idleLevel = PWM_IDLE_LOW;
    pwm_params.periodUnits = PWM_PERIOD_HZ;
    pwm_params.periodValue = 10000000;
    pwm_params.dutyUnits = PWM_DUTY_FRACTION;
    pwm_params.dutyValue = PWM_DUTY_FRACTION_MAX/2;

    gclk_pwm_h = PWM_open(QC14BOARD_PWM_GSCLK, &pwm_params);

    PWM_start(gclk_pwm_h);

    Task_sleep(10); // 10 ticks. (100 us)

    Error_Block eb;

    led_load_sem = Semaphore_create(0, NULL, &eb);

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = qcLoadTaskStack;
    taskParams.stackSize = sizeof(qcLoadTaskStack);
    taskParams.priority = 1;
    Task_construct(&qcLoadTask, qc_load_led_fn, &taskParams, NULL);

    ADC_Handle adc;
    ADC_Params adcp;
    ADC_init();

    ADC_Params_init(&adcp);
    adc = ADC_open(QC14BOARD_ADC7_LIGHT, &adcp);

    SPI_Handle tlc_spi;
    SPI_Params tlc_spi_params;

    SPI_Params_init(&tlc_spi_params);
    // Defaults should be good.

    tlc_spi_params.mode = SPI_MODE_BLOCKING;
    tlc_spi_params.bitRate = 100000;

    tlc_spi = SPI_open(QC14BOARD_TLC_SPI, &tlc_spi_params);

    SPI_Transaction fun_data_transaction;

    fun_data_transaction.txBuf = fun_base;
    fun_data_transaction.rxBuf = NULL;
    fun_data_transaction.count = sizeof fun_base;

    SPI_transfer(tlc_spi, &fun_data_transaction);
    //     LATCH!
    PIN_setOutputValue(mbi_pin_h, LED_LE, 1);
    Task_sleep(1);
    PIN_setOutputValue(mbi_pin_h, LED_LE, 0);


    SPI_Transaction gs_data_transaction;
    gs_data_transaction.txBuf = all_on;
    gs_data_transaction.rxBuf = rx_buf;
    gs_data_transaction.count = sizeof all_on;
    SPI_transfer(tlc_spi, &gs_data_transaction);
    // LATCH!
    PIN_setOutputValue(mbi_pin_h, LED_LE, 1);
    Task_sleep(1);
    PIN_setOutputValue(mbi_pin_h, LED_LE, 0);


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


    while (1)
    {
        Task_sleep(BIOS_WAIT_FOREVER);
    }
}

int main()
{

  PIN_init(BoardGpioInitTable);

//  Power_setDependency(XOSC_HF); // TODO

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
