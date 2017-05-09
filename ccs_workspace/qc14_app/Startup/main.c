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
    LED_DIN         | PIN_INPUT_EN | PIN_PULLDOWN,
//    LED_GSCLK       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, // TODO
    LED_DOUT        | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED_CLK         | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
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

PWM_Handle hPWM;

#define LED_MP_CYCLES 256

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

void mbi_dclk_swi();


void mbi_wr_dat_cmd(uint16_t dat, uint16_t cmd) {
    uint16_t bit = 16;
    while (bit) {
        if (bit == cmd) PIN_setOutputValue(mbi_pin_h, LED_LE, 1);
        bit--;
        PIN_setOutputValue(mbi_pin_h, LED_DOUT, (dat>>bit) & 0x0001);
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 1);
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 0);
    }

    PIN_setOutputValue(mbi_pin_h, LED_LE, 0);
}

uint16_t mbi_read_cfg1() {
    mbi_wr_dat_cmd(0x0000, MBI_CMD_READCFG1);
    volatile uint16_t cfg1 = 0;
    cfg1 = mbi_r_dat(0x0000);
    __nop();
    return cfg1;
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

void mbi_start_write() {
    mbi_writing = 1;
    mbi_wr_sl = 0;
    mbi_wr_ch = 15;
    mbi_wr_bit = 15;
    mbi_gclk_hold = 0;
    mbi_vsync_ready = 0;
    mbi_vsync_wait = 0;
}

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

volatile uint16_t mbi_dat;
volatile uint16_t mbi_cmd;
volatile uint8_t mbi_wr_index = 0;
volatile uint8_t mbi_le_needs_low = 1;
volatile uint8_t mbi_dclk_ticking = 0;

void mbi_wr_dat_cmd_a(uint16_t dat, uint16_t cmd) {
    while (mbi_dclk_ticking); // spin until we can write again.
    mbi_wr_index = 16;
    mbi_cmd = cmd;
    mbi_dat = dat;
    mbi_dclk_ticking = 1;
}

//uint16_t bit = 16;
//while (bit) {
//    if (bit == cmd) PIN_setOutputValue(mbi_pin_h, LED_LE, 1);
//    bit--;
//    PIN_setOutputValue(mbi_pin_h, LED_DOUT, (dat>>bit) & 0x0001);
//    PIN_setOutputValue(mbi_pin_h, LED_CLK, 1);
//    PIN_setOutputValue(mbi_pin_h, LED_CLK, 0);
//}
//
//PIN_setOutputValue(mbi_pin_h, LED_LE, 0);


void mbi_dclk_swi() {
    static uint8_t dclk_state = 0;
    static uint16_t led_value = 0x0000; // For now, SHUT IT ALL DOWN.

    if (!mbi_dclk_ticking) return;

    dclk_state = ~dclk_state;
    PINCC26XX_setOutputValue(LED_CLK, dclk_state);

    if (dclk_state) { // We have just gone HIGH: Time to read DIN

    } else { // We have just gone LOW: time to change LE, or change DOUT.
        if (mbi_wr_index) { // We're writing something. Set DOUT and/or assert LE.
            if (mbi_wr_index == mbi_cmd) PINCC26XX_setOutputValue(LED_LE, 1);
            PINCC26XX_setOutputValue(LED_DOUT, (mbi_dat>>(mbi_wr_index-1)) & 0x0001);
            mbi_wr_index--;
        } else { // we are set to ticking, but we've finished writing everything.
            PINCC26XX_setOutputValue(LED_LE, 0);
            mbi_dclk_ticking = 0; // done.
        }
    }
}

volatile uint8_t gclk_state = 0;

GPTimerCC26XX_Handle gclk_timer_h;

void gclk_tick() {
    gclk_state = ~gclk_state;

    if (mbi_vsync_wait)
        mbi_vsync_wait--;

    if (mbi_gclk_hold) {
        // If the GS clock is HELD, don't tick it.
        // After the hold is over we'll be starting all over again, so it's time to
        // fix the multiplexing.
        if (gclk_state) mbi_gclk_hold--;
        if (!mbi_gclk_hold) {
            // done holding:
            gclk = 0;
            scan_line = 0;
            do { // Clear out & prime the shift register:
                mp_shift();
            } while (scan_line);
        }
        return;
    }

    PINCC26XX_setOutputValue(LED_GSCLK, gclk_state);

    if (gclk_state) return; // if we just brought it high, nothing to do.

    // Tick the GCLK and handle the multiplexing.
    if (gclk == LED_MP_CYCLES) {
        mp_shift();
        gclk = 0;
    } else {
        gclk++;
    }
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

        continue;

        // Reset is SUPPOSED to turn everything off.
//        mbi_wr_dat_cmd(0x0000, MBI_CMD_RST);

        Task_sleep(10);

        // Now let's wait for a fresh GCLK cycle, shall we?
        while (gclk);

        // Now. We're pretending there's only 1 scan line.
        // We only have one chip, and we have 16 channels.
        // The process for setting a grayscale is this.
        //  Send ch15,14,13,...0.
        //  Each GS setting is 16 bits, MSB first.
        //  It ends with a data latch command, which is a 1-clock LE assertion.
        for (uint8_t i=0; i<16; i++) {
            mbi_wr_dat_cmd_a(0x0ff0, MBI_CMD_LATCH);
        }
        // Then, after all 16x16 bits of GS have been clocked in, it's time to do a VSYNC.
        // We need to wait for at least 50 GCLKs before we start the vsync.

        mbi_vsync_wait = 100;
        while (mbi_vsync_wait);
        //VSYNC procedure:
        // Stop GCLK.
        mbi_gclk_hold = 100;
        mbi_wr_dat_cmd_a(0x0000, MBI_CMD_VSYNC);
        mbi_wr_dat_cmd_a(0x0000, 0); // generate some extra clocks which may or may not be needed.
        while (mbi_dclk_ticking); // spin until we can write again.
        while (mbi_gclk_hold) {
            __nop();
        }
        // GCLK will start itself back up automatically.
    }
}

Task_Struct qcTask;
char qcTaskStack[1200];

void qc_task_fn(UArg a0, UArg a1)
{
    // Set up our clocks:
    Clock_Params clockParams;
    Error_Block eb;
    Error_init(&eb);
    Clock_Params_init(&clockParams);
    clockParams.period = 1;
    clockParams.startFlag = TRUE;
    gclk_clock = Clock_create(mbi_dclk_swi, 1, &clockParams, &eb);

    GPTimerCC26XX_Params gpt_params;
    GPTimerCC26XX_Params_init(&gpt_params);
    gpt_params.mode  = GPT_MODE_PERIODIC_UP;
    gpt_params.width = GPT_CONFIG_16BIT;
    gclk_timer_h = GPTimerCC26XX_open(QC14BOARD_GPTIMER0A, &gpt_params);

    GPTimerCC26XX_setLoadValue(gclk_timer_h, 4800);
    GPTimerCC26XX_registerInterrupt(gclk_timer_h, gclk_tick, GPT_INT_TIMEOUT);
    GPTimerCC26XX_start(gclk_timer_h);

    // Prod values:
    mbi_read_cfg1();
    mbi_wr_dat_cmd_a(0x0000, MBI_CMD_PREACT);
    mbi_wr_dat_cmd_a(0b1000111011101011, MBI_CMD_CFG1);
    while (mbi_dclk_ticking);
    if (mbi_read_cfg1() != 0b1000111011101011) {
        __nop();
    }
    // We are pretending there's only one scan line:

    mbi_wr_dat_cmd_a(0x0000, MBI_CMD_PREACT);
    mbi_wr_dat_cmd_a(0b0000000000111111, MBI_CMD_CFG1);
    while (mbi_dclk_ticking);
    mbi_read_cfg1();

    Task_sleep(10); // 10 ticks. (100 us)

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
