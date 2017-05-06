#include <xdc/runtime/Error.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>

#include "icall.h"
#include "hal_assert.h"
#include "bcomdef.h"
#include "broadcaster.h"
#include "simple_broadcaster.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
bleUserCfg_t user0Cfg = BLE_USER_CFG;

#endif // USE_DEFAULT_USER_CFG

#include <ti/mw/display/Display.h>

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

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
    LED_GSCLK       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
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
uint16_t scan_line = 0;
uint16_t gclk = 0;
uint8_t mbi_needs_vsync = 0;
Clock_Handle gclk_clock;
Clock_Handle sw_clock;

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

void mbi_gs_clock_swi();


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
        mp_shift();
    } else if (sw_c_curr && sw_c_last && sw_c_clicked) {
        sw_c_clicked = 0;
        // unclicked.
    }
    sw_c_last = sw_c_curr;
}


uint8_t mbi_writing = 0;
uint8_t mbi_wr_sl = 0;
uint8_t mbi_wr_ch = 0;
uint8_t mbi_wr_bit = 0;
uint8_t mbi_gclk_hold = 0;
uint8_t mbi_vsync_ready = 0;
uint8_t mbi_vsync_wait = 0;

void mbi_start_write() {
    mbi_writing = 1;
    mbi_wr_sl = 0;
    mbi_wr_ch = 15;
    mbi_wr_bit = 15;
    mbi_gclk_hold = 0;
    mbi_vsync_ready = 0;
    mbi_vsync_wait = 0;
}

void mbi_gs_clock_swi() {
    static uint16_t led_value = 0x0000; // For now, SHUT IT ALL DOWN.
    // This is the grayscale clock for the LEDs.
    // I'm also going to use it for the DCLK.

    if (mbi_writing) {
        PIN_setOutputValue(mbi_pin_h, LED_DOUT, 0x01 & (led_value >> mbi_wr_bit));
        if (mbi_wr_bit == 0) PIN_setOutputValue(mbi_pin_h, LED_LE, 1); // LATCH.

        // Tick clock:
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 1);
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 0);

        PIN_setOutputValue(mbi_pin_h, LED_LE, 0); // UNLATCH.

        if (!mbi_wr_bit) { // finished last bit (LSB)
            mbi_wr_bit = 15; // each of these is a channel.
            if (!mbi_wr_ch) { // finished last channel (lowest channel)
                mbi_wr_ch = 15; // each of these is a scanline.
                if (mbi_wr_sl == 0) { // finished last scan line (highest scan line) // TODO
                    // now we're done writing. We need to allow at least 50 GCLKs before
                    // arresting GCLK and doing a VSYNC.
                    mbi_writing = 0;
                    mbi_vsync_ready = 1;
                    mbi_vsync_wait = 55;
                } else {
                    mbi_wr_sl++;
                }
            } else {
                mbi_wr_ch--;
            }
        } else {
            mbi_wr_bit--;
        }
    }

    // GCLK logic:

    if (mbi_vsync_ready && !mbi_vsync_wait) { // vsync is needed NOW.
        // Stop the GCLK.
        mbi_gclk_hold = 2;

        PIN_setOutputValue(mbi_pin_h, LED_LE, 1); // VSYNC.
        // Tick clock:
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 1);
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 0);
        // Tick clock:
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 1);
        PIN_setOutputValue(mbi_pin_h, LED_CLK, 0);
        PIN_setOutputValue(mbi_pin_h, LED_LE, 0); // end VSYNC cmd.

        mbi_vsync_ready = 0; // not ready anymore because we already did it.

    } else if (mbi_vsync_ready) { // vsync will be needed in mbi_vsync_wait GCLKs.
        mbi_vsync_wait--;
    }

    if (mbi_gclk_hold) {
        // If the GS clock is HELD, don't tick it.
        // After the hold is over we'll be starting all over again, so it's time to
        // fix the multiplexing.
        mbi_gclk_hold--;
        if (!mbi_gclk_hold) {
            // done holding:
            gclk = 0;
            scan_line = 0;
            do { // Clear out & prime the shift register:
                mp_shift();
            } while (scan_line);
        }
    } else { // Tick the GCLK and handle the multiplexing.
        PIN_setOutputValue(mbi_pin_h, LED_GSCLK, 1);
        if (gclk == 256) {
//            scan_line = (scan_line + 1) % 15;
//            mp_shift(scan_line != 0);
            gclk = 0;
        } else {
            gclk++;
        }
        PIN_setOutputValue(mbi_pin_h, LED_GSCLK, 0);
    }
}

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

    // Prod values:
    mbi_wr_dat_cmd(0x0000, MBI_CMD_PREACT);
    mbi_wr_dat_cmd(0b1000111011101011, MBI_CMD_CFG1);
    mbi_read_cfg1();
    // We are pretending there's only one scan line:
    mbi_wr_dat_cmd(0x0000, MBI_CMD_PREACT);
    mbi_wr_dat_cmd(0b0000000000000000, MBI_CMD_CFG1);
    mbi_read_cfg1();

    // Set up our clocks:
    Clock_Params clockParams;
    Error_Block eb;
    Error_init(&eb);
    Clock_Params_init(&clockParams);
    clockParams.period = 10;
    clockParams.startFlag = TRUE;
    gclk_clock = Clock_create(mbi_gs_clock_swi, 2, &clockParams, &eb);

    mbi_start_write();
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

int main()
{
  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);

  PIN_init(BoardGpioInitTable);

#ifndef POWER_SAVING
  /* Set constraints for Standby, powerdown and idle mode */
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif // POWER_SAVING

  init_ble();
  init_switch();
  init_mbi();

  // And we're off to see the wizard!
  BIOS_start();

  return 0;
}


/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack
 *              project this function will be called when an assert is raised, 
 *              and can be used to observe or trap a violation from expected 
 *              behavior.       
 *              
 *              As an example, for Heap allocation failures the Stack will raise 
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and 
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can 
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an 
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
  // Open the display if the app has not already done so
  if ( !dispHandle )
  {
    dispHandle = Display_open(Display_Type_LCD, NULL);
  }

  Display_print0(dispHandle, 0, 0, ">>>STACK ASSERT");

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> OUT OF MEMORY!");
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL FW ERROR!");
      }
      else
      {
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL ERROR!");
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> ICALL ABORT!");
      HAL_ASSERT_SPINLOCK;
      break;

    default:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> DEFAULT SPINLOCK!");
      HAL_ASSERT_SPINLOCK;
  }

  return;
}


/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS.
 *
 * input parameters
 *
 * @param       eb - Pointer to Error Block.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

/*******************************************************************************
 */
