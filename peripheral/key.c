/*
 * key.c
 *
 *  Created on: 2020Äê9ÔÂ9ÈÕ
 *      Author: mi
 */
#include "key.h"
#include "em_rtcc.h"
#include "em_gpio.h"
#include "native_gecko.h"
#include "hal-config.h"
#include "gpiointerrupt.h"

// button press timestamp for long/short Push Button 0 press detection
static uint32 pb0_press;
void gpioint(uint8_t pin)
{
    uint32_t t_diff;

    if (pin == BSP_BUTTON0_PIN) {
        if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0) {
            // PB0 pressed - record RTCC timestamp
            pb0_press = RTCC_CounterGet();
        } else {
            // PB0 released - check if it was short or long press
            t_diff = RTCC_CounterGet() - pb0_press;
            if (t_diff < LONG_PRESS_TIME_TICKS) {
                gecko_external_signal(EXT_SIGNAL_PB0_SHORT_PRESS);
            } else {
                gecko_external_signal(EXT_SIGNAL_PB0_LONG_PRESS);
            }
        }
    }
}

/**
 * button initialization. Configure pushbuttons PB0,PB1
 * as inputs.
 */
void button_init(void)
{
    // configure pushbutton PB0 as inputs, with pull-up enabled
    GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);

    GPIOINT_Init();

    /* configure interrupt for PB0 and PB1, both falling and rising edges */
    GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, true, true);

    /* register the callback function that is invoked when interrupt occurs */
    GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, gpioint);
}
