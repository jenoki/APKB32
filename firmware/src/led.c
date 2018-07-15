/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "led.h"
#include "system_config/default/framework/driver/oc/drv_oc_static.h"
#include "system_config/default/framework/driver/tmr/drv_tmr_static.h"
/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */
uint16_t oc_period;
uint16_t period;

void LED_Initialize(){
    period=DRV_TMR1_PeriodValueGet();
    oc_period = period / 2;
}

// *****************************************************************************

/** 
  @Function
    int ExampleInterfaceFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */
void LED_Set(uint8_t color, uint8_t blink) {
    DRV_OC0_Stop();
    DRV_OC1_Stop();
    DRV_OC2_Stop();

    switch (blink){
        case LED_BLINK_NONE:
        case LED_BLINK_SLOW:
            DRV_TMR1_PeriodValueSet(period);
            break;
        case LED_BLINK_FAST:
            DRV_TMR1_PeriodValueSet(period / 2);
            break;
    }

    uint16_t pd;

    pd = ((color & LED_BIT_RED) != 0) ?
        (blink==LED_BLINK_NONE)? 0
            : (blink==LED_BLINK_FAST)? (oc_period/2) : oc_period 
        : (period-1);
    DRV_OC0_PulseWidthSet(pd);

    pd = ((color & LED_BIT_BLUE) != 0) ? 
        (blink==LED_BLINK_NONE)? 0
            : (blink==LED_BLINK_FAST)? (oc_period/2) : oc_period
        : (period-1);
    DRV_OC1_PulseWidthSet(pd);

    pd = ((color & LED_BIT_GREEN) != 0) ?
            (blink==LED_BLINK_NONE)? 0 
                : (blink==LED_BLINK_FAST)? (oc_period/2) : oc_period
            : (period-1);
    DRV_OC2_PulseWidthSet(pd);

    DRV_OC2_Start();
    DRV_OC1_Start();
    DRV_OC0_Start();

}


/* *****************************************************************************
 End of File
 */
