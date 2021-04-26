/* date = January 18th 2021 5:12 pm */

#ifndef FLIGHTCONTROLLER_ONBOARDLEDS_H
#define FLIGHTCONTROLLER_ONBOARDLEDS_H

#include "MK82F25615.h"
#include "LAL.h"

#define LED_RED   (0x100) /// PTC8
#define LED_GREEN (0x200) /// PTC9
#define LED_BLUE  (0x400) /// PTC10

#define MUX_GPIO (0x0100)

void
OBLEDs_init(void)
{
    // ****************************************
    // ONBOARD LED SETUP
    // ****************************************
    /// SIM PTC
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    
    /// PORTC configure pins
    PORTC->PCR[ 8] = MUX_GPIO; /// R
    PORTC->PCR[ 9] = MUX_GPIO; /// G
    PORTC->PCR[10] = MUX_GPIO; /// B
    
    /// GPIOC configure
    GPIOC->PDDR |= LED_RED;
    GPIOC->PDDR |= LED_GREEN;
    GPIOC->PDDR |= LED_BLUE;
    
    // TURN OFF LED
    GPIOC->PSOR |= LED_RED;
    GPIOC->PSOR |= LED_GREEN;
    GPIOC->PSOR |= LED_BLUE;
    
    return;
}

#endif //FLIGHTCONTROLLER_ONBOARDLEDS_H


