#include "MK82F25615.h"
#include "LAL.h"

#define LED_RED   (0x100) /// PTC8
#define LED_GREEN (0x200) /// PTC9
#define LED_BLUE  (0x400) /// PTC10

#define GPIO (0x0100)
#define PORTC_SIM_SCGC_MASK (0x800)
#define RUNNING (1)

int main(void) 
{
    /// SIM PTC
    SIM->SCGC5 |= PORTC_SIM_SCGC_MASK;
    
    /// PORTC configure pins
    PORTC->PCR[ 8] = GPIO; /// R
    PORTC->PCR[ 9] = GPIO; /// G
    PORTC->PCR[10] = GPIO; /// B
    
    /// GPIOC configure
    GPIOC->PDDR |= LED_RED;
    GPIOC->PDDR |= LED_GREEN;
    GPIOC->PDDR |= LED_BLUE;
    
    // TURN OFF LED
    GPIOC->PSOR |= LED_RED;
    GPIOC->PSOR |= LED_GREEN;
    GPIOC->PSOR |= LED_BLUE;
    
    
    //printf("hello world \n");
    
    u32 counter = 0;
    
    while(RUNNING)
    {
        if(counter == 1000000/4)
        {
            /// TOGGLE LED ON
            GPIOC->PSOR |= LED_GREEN;
        }
        if(counter == 1000000/2)
        {
            /// TOGGLE LED OFF
            GPIOC->PCOR |= LED_GREEN;
        }
        else if (counter == 1000000)
        {
            counter = 0;
        }
        counter++;
    }
}

