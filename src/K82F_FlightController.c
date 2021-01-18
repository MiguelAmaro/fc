#include "MK82F25615.h"
#include "LAL.h"
#include "FlightController_Debug.h"

#define LED_RED   (0x100) /// PTC8
#define LED_GREEN (0x200) /// PTC9
#define LED_BLUE  (0x400) /// PTC10

#define MUX_GPIO (0x0100)
#define RUNNING (1)
#define GET_UART_BYTE ((u8)LPUART4->DATA & 0xFFU)

#define DASSERT(Expression) if(!(Expression)){GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

int 
main(void) 
{
    Debug_init_uart(115200);
    
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
    
    
    
    // ****************************************
    // COMPASS SETUP - FXOS8700CQ
    // ****************************************
    // PTA2/ I2C3_SCL
    // PTA1/ I2C3_SDA
    // PTC13
    
    
    // ****************************************
    // RADIO SETUP
    // ****************************************
    
    // ************ MOTOR CONTROL *************
    {
        // ****************************************
        // FLEXTIMER SETUP - pg.1442
        // ****************************************
        SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
        
        
        // ****************************************
        // DMA SETUP - pg. 539
        // ****************************************
        
        
    }
    // ****************************************
    // CAMERA SETUP
    // ****************************************
    
    
    
    u32 counter = 0;
    
    while(RUNNING)
    {
        printf("hello world \n\r");
        
        if(counter == 1000/4)
        {
            /// TOGGLE LED ON
            GPIOC->PSOR |= LED_BLUE;
        }
        if(counter == 1000/2)
        {
            /// TOGGLE LED OFF
            GPIOC->PCOR |= LED_BLUE;
        }
        else if (counter == 1000)
        {
            counter = 0;
        }
        counter++;
    }
}

