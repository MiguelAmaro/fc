#include "MK82F25615.h"
#include "LAL.h"
#include "FlightController_Debug.h"
#include "FlightController_OnboardLEDs.h"

#define RUNNING (1)
#define DASSERT(Expression) if(!(Expression)){GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

#define DSHOT_CMD_BUFFER_SIZE (16 + 1)
global volatile u8  Dshot_command_buffer[DSHOT_CMD_BUFFER_SIZE];
//global u16 Dshot_last_command = 48;

int 
main(void) 
{
    
    Debug_init_uart(115200);
    OBLEDs_init();
    
    printf("Core Frequency: %d \n\n\r", SystemCoreClock);
    
    // NOTE(MIGUEL): DUE TO PIN SHARING MODULES MUST BE TESTED INDIVIDUALLY
    // TODO(MIGUEL): CREATE A PIN MUTUAL EXCLUSION SYSTEM
    
    // ************ ECOMPASS CONTROL *************
    {
        /*
        // ****************************************
        // I2C SETUP
        // ****************************************
        
        
        
        // ****************************************
        // FXOS8700CQ SETUP
        // ****************************************
        // PTA2/ I2C3_SCL
        // PTA1/ I2C3_SDA
        // PTC13
#define ECOMPASS_INTERRUPT (0x2000) /// PTC13
        // Enable PORTA and PORTC
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        
        // Config PTA Pins
        PORTA->PCR[ 1] = PORT_PCR_MUX(4); /// SDATA
        PORTA->PCR[ 2] = PORT_PCR_MUX(4); /// SCLOCK
        
        // Config PTC Pins
        PORTC->PCR[13] = PORT_PCR_MUX ( 1); /// FXOS8700CQ Interrupt
        GPIOC->PDDR   |= GPIO_PDDR_PDD(13);
        */
        
    }
    
    // ************ RADIO CONTROL *************
    {
        // ****************************************
        // SPI SETUP
        // ****************************************
        /*
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
        
        // Config PTB Pins
        // NOTE(MIGUEL): PTB23 Shared with Camera 
        PORTB->PCR[ 9] = PORT_PCR_MUX ( 2); /// PCS1
        PORTB->PCR[23] = PORT_PCR_MUX ( 1); /// IRQ
        GPIOC->PDDR   |= GPIO_PDDR_PDD(23); /// GPIO
        
        // Config PTC Pins
        PORTC->PCR[0] = PORT_PCR_MUX (1); /// CHIP ENABLE
        GPIOC->PDDR  |= GPIO_PDDR_PDD(0); /// GPIO
        
        // Config PTD Pins
        PORTC->PCR[5] = PORT_PCR_MUX ( 7); /// SCLOCK
        PORTC->PCR[6] = PORT_PCR_MUX ( 7); /// SOUT
        PORTC->PCR[7] = PORT_PCR_MUX ( 7); /// SIN
        GPIOC->PDDR  |= GPIO_PDDR_PDD(13);
        */
        
        // ****************************************
        // NRF24L01P SETUP
        // ****************************************
        
    }
    // NOTE(MIGUEL): MOTOR CODE IN MOTOR BRANCH ALL RELATED CODE IN THIST BRANGH WILL BE DELETED
    
    // ************ CAMERA CONTROL *************
    {
        /*
        // ****************************************
        // FLEXIO SETUP
        // ****************************************
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        
        // NOTE(MIGUEL): PTA1 & PTA2 are shared with ECompass
        PORTA->PCR[ 1] = PORT_PCR_MUX(1); /// SDATA
        PORTA->PCR[ 2] = PORT_PCR_MUX(1); /// SCLOCK
        
        // NOTE(MIGUEL): PTB23 Shared with Radio 
        PORTB->PCR[ 0] = PORT_PCR_MUX(7); /// FXIO_DATA__0 - CAM_PCLK
        PORTB->PCR[ 2] = PORT_PCR_MUX(7); /// FXIO_DATA__2 - CAM_VSYNC
        PORTB->PCR[ 3] = PORT_PCR_MUX(7); /// FXIO_DATA__3 - CAM_HREF
        PORTB->PCR[10] = PORT_PCR_MUX(7); /// FXIO_DATA__4 - CAM_DATA_0
        PORTB->PCR[11] = PORT_PCR_MUX(7); /// FXIO_DATA__5 - CAM_DATA_1
        PORTB->PCR[18] = PORT_PCR_MUX(7); /// FXIO_DATA__6 - CAM_DATA_2
        PORTB->PCR[19] = PORT_PCR_MUX(7); /// FXIO_DATA__7 - CAM_DATA_3
        PORTB->PCR[20] = PORT_PCR_MUX(7); /// FXIO_DATA__8 - CAM_DATA_4
        PORTB->PCR[21] = PORT_PCR_MUX(7); /// FXIO_DATA__9 - CAM_DATA_5
        PORTB->PCR[22] = PORT_PCR_MUX(7); /// FXIO_DATA_10 - CAM_DATA_6
        PORTB->PCR[23] = PORT_PCR_MUX(7); /// FXIO_DATA_11 - CAM_DATA_7
        
        PORTC->PCR[ 3] = PORT_PCR_MUX(5); /// CLOCKOUT
        PORTC->PCR[ 8] = PORT_PCR_MUX(1); /// CAM RESET
        PORTC->PCR[ 9] = PORT_PCR_MUX(1); /// POWER DOWN
        
        GPIOC->PDDR   |= GPIO_PDDR_PDD(8);
        GPIOC->PDDR   |= GPIO_PDDR_PDD(9);
        */
        
        // ****************************************
        // OV7670 SETUP
        // ****************************************
    }
    
    printf("Core Frequency: %d \n\n\r", SystemCoreClock);
    
    printf("\n\n\r");
    u32 counter = 0;
    
    while(RUNNING)
    {
        //printf("Core Frequency: %d \n\n\r", SystemCoreClock);
        if(counter == 1000000/2)
        {
            /// TOGGLE LED ON
            GPIOC->PCOR |= LED_BLUE;
            
        }
        else if (counter == 1000000)
        {
            /// TOGGLE LED OFF
            GPIOC->PSOR |= LED_BLUE;
            counter = 0;
        }
        counter++;
    }
}

