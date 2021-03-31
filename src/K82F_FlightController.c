#include "MK82F25615.h"
#include "LAL.h"
#include "FlightController_Debug.h"
#include "FlightController_OnboardLEDs.h"
#include "FlightController_I2C.h"
#include "FlightController_ECompass.h"

#define RUNNING (1)
#define DASSERT(Expression) if(!(Expression)){GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

//~ 0V7670 DEFINITIONS
#define OV7670_SLAVE_ADDRESS (0x42)
#define OV7670_REG_VREF      (0x03)
#define OV7670_REG_PID       (0x0A)
#define OV7670_REG_VER       (0x0B)
#define OV7670_REG_COM7      (0x12)
#define OV7670_REG_COM8      (0x13)

//#define test_camera
#define test_ecompass

int main(void) 
{
    Debug_init_uart(115200);
    OBLEDs_init();
    
    printf("Core Frequency: %d \n\n\r", SystemCoreClock);
    
    // NOTE(MIGUEL): DUE TO PIN SHARING MODULES MUST BE TESTED INDIVIDUALLY
    // TODO(MIGUEL): CREATE A PIN MUTUAL EXCLUSION SYSTEM
    
    // ************ ECOMPASS CONTROL *************
    {
#ifdef test_ecompass
        // ****************************************
        // I2C SETUP
        // ****************************************
        
        I2C_init();
        
        // ****************************************
        // FXOS8700CQ SETUP
        // ****************************************
        // PTA2/ I2C3_SCL
        // PTA1/ I2C3_SDA
        // PTC13
        
        // Enable PORTC
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        
        // Config PTC Pins
        PORTC->PCR[13] = PORT_PCR_MUX  ( 1); /// FXOS8700CQ Interrupt
        GPIOC->PDDR   |= GPIO_PDDR_PDD (13);
        GPIOC->PSOR   |= GPIO_PSOR_PTSO(13);
        
        Ecompass_init((void *)0x00);
#endif
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
    // NOTE(MIGUEL): MOTOR CODE IN MOTOR BRANCH ALL RELATED CODE IN THIS BRANGH WILL BE DELETED
    
    // ************ CAMERA CONTROL *************
    {
#ifdef test_camera
        I2C_init();
        
        // ****************************************
        // FLEXIO SETUP
        // ****************************************
        // TODO(MIGUEL): Check CLOCKOUT with o-scope
        
        //~ GATING
        SIM->SCGC2 |= SIM_SCGC2_FLEXIO_MASK ;
        //SIM->SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK  ;
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK  ;
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK  ;
        
        
        //~ FLEXIO SETUP
        FLEXIO0->CTRL |= FLEXIO_CTRL_FLEXEN_MASK;
        
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
        
        //~ IO & CLOCK SETUP
        PORTC->PCR[ 3] = PORT_PCR_MUX(5); /// CLOCKOUT
        PORTC->PCR[ 8] = PORT_PCR_MUX(1); /// CAM RESET
        PORTC->PCR[ 9] = PORT_PCR_MUX(1); /// POWER DOWN
        
        GPIOC->PDDR   |= GPIO_PDDR_PDD(8);
        GPIOC->PDDR   |= GPIO_PDDR_PDD(9);
        
        SIM->SOPT2    |= SIM_SOPT2_CLKOUTSEL(7); /// MCGIRCLK - I THINK ITS THE SAME AS SYSTEM CLOCK FREQ
        
        // ****************************************
        // OV7670 SETUP
        // ****************************************
#endif
    }
    
    printf("Core Frequency: %d \n\n\r", SystemCoreClock);
    printf("\n\n\r");
    
    u32 counter = 0;
    u8  result  = 0;
    
    while(RUNNING)
    {
        //printf("Core Frequency: %d \n\n\r", SystemCoreClock);
        if(counter == 1000000/2)
        {
            /// TOGGLE LED ON
            GPIOC->PCOR |= LED_GREEN;
            
            {//- I2C TESTING
                
#ifdef test_camera
                //I2C3->FLT &= ~I2C_FLT_STOPF_MASK;
                //I2C_write_byte         (OV7670_SLAVE_ADDRESS, OV7670_REG_PID, 0x1);
                //result = I2C_read_byte_simple (OV7670_SLAVE_ADDRESS, OV7670_REG_COM7);
                I2C_write_byte         (OV7670_SLAVE_ADDRESS, OV7670_REG_PID, 0x1);
                printf("I2C read value: %#2X \n\r", (u32)result);
#endif
                
                I2C_debug_log_status();
            }//-
        }
        else if (counter == 1000000)
        {
            /// TOGGLE LED OFF
            GPIOC->PSOR |= LED_GREEN;
            counter = 0;
        }
        counter++;
    }
}

