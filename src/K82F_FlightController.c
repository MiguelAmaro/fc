#include "MK82F25615.h"
#include "LAL.h"
#include "FlightController_System.h"
#include "FlightController_Debug.h"
#include "FlightController_Motor.h"
#include "FlightController_OnboardLEDs.h"

#define RUNNING (1)
#define DASSERT(Expression) if(!(Expression)){GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

// NOTE(MIGUEL): DUE TO PIN SHARING MODULES MUST BE TESTED INDIVIDUALLY
// TODO(MIGUEL): CREATE A PIN MUTUAL EXCLUSION SYSTEM
// TODO(MIGUEL): CHANGE SYSTEM FREQUENCY

int 
main(void) 
{
    
    Debug_init_uart(115200);
    OBLEDs_init();
    Motor_init ();
    
    printf("Core Frequency: %d \n\n\r", SystemCoreClock);
    
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
    printf("Bus  Frequency: %d \n\n\r" ,  (SystemCoreClock * (0x01U + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)) ) /  (0x01U + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT)));
    
    for(u32 i = 0; i < 17; i++)
        Motor_log_dma_buffers();
    //ARM MOTORS
    //Motor_dshot_packet_create(1000);
    //Motor_dshot_packet_send();
    
    printf("\n\n\r");
    readonly u32 counter_max = 2000/5.3;
    u32 counter     = 0;
    u32 led_counter = 0;
    u32 is_armed    = 0;
    u32 sp_data     = 0;
    while(RUNNING)
    {
        if(!RingBuffer_Empty(&receiveQueue))
        {
            sp_data = (u32)RingBuffer_Dequeue_Byte(&receiveQueue);
        }
        //printf("Core Frequency: %d \n\n\r", SystemCoreClock);
        if(counter == counter_max/2)
        {
            if(led_counter == 100/2)
            {
                /// TOGGLE LED ON
                GPIOC->PCOR |= LED_BLUE;
                
                //Motor_log_dma_buffers();
                //for(u32 i = 0; i < 17; i++)
                //Motor_log_dma_buffers();
            }
            printf("RC: %d\n\r", (u32)sp_data);
            
            if(led_counter < 1 && !is_armed)
            {
                Motor_dshot_packet_create(48);
                Motor_dshot_packet_send();
            }
            else if(led_counter < 2 && !is_armed)
            {
                Motor_dshot_packet_create(0);
                Motor_dshot_packet_send();
            }
            else
            {
                is_armed = 1;
                //Motor_dshot_packet_create(48 + ( ( sp_data/255 ) * 2000 ));
                Motor_dshot_packet_create(48 + sp_data);
                Motor_dshot_packet_send();
            }
            
        }
        else if (counter == counter_max)
        {
            //Motor_dshot_packet_send();
            if(led_counter == 100)
            {
                /// TOGGLE LED OFF
                GPIOC->PSOR |= LED_BLUE;
                GPIOC->PSOR |= LED_RED;
                GPIOC->PSOR |= LED_GREEN;
                led_counter = 0;
            }
            led_counter++;
            counter = 0;
        }
        counter++;
    }
}


void
throwaway(void)
{
    
    //FTM0->SC    = FTM_SC_TOIE(0) | FTM_SC_CLKS(1) | FTM_SC_PS(0); //START TIMER
    //FTM0->SC    = FTM_SC_TOIE(0) | FTM_SC_CLKS(0) | FTM_SC_PS(0); //STOP  TIMER
    // NOTE(MIGUEL): the 0 !!!!means channel 0!!!! not bit 0
    //DMA0->SSRT |= DMA_SSRT_SSRT(0); //Set Start bit in TCD control status register -starts minor loop
    //Motor_display_dma_status_errors();
    
    /// TIMER TESTING
    //printf("FTM0 CH2 counter should be: %#2X \n\r", global_Dshot_command_buffer[0]);
    //printf("FTM0 CH2 conuter is       : %#2X \n\r", FTM0->CONTROLS[2].CnV);
    
    return;
}

