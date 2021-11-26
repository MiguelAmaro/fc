#include "MK82F25615.h"
#include "LAL.h"

#include "fc_system.h"
#include "fc_debug.h"
#include "fc_motor.h"
#include "fc_onboardleds.h"
#include "fc_i2c.h"
#include "fc_telemtry.h"
#include "fc_ecompass.h"

#define RUNNING (1)
#define DASSERT(Expression) if (!(Expression){ GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

// NOTE(MIGUEL): DUE TO PIN SHARING MODULES MUST BE TESTED INDIVIDUALLY
// TODO(MIGUEL): CREATE A PIN MUTUAL EXCLUSION SYSTEM
// TODO(MIGUEL): CHANGE SYSTEM FREQUENCY


u32 g_McuStateIndicator = LED_GREEN;

#define TEST_CAMERA     0
#define TEST_MOTOR      0
#define TEST_ECOMPASS   0
#define ENABLE_MAINLOOP 0

//~ 0V7670 DEFINITIONS
#define OV7670_SLAVE_ADDRESS (0x42)
#define OV7670_REG_VREF      (0x03)
#define OV7670_REG_PID       (0x0A)
#define OV7670_REG_VER       (0x0B)
#define OV7670_REG_COM7      (0x12)
#define OV7670_REG_COM8      (0x13)

int main(void)
{
    __disable_irq();
    // NOTE(MIGUEL): set desired clock (state)FLL ENGAGED EXTERNAL(FBE)
    SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV4(1); //flash clk divider: div by 2
    SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1); //bus clk divider: div by 2
    MCG->C7      |= MCG_C7_OSCSEL(2); //IRC48M internal osc
    MCG->C1      |= MCG_C1_CLKS  (2);   //bypass FLL & use external clk src dircetly
    
#if 1
    // NOTE(MIGUEL): systic
	SysTick->CTRL |= 0;       //  Disabls SysTick
    SysTick->LOAD = 48000000L/1;
	
	NVIC_SetPriority(SysTick_IRQn, 3); // Set the interrupt priority
	NVIC_EnableIRQ  (SysTick_IRQn);
    
	SysTick->CTRL |= 2;       // Enable the SysTick interrupt
	SysTick->CTRL |= 5;       // Set the Clock and Enable the down counter
#endif
    
    Debug_InitUart(115200);
    OBLEDs_Init();
    Motor_Init();
    
    printf("Core Frequency: %d \n\n\r", query_system_clock());
    
    // ************ ECOMPASS CONTROL *************
    {
#if TEST_ECOMPASS
        // ****************************************
        // I2C SETUP
        // ****************************************
        
        I2C_Init(2, 0x11);
        //I2C_scanner();
        // ****************************************
        // FXOS8700CQ SETUP
        // ****************************************
        // PTA2/ I2C3_SCL
        // PTA1/ I2C3_SDA
        // PTC13
        
        // Enable PORTC
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        
        // Config PTC Pins
        PORTC->PCR[13] = PORT_PCR_MUX(1); /// FXOS8700CQ Interrupt
        GPIOC->PDDR |= GPIO_PDDR_PDD(13);
        GPIOC->PSOR |= GPIO_PSOR_PTSO(13);
        printf("initing ecompass\n\r");
        Ecompass_init((void *)0x00);
        printf("initing ecompass done\n\r");
        
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
#if TEST_CAMERA
#if !TEST_ECOMPASS
        I2C_init(2, 0x11);
#endif
        // ****************************************
        // FLEXIO SETUP
        // ****************************************
        // TODO(MIGUEL): Check CLOCKOUT with o-scope
        
        //~ GATING
        SIM->SCGC2 |= SIM_SCGC2_FLEXIO_MASK;
        //SIM->SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        
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
        PORTC->PCR[3] = PORT_PCR_MUX(5); /// CLOCKOUT
        PORTC->PCR[8] = PORT_PCR_MUX(1); /// CAM RESET
        PORTC->PCR[9] = PORT_PCR_MUX(1); /// POWER DOWN
        
        GPIOC->PDDR |= GPIO_PDDR_PDD(8);
        GPIOC->PDDR |= GPIO_PDDR_PDD(9);
        
        SIM->SOPT2 |= SIM_SOPT2_CLKOUTSEL(7); /// MCGIRCLK - I THINK ITS THE SAME AS SYSTEM CLOCK FREQ
        
        // ****************************************
        // OV7670 SETUP
        // ****************************************
#endif
    }
    
    printf("Core Frequency: %d \n\n\r", query_system_clock());
    printf("Bus  Frequency: %d \n\n\r", query_bus_clock() );
#if 0
    for (u32 i = 0; i < 17; i++)
        Motor_log_dma_buffers();
#endif
    //ARM MOTORS
    //Motor_dshot_packet_create(1000);
    //Motor_dshot_packet_send();
    
    printf("\n\n\r");
    readonly u32 CounterMax = (u32)(2000 / 5.3);
    u32 Counter = 0;
    u32 LedCounter = 0;
    u32 IsArmed = 0;
    u32 SPData = 0; // NOTE(MIGUEL): serial port 
    
    __enable_irq();
    // NOTE(MIGUEL): clock speed changed 48Mhz loop speed affected & aswell as some modules
#if ENABLE_MAINLOOP
    while (RUNNING)
    {
        SPData = (u32)RingBuffer_Dequeue_Byte(&receiveQueue);
        
        //printf("Core Frequency: %d \n\n\r", SystemCoreClock);
        if (Counter == CounterMax / 2)
        {
            if (LedCounter == 100 / 2)
            {
                /// TOGGLE LED ON
                //GPIOC->PCOR |= LED_BLUE;
                
                //Motor_log_dma_buffers();
                //for(u32 i = 0; i < 17; i++)
                //Motor_log_dma_buffers();
            }
            
#if TEST_MOTOR
            //printf("RC: %d\n\r", (u32)sp_data);
            if (LedCounter < 1 && !IsArmed)
            {
                Motor_CreateDshotPacket(0);
                Motor_SendDshotPacket();
            }
            else if (LedCounter < 2 && !IsArmed)
            {
                Motor_CreateDshotPacket(48);
                Motor_SendDshotPacket();
            }
            else
            {
                IsArmed = 1;
                f32 ThrottleNormalized = 0.0f + ((f32)SPData / (f32)255.0f );
                u16 Throttle = (u16)(ThrottleNormalized * 2000 );
                Motor_CreateDshotPacket(48 + Throttle);
                //Motor_dshot_packet_create(48 + sp_data);
                Motor_SendDshotPacket();
                printf("throttle: %d \n\r", Throttle);
            }
#endif
            
            /// TOGGLE LED ON
            //GPIOC->PCOR |= LED_GREEN;
            
            //- I2C TESTING
            { 
#if TEST_CAMERA
                I2C_WriteByte             (OV7670_SLAVE_ADDRESS, OV7670_REG_PID, 0x1);
                u8 Result = I2C_ReadByte (OV7670_SLAVE_ADDRESS, OV7670_REG_COM7);
                
                I2C_WriteByte(OV7670_SLAVE_ADDRESS, OV7670_REG_PID, 0x1);
                printf("I2C read value: %#2X \n\r", (u32)Result);
                
                I2C_DebugLogStatus();
#endif
                
#if TEST_ECOMPASS
                //Ecompass_print_debug_info();
#endif
                
            } 
            //-
        }
        else if (Counter == CounterMax)
        {
            //Motor_dshot_packet_send();
            if (LedCounter == 100)
            {
                /// TOGGLE LED OFF
                GPIOC->PSOR |= LED_BLUE;
                //GPIOC->PSOR |= LED_RED;
                GPIOC->PSOR |= LED_GREEN;
                LedCounter = 0;
            }
            LedCounter++;
            
            Counter = 0;
        }
        Counter++;
    }
#else
    telem_state TelemState = Telem_NoConnection;
    
    while(1)
    {
        /*
        switch(TelemState)
        {
            case Telem_NoConnection:
            {
                
                Telemetry_HandShaked(Telem_Data, Telem_str8, "hello world", sizeof("hello world"));
            } break;
            case Telem_Waiting:
            {
                
                Telemetry_PackageAndSend(Telem_Data, Telem_str8, "hello world", sizeof("hello world"));
            } break;
            case Telem_Ack:
            {
                
                Telemetry_PackageAndSend(Telem_Data, Telem_str8, "hello world", sizeof("hello world"));
            } break;
        }*/
    }
#endif
}

b32 SysTicked = 0;
void SysTick_Handler(void)
{
    if(SysTicked)
    {
        GPIOC->PSOR |= g_McuStateIndicator;
    }
    else
    {
        GPIOC->PCOR |= g_McuStateIndicator;
        Telemetry_PackageAndSend(Telem_Data, Telem_str8, "blow me", sizeof("blow me"));
    }
    
    SysTicked= !SysTicked;
    
    return;
}

void throwaway(void)
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


