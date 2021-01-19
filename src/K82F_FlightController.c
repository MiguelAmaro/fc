#include "MK82F25615.h"
#include "LAL.h"
#include "FlightController_Debug.h"
#include "FlightController_OnboardLEDs.h"

#define RUNNING (1)
#define DASSERT(Expression) if(!(Expression)){GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

void FTM0_IRQHandler(void)
{
    
    return;
}

#define DSHOT_CMD_BUFFER_SIZE (256)
u8  Dshot_command_buffer[DSHOT_CMD_BUFFER_SIZE];
u16 Dshot_last_command = 48;

int 
main(void) 
{
    Debug_init_uart(115200);
    OBLEDs_init();
    
    // ************ ECOMPASS CONTROL *************
    {
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
        
        
    }
    // ************ RADIO CONTROL *************
    {
        // ****************************************
        // SPI SETUP
        // ****************************************
        
        
        // ****************************************
        // NRF24L01P SETUP
        // ****************************************
        
    }
    // ************ MOTOR CONTROL *************
    {
        /*
        // ****************************************
        // FLEXTIMER SETUP [FTM0 | Ch1 DMA | Ch2 PULSE] - pg.1442
        // ****************************************
        SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
        // NOTE(MIGUEL): what is the correct polarity?
        FTM0->SC    = 0x00U;
        FTM0->CNT   = 0x00U;
        u32 mod     = (BUS_CLOCK + DSHOT_CLOCK / 2) / DSHOT_CLOCK;
        FTM0->MOD   = (mod - 1);
        FTM0->SC    = FTM_SC_CLKS(1) | FTM_SC_PS(0);
        FTM0->MODE |= FTM_MODE_WPDIS_MASK;
        // NOTE(MIGUEL): Maybe confige should be more explicit [DECAPEN=0,COMBINE=0,CPWMS=0,MSNB=0]
        FTM0->CONTROLS[1].CnSC = FTM_CnSC_DMA_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK;
        FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
        FTM0->CONTROLS[1].CnV  = ((mod * 250) >> 8);
        FTM0->CONTROLS[2].CnV  = 0;
        
        
        // ****************************************
        // DMA SETUP - pg. 539
        // ****************************************
        // Two major components DMA enging & Transfer Control Descriptor(TDC)
        // DMA ENGINE: - pg. 541
        //     * Adress Path Block
        //     * Data   Path Block
        //     * Prog Model/Ch Arbitration Block - Resolves Conflicts
        //     * Control     Block
        // TRANSFER CONTROL DESCRIPTOR: - pg. 541
        //     * Memrory Conroller Block
        //     * Memrory Array     Block
        // eDMA BASIC DATA FLOW / OPERATION - 621
        
        // TODO(MIGUEL): Config Control Reg
        DMA0->CR;
        // TODO(MIGUEL): Config Priority Reg
        DMA0->DCHPRI0;
        // TODO(MIGUEL): Enable Error Interrupts
        DMA0->EEI;
        // TODO(MIGUEL): Initialize Transfer Control Descriptor
        //DMA0->TCD[0].CITER; 
        DMA0->TCD[0].NBYTES_MLNO = 1; 
        DMA0->TCD[0].SADDR       = (u32 *)Dshot_command_buffer;
        DMA0->TCD[0].SOFF;
        DMA0->TCD[0].ATTR[SRC_SIZE];
        DMA0->TCD[0].SLAST;
        DMA0->TCD[0].DADDR = FTM0->CONTROLS[2].CnV;
        DMA0->TCD[0].DOFF;
        DMA0->TCD[0].ATTR[DEST_SIZE];
        DMA0->TCD[0].DLAST_SGA;
        
        // TODO(MIGUEL): Enable Hardware Service Requests
        
        // TODO(MIGUEL): Requset channel service
        // NOTE(MIGUEL): FTM probably has hardware dma signal capabilities
        */
        
    }
    {
        
        // ****************************************
        // FLEXIO SETUP
        // ****************************************
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        
        // NOTE(MIGUEL): PTA1 & PTA2 are shared with ECompass
        PORTA->PCR[ 1] = PORT_PCR_MUX ( 1); /// SDATA
        PORTA->PCR[ 2] = PORT_PCR_MUX ( 1); /// SCLOCK
        
        PORTB->PCR[ 0] = PORT_PCR_MUX(7); /// FXIO_DATA__0 - CAM_PCLK
        PORTB->PCR[ 2] = PORT_PCR_MUX(7); /// FXIO_DATA__2 - CAM_VSYNC
        PORTB->PCR[ 3] = PORT_PCR_MUX(7); /// FXIO_DATA__3 - CAM_HREF
        PORTB->PCR[10] = PORT_PCR_MUX(7); /// FXIO_DATA__4 - CAM_DATA_0
        PORTB->PCR[11] = PORT_PCR_MUX(7); /// FXIO_DATA__5 - CAM_DATA_1
        PORTB->PCR[18] = PORT_PCR_MUX(7); /// FXIO_DATA__6 - CAM_DATA_2
        PORTB->PCR[19] = PORT_PCR_MUX(7); /// FXIO_DATA__7 - CAM_DATA_3
        PORTB->PCR[21] = PORT_PCR_MUX(7); /// FXIO_DATA__9 - CAM_DATA_5
        PORTB->PCR[22] = PORT_PCR_MUX(7); /// FXIO_DATA__6 - CAM_DATA_2
        PORTB->PCR[23] = PORT_PCR_MUX(7); /// FXIO_DATA_11 - CAM_DATA_7
        
        PORTC->PCR[ 3] = PORT_PCR_MUX(5); /// CLOCKOUT
        PORTC->PCR[ 8] = PORT_PCR_MUX(1); /// CAM RESET
        PORTC->PCR[ 9] = PORT_PCR_MUX(1); /// POWER DOWN
        GPIOC->PDDR |= GPIO_PDDR_PDD (8);
        GPIOC->PDDR |= GPIO_PDDR_PDD (9);
        // ****************************************
        // OV7670 SETUP
        // ****************************************
    }
    
    
    u32 counter = 0;
    
    while(RUNNING)
    {
        printf("Core Frequency: %d \n\r", SystemCoreClock);
        
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

