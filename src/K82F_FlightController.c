#include "MK82F25615.h"
#include "LAL.h"
#include "FlightController_System.h"
#include "FlightController_Debug.h"
#include "FlightController_OnboardLEDs.h"

#define RUNNING (1)
#define DASSERT(Expression) if(!(Expression)){GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

#define DSHOT_CMD_BUFFER_SIZE (16 + 1)
global volatile u16  global_Dshot_command_buffer[DSHOT_CMD_BUFFER_SIZE];
global volatile u16  global_Dshot_dma_recieve_test;
//global u16 Dshot_last_command = 48;

volatile u32 dma_test_src [4]  = { 0 };
volatile u32 dma_test_dest[4] = { 0 }; 

int 
main(void) 
{
    
    Debug_init_uart(115200);
    OBLEDs_init();
    
    printf("Core Frequency: %d \n\n\r", SystemCoreClock);
    
    // NOTE(MIGUEL): DUE TO PIN SHARING MODULES MUST BE TESTED INDIVIDUALLY
    // TODO(MIGUEL): CREATE A PIN MUTUAL EXCLUSION SYSTEM
    // TODO(MIGUEL): CHANGE SYSTEM FREQUENCY
    
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
    
    // ************ MOTOR CONTROL *************
    {
        //------------------ SETUP PACKET ---------------------
#define DSHOT_0_TIMING  (u32)(0.38 * 255)
#define DSHOT_1_TIMING  (u32)(0.75 * 255)
#define DSHOT_CLOCK          (300)
        
        volatile u32 mod     = (u32)((BUS_CLOCK + DSHOT_CLOCK / 2) / DSHOT_CLOCK);
        
        volatile u16 packet   = 0;
        volatile u8  checksum = 0;
        
        // FRAME THE PACKET
        packet = 1000 << 1;
        
        volatile u16 temp_packet = packet;
        // Checksum
        for(u32 i = 0; i < 3; i++)
        {
            checksum     ^= temp_packet;
            temp_packet >>=           4; 
        }
        
        checksum &= 0xf; 
        
        packet = (u16)(packet << 4) | checksum;
        
        // TRANSFORM PACKET DATA & FILL CMD BUFFER
        // NOTE(MIGUEL): Is this correctlly represented in memory?
        // TODO(MIGUEL): CALCULATE CORRECT PACKET VALUES 
        for(u32 i = 0; i < DSHOT_CMD_BUFFER_SIZE; i++)
        {
            if((1 << i) & packet)
            {
                //printf("mod value: %d * %d  = %d \n\r", mod, DSHOT_1_TIMING, mod * DSHOT_1_TIMING);
                global_Dshot_command_buffer[16 - i] = (u8)((mod * DSHOT_1_TIMING) >> 8); // pack buffer MSB first (expression >> 8 ) = (expression / 2^8)
                //global_Dshot_command_buffer[16U - i] = (u8)('A' + (16 - i));
            }
            else
            {
                //printf("mod value: %d \n\r", mod);
                global_Dshot_command_buffer[16 - i] = (u8)((mod * DSHOT_0_TIMING) >> 8) ; // pack buffer MSB first  (expression >> 8 ) = (expression / 2^8)
                //global_Dshot_command_buffer[16U - i] = (u8)('A' + (16 - i));
            }
            //%#2X
            printf("i = %d | PWM CMD VALUES: %#2X \n\r", i, global_Dshot_command_buffer[16 - i]);
        }
        
        
        
        
        
        // Enable DMAMUX
        
        SIM->SCGC6 = SIM_SCGC6_DMAMUX_MASK;
        DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_TRIG  ( 0); /// No periodic tirggering
        DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(21); /// FTM0CH1
        DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_ENBL  ( 1);
        
        //printf()
        // ****************************************
        // DMA SETUP - pg. 539
        // ****************************************
        
        SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
        // TODO(MIGUEL): WHY IS FTM BROKEN AFTER CONFIGURING DMA????
        // TODO(MIGUEL): Config Control Reg
        //DMA0->CR;
        //DMA0->CERQ |= DMA_CERQ_CERQ(0); /// 0 = ch! Disable Hardware DMA Requests
        //DMA0->SERQ = DMA_SERQ_SERQ(0); /// 0 = ch! Enable Hardware DMA Requests
        // TODO(MIGUEL): Config Priority Reg
        //DMA0->DCHPRI0;
        // TODO(MIGUEL): Enable Error Interrupts
        DMA0->SEEI = DMA_SEEI_SEEI(0); // NOTE(MIGUEL): Might be useful for debuging
        
        /// Transfer Control Descriptor
        DMA0->TCD[0].CSR |= DMA_CSR_DREQ_MASK;
        DMA0->TCD[0].CSR &= ~(DMA_CSR_ESG_MASK | DMA_CSR_START_MASK | DMA_CSR_ACTIVE_MASK | DMA_CSR_DONE_MASK); // Disable Scatter Gather
        // NOTE(MIGUEL): Only doing 1 Major Iteration so Current Iter and Begin Iter is set to 1
        DMA0->TCD[0].CITER_ELINKNO = DMA0->TCD[0].BITER_ELINKNO = 1; /// Set Current and Beining Majorloop counter/Iterator value
        DMA0->TCD[0].NBYTES_MLNO   = DMA_NBYTES_MLNO_NBYTES(16);      /// Total number of bytes to transfer on each minor loop
        
        ///TEST DMA CONFIG
        /// Settings for reading from the source
        DMA0->TCD[0].SADDR = DMA_SADDR_SADDR((u32)&dma_test_src); /// source address
        DMA0->TCD[0].SOFF  = DMA_SOFF_SOFF (1); /// the offset(in bytes) to add to src address after each read of the source
        DMA0->TCD[0].ATTR  = DMA_ATTR_SSIZE(0); /// number of bits to fetch on each read (0 = 8bits)
        DMA0->TCD[0].SLAST = -16;               /// offset(in bytes) to add to the src address after the major loop completes - can use this to reset src address to initial value 
        
        /// Settings for writting to the destination
        DMA0->TCD[0].DADDR = DMA_DADDR_DADDR((u32)&dma_test_dest); /// destination address (u32)&(FTM0->CONTROLS[2].CnV)
        DMA0->TCD[0].DOFF  = DMA_DOFF_DOFF (4); /// the offset(in bytes) to add to the dest address after each write to the destination
        DMA0->TCD[0].ATTR  = DMA_ATTR_DSIZE(2); /// number of bits to store on each write (0 = 16bits)
        DMA0->TCD[0].DLAST_SGA = -16;           /// offset to add to the dest address after the major loop completes - use this to reset dest address to initial value 
        //DMA0->TCD[0].CSR  |= DMA_CSR_INTMAJOR_MASK; /// Interupt on major loop completion
        
        // NOTE(MIGUEL): Same priority level as LPUART4 
        NVIC_SetPriority    (DMA_Error_IRQn, 2); // 0, 1, 2, or 3
        NVIC_ClearPendingIRQ(DMA_Error_IRQn   ); 
        NVIC_EnableIRQ      (DMA_Error_IRQn   );
        
        // Verify Source addres is correct
        printf("DMA source address: 0x%p \n\r", &global_Dshot_command_buffer);
        printf("DMA source address: %#2x \n\r",   DMA0->TCD[0].SADDR);
        printf("DMA dest   address: 0x%p \n\r", &(FTM0->CONTROLS[2].CnV));
        printf("DMA dest   address: %#2x \n\r",   DMA0->TCD[0].DADDR);
        
        
        // TODO(MIGUEL): Enable Hardware Service Requests
        
        // TODO(MIGUEL): Requset channel service
        // NOTE(MIGUEL): FTM probably has hardware dma signal capabilities
        
        
        // TODO(MIGUEL): Calc correct value for DSHOT_CLOCK
        // ****************************************
        // FLEXTIMER SETUP [FTM0 | Ch1 DMA | Ch2 PULSE] - pg.1442
        // ****************************************
        
        SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK ;
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        
        PORTC->PCR[3] = PORT_PCR_MUX(4); /// FTM0CH2
        printf("FLL eXTernAL oscillator %#2X \n\r", (MCG->C7 & MCG_C7_OSCSEL_MASK));
        printf("mod: %d \n\r", mod);
        // FLEXTIMER0 SETUP
        // NOTE(MIGUEL): COMBINE config not needed
        // NOTE(MIGUEL): COMBINE config not needed
        
        FTM0->MODE |= FTM_MODE_WPDIS_MASK;
        FTM0->CONF  = FTM_CONF_BDMMODE_MASK; // BDM field to 0x11
        FTM0->FMS   = 0x00; 
        // NOTE(MIGUEL): what is the correct polarity?
        FTM0->SC    = 0x00U; /// Active-low
        FTM0->CNT   = 0x00U; /// Initial conter value
        // NOTE(MIGUEL): What should DSHOT_CLOCK be defineed as
        printf("FTM0 modulo: %d \n\n\r", mod);
        FTM0->MOD   = (mod - 1);
        
        // CHANNEL SETUP
        // DMA trigger
        // NOTE(MIGUEL): Maybe confige should be more explicit [DECAPEN=0,COMBINE=0,CPWMS=0,MSNB=0]
        FTM0->CONTROLS[1].CnSC = (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK) & ~FTM_CnSC_ELSA_MASK;
        FTM0->CONTROLS[1].CnV  = 1000; //((mod * 210) >> 8); // (x >> 8) = (x / 2^8)
        // Packet pulse
        FTM0->CONTROLS[2].CnSC = (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK) & ~FTM_CnSC_ELSA_MASK;
        // NOTE(MIGUEL): 16bit Counter - Max Counter 65536
        FTM0->CONTROLS[2].CnV  = 2000; /// Controls Pulse width - is DMA destination
        
        
        // NOTE(MIGUEL): Same priority level as LPUART4 
        NVIC_SetPriority    (FTM0_IRQn, 2); // 0, 1, 2, or 3
        NVIC_ClearPendingIRQ(FTM0_IRQn   ); 
        NVIC_EnableIRQ      (FTM0_IRQn   );
        
        
        //------------------ START TIMER ---------------------
        FTM0->SC    = FTM_SC_TOIE(0) | FTM_SC_CLKS(1) | FTM_SC_PS(0);
        
        // DMA should kick of the timer? NO
        
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
    
    printf("\n\n\r");
    u32 counter = 0;
    while(RUNNING)
    {
        //printf("Core Frequency: %d \n\n\r", SystemCoreClock);
        if(counter == 1000000/2)
        {
            //GPIOC->PSOR |= LED_GREEN;
            /// TOGGLE LED ON
            GPIOC->PCOR |= LED_BLUE;
            
            /// DMA TESTING
            
            DMA0->SERQ = DMA_SERQ_SERQ(0); /// 0 = ch! Enable Hardware DMA Requests
            
            // NOTE(MIGUEL): the 0 !!!!means channel 0!!!! not bit 0
            //DMA0->SSRT |= DMA_SSRT_SSRT(0); //Set Start bit in TCD control status register
            printf("HRS: %#2X \n\r", DMA0->HRS & DMA_HRS_HRS0_MASK);
            printf("VLD: %#2X \n\r", DMA0->ES  & DMA_ES_VLD_MASK);
            printf("ECX: %#2X \n\r", DMA0->ES  & DMA_ES_ECX_MASK);
            printf("GPE: %#2X \n\r", DMA0->ES  & DMA_ES_GPE_MASK);
            printf("CPE: %#2X \n\r", DMA0->ES  & DMA_ES_CPE_MASK);
            printf("SAE: %#2X \n\r", DMA0->ES  & DMA_ES_SAE_MASK);
            printf("SOE: %#2X \n\r", DMA0->ES  & DMA_ES_SOE_MASK);
            printf("DAE: %#2X \n\r", DMA0->ES  & DMA_ES_DAE_MASK);
            printf("DOE: %#2X \n\r", DMA0->ES  & DMA_ES_DOE_MASK);
            printf("NCE: %#2X \n\r", DMA0->ES  & DMA_ES_NCE_MASK);
            printf("SGE: %#2X \n\r", DMA0->ES  & DMA_ES_SGE_MASK);
            printf("SBE: %#2X \n\r", DMA0->ES  & DMA_ES_SBE_MASK);
            printf("DBE: %#2X \n\r", DMA0->ES  & DMA_ES_DBE_MASK);
            printf("DMACH0E: %#2X \n\r", DMA0->ERR & DMA_ERR_ERR0_MASK);
            // DSHOT DMA TESTING
            //printf("Dshot Buffer[0] is: %#2X \n\r", global_Dshot_command_buffer[0]);
            //printf("Dshot Buffer recieve test: %#2X \n\r", global_Dshot_dma_recieve_test);
            
            
            dma_test_src[0] =   0x01;
            dma_test_src[1] =   0x20;
            dma_test_src[2] =  0x400;
            dma_test_src[3] = 0x8000;
            
            // BASE CASE DMA TESTING
            printf("DMA src  value is: %#2X \n\r", dma_test_src [0]);
            printf("DMA dest value is: %#2X \n\r", dma_test_dest[0]);
            printf("DMA dest value is: %#2X \n\r", dma_test_dest[1]);
            printf("DMA dest value is: %#2X \n\r", dma_test_dest[2]);
            printf("DMA dest value is: %#2X \n\r", dma_test_dest[3]);
            
            /// TIMER TESTING
            
            //printf("FTM0 CH2 counter should be: %#2X \n\r", global_Dshot_command_buffer[0]);
            //printf("FTM0 CH2 conuter is       : %#2X \n\r", FTM0->CONTROLS[2].CnV);
            printf("FTMCH1 : %#2X \n\r", FTM0->STATUS & FTM_STATUS_CH1F_MASK);
            printf("FTMCH2 : %#2X \n\r", FTM0->STATUS & FTM_STATUS_CH2F_MASK);
            
            printf("\n\n\r");
        }
        else if (counter == 1000000)
        {
            /// STOP DMA
            DMA0->CERQ |= DMA_CERQ_CERQ(0); /// 0 = ch! Disable Hardware DMA Requests
            
            /// TOGGLE LED OFF
            GPIOC->PSOR |= LED_BLUE;
            GPIOC->PSOR |= LED_RED;
            GPIOC->PSOR |= LED_GREEN;
            counter = 0;
        }
        counter++;
    }
}

void
DMA_Error_IRQHandler(void)
{
    GPIOC->PCOR |= LED_GREEN;
    
    // TODO(MIGUEL): Disable DMA so this doesn't block
    
    printf("HRS: %#2X \n\r", DMA0->HRS & DMA_HRS_HRS0_MASK);
    printf("VLD: %#2X \n\r", DMA0->ES  & DMA_ES_VLD_MASK);
    printf("ECX: %#2X \n\r", DMA0->ES  & DMA_ES_ECX_MASK);
    printf("GPE: %#2X \n\r", DMA0->ES  & DMA_ES_GPE_MASK);
    printf("CPE: %#2X \n\r", DMA0->ES  & DMA_ES_CPE_MASK);
    printf("SAE: %#2X \n\r", DMA0->ES  & DMA_ES_SAE_MASK);
    printf("SOE: %#2X \n\r", DMA0->ES  & DMA_ES_SOE_MASK);
    printf("DAE: %#2X \n\r", DMA0->ES  & DMA_ES_DAE_MASK);
    printf("DOE: %#2X \n\r", DMA0->ES  & DMA_ES_DOE_MASK);
    printf("NCE: %#2X \n\r", DMA0->ES  & DMA_ES_NCE_MASK);
    printf("SGE: %#2X \n\r", DMA0->ES  & DMA_ES_SGE_MASK);
    printf("SBE: %#2X \n\r", DMA0->ES  & DMA_ES_SBE_MASK);
    printf("DBE: %#2X \n\r", DMA0->ES  & DMA_ES_DBE_MASK);
    printf("DMACH0E: %#2X \n\r", DMA0->ERR & DMA_ERR_ERR0_MASK);
    
    return;
}

void
FTM0_IRQHandler(void)
{
    // NOTE(MIGUEL): software dma request from here not a feasable solution. main loop gets blocked
    //DMA0->SSRT |= DMA_SSRT_SSRT(0); //Set Start bit in TCD control status register
    //GPIOC->PCOR |= LED_RED;
    
    return;
}


