/* date = January 24th 2021 9:50 pm */

#ifndef FLIGHTCONTROLLER_MOTOR_H
#define FLIGHTCONTROLLER_MOTOR_H

#include "FlightController_OnboardLEDs.h"

#define DSHOT_CMD_BUFFER_SIZE (16 + 1)

//------------------ SETUP PACKET ---------------------
#define DSHOT_BAUD           (600000)
#define DSHOT_0_TIMING  (u32)(0.38 * 35)
#define DSHOT_1_TIMING  (u32)(0.75 * 35)

global volatile u16  global_Dshot_command_buffer[DSHOT_CMD_BUFFER_SIZE] = { 0 };
global volatile u16  global_Dshot_dma_recieve_test;
//global u16 Dshot_last_command = 48;

global volatile u32 dma_test_dest[4] = { 0 }; 
global volatile u32 dma_test_src [4] = { 0 };

global volatile u32 global_mod = (u32)((BUS_CLOCK) / DSHOT_BAUD );

typedef enum
{
    bits_8  = 0,
    bits_16 = 1,
    bits_32 = 2
} dma_size;


void
Motor_init(void)
{
    // Enable DMAMUX
    // ****************************************
    // DMAMUX SETUP
    // ****************************************
    SIM->SCGC6 = SIM_SCGC6_DMAMUX_MASK;
    DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_TRIG  ( 0); /// No periodic tirggering
    DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(21); /// FTM0CH1
    DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_ENBL  ( 1);
    
    // ****************************************
    // DMA SETUP - pg. 539
    // ****************************************
    //#define dma_test
    /*
What is the problem?
FTM is outputing the same pulse
DMA is likely writing the the same value into the FTM counter
I cant log out all the values tranferd to the FTM counter reg w/ HW dma req. no control  
what are the possibble cause???...
Likely that the TCD is broken
How can i can i get insight??
software dma req for testing 
dma irq w/ printf for controled log on maj loop completion
*/
    
    SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
    // TODO(MIGUEL): WHY IS FTM BROKEN AFTER CONFIGURING DMA????
    // TODO(MIGUEL): Config Control Reg
    //DMA0->CR;
    //DMA0->CERQ |= DMA_CERQ_CERQ(0); /// 0 = ch! Disable Hardware DMA Requests
    //DMA0->SERQ  = DMA_SERQ_SERQ(0); /// 0 = ch! Enable Hardware DMA Requests
    // TODO(MIGUEL): Config Priority Reg
    //DMA0->DCHPRI0;
    // TODO(MIGUEL): Enable Error Interrupts
    DMA0->SEEI = DMA_SEEI_SEEI(0); // NOTE(MIGUEL): Might be useful for debuging
    
    /// Transfer Control Descriptor
    DMA0->TCD[0].CSR |= DMA_CSR_DREQ_MASK; // Disable after 
    DMA0->TCD[0].CSR &= ~(DMA_CSR_ESG_MASK | DMA_CSR_START_MASK | DMA_CSR_ACTIVE_MASK | DMA_CSR_DONE_MASK); // Disable Scatter Gather
    // NOTE(MIGUEL): Only doing 1 Major Iteration so Current Iter and Begin Iter is set to 1
    DMA0->TCD[0].CITER_ELINKNO = DMA0->TCD[0].BITER_ELINKNO = 17; /// Set Current and Beining Majorloop counter/Iterator value
    DMA0->TCD[0].NBYTES_MLNO   = DMA_NBYTES_MLNO_NBYTES(2);       /// Total number of bytes to transfer on each minor loop
    
    ///TEST DMA CONFIG
    /// Settings for reading from the source
    DMA0->TCD[0].SADDR = DMA_SADDR_SADDR((u32)&global_Dshot_command_buffer); /// source address
    DMA0->TCD[0].SOFF  = DMA_SOFF_SOFF (1);      /// the offset(in bytes) to add to src address after each read of the source
    DMA0->TCD[0].ATTR  = DMA_ATTR_SSIZE(bits_8); /// number of bits to fetch on each read (0 = 8bits)
    DMA0->TCD[0].SLAST = -34;                    /// offset(in bytes) to add to the src address after the major loop completes - can use this to reset src address to initial value 
    
    /// Settings for writting to the destination
    //DMA0->TCD[0].DADDR = DMA_DADDR_DADDR((u32)&global_Dshot_dma_recieve_test); /// destination address
    DMA0->TCD[0].DADDR     = DMA_DADDR_DADDR((u32)&(FTM0->CONTROLS[2].CnV));     /// destination address
    DMA0->TCD[0].DOFF      = DMA_DOFF_DOFF  (0);                                 /// the offset(in bytes) to add to the dest address after each write to the destination
    DMA0->TCD[0].ATTR      = DMA_ATTR_DSIZE(bits_16); /// number of bits to store on each write (0 = 16bits)
    DMA0->TCD[0].DLAST_SGA = 0;                       /// offset to add to the dest address after the major loop completes - use this to reset dest address to initial value 
    DMA0->TCD[0].CSR      |= DMA_CSR_DREQ_MASK    ;  /// Auto clear Interupt Flag on major loop completion
    DMA0->TCD[0].CSR      |= DMA_CSR_INTMAJOR_MASK;  /// Interupt on major loop completion
    
    // NOTE(MIGUEL): Same priority level as LPUART4 
    NVIC_SetPriority    (DMA_Error_IRQn, 2); // 0, 1, 2, or 3
    NVIC_ClearPendingIRQ(DMA_Error_IRQn   ); 
    NVIC_EnableIRQ      (DMA_Error_IRQn   );
    /*
    
    
    */
    // NOTE(MIGUEL): Same priority level as LPUART4 
    NVIC_SetPriority    (DMA0_DMA16_IRQn, 2); // 0, 1, 2, or 3
    NVIC_ClearPendingIRQ(DMA0_DMA16_IRQn   ); 
    NVIC_EnableIRQ      (DMA0_DMA16_IRQn   );
    
    // Verify Source addres is correct
    printf("DMA source address: 0x%p \n\r", &global_Dshot_command_buffer);
    printf("DMA source address: %#2x \n\r",   DMA0->TCD[0].SADDR        );
    printf("DMA dest   address: 0x%p \n\r", &(FTM0->CONTROLS[2].CnV)    );
    printf("DMA dest   address: %#2x \n\r",   DMA0->TCD[0].DADDR        );
    
    // TODO(MIGUEL): Enable Hardware Service Requests
    
    // TODO(MIGUEL): Requset channel service
    // NOTE(MIGUEL): FTM probably has hardware dma signal capabilities
    
    
    // TODO(MIGUEL): Calc correct value for DSHOT_CLOCK
    // ****************************************
    // FLEXTIMER SETUP [FTM0 | Ch1 DMA | Ch2 PULSE] - pg.1442
    // ************************************* ***
    
    SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK ;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    
    PORTC->PCR[3] = PORT_PCR_MUX(4); /// FTM0CH2
    
    // FLEX TIMER 0 SETUP
    FTM0->MODE |= FTM_MODE_WPDIS_MASK  ;
    FTM0->CONF  = FTM_CONF_BDMMODE_MASK; // BDM field to 0x11
    FTM0->FMS   = 0x00 ; 
    FTM0->SC    = 0x00U; /// Active-low
    FTM0->CNT   = 0x00U; /// Initial conter value
    FTM0->MOD   = (global_mod - 1);
    
    // CHANNEL SETUP
    // DMA TRIGGER
    // NOTE(MIGUEL): Maybe confige should be more explicit [DECAPEN=0,COMBINE=0,CPWMS=0,MSNB=0]
    FTM0->CONTROLS[1].CnSC = (FTM_CnSC_MSB_MASK  | FTM_CnSC_ELSB_MASK | 
                              FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK) & ~FTM_CnSC_ELSA_MASK;
    FTM0->CONTROLS[1].CnV  = ((global_mod * 200) >> 8); // NOTE(MIGUEL): (x >> 8) = (x / 2^8)
    
    // PACKET BIT PULSE
    FTM0->CONTROLS[2].CnSC = (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK) & ~FTM_CnSC_ELSA_MASK;
    FTM0->CONTROLS[2].CnV  = 0; /// Controls Pulse width - is DMA destination
    
    
    // NOTE(MIGUEL): Same priority level as LPUART4 
    NVIC_SetPriority    (FTM0_IRQn, 3); // 0, 1, 2, or 3
    NVIC_ClearPendingIRQ(FTM0_IRQn   ); 
    NVIC_EnableIRQ      (FTM0_IRQn   );
    
    
    //------------------ START TIMER ---------------------
    FTM0->SC    = FTM_SC_TOIE(0) | FTM_SC_CLKS(1) | FTM_SC_PS(0);
    
    return;
}

void 
Motor_display_dma_status_errors(void)
{
    /*printf("HRS: %#2X \n\r"    , DMA0->HRS & DMA_HRS_HRS0_MASK);
    printf("VLD: %#2X \n\r"    , DMA0->ES  & DMA_ES_VLD_MASK);
    printf("ECX: %#2X \n\r"    , DMA0->ES  & DMA_ES_ECX_MASK);
    printf("GPE: %#2X \n\r"    , DMA0->ES  & DMA_ES_GPE_MASK);
    printf("CPE: %#2X \n\r"    , DMA0->ES  & DMA_ES_CPE_MASK);
    printf("SAE: %#2X \n\r"    , DMA0->ES  & DMA_ES_SAE_MASK);
    printf("SOE: %#2X \n\r"    , DMA0->ES  & DMA_ES_SOE_MASK);
    printf("DAE: %#2X \n\r"    , DMA0->ES  & DMA_ES_DAE_MASK);
    printf("DOE: %#2X \n\r"    , DMA0->ES  & DMA_ES_DOE_MASK);
    printf("NCE: %#2X \n\r"    , DMA0->ES  & DMA_ES_NCE_MASK);
    printf("SGE: %#2X \n\r"    , DMA0->ES  & DMA_ES_SGE_MASK);
    printf("SBE: %#2X \n\r"    , DMA0->ES  & DMA_ES_SBE_MASK);
    printf("DBE: %#2X \n\r"    , DMA0->ES  & DMA_ES_DBE_MASK);
    printf("DMACH0E: %#2X \n\r", DMA0->ERR & DMA_ERR_ERR0_MASK);
    */
    return;
}

void Motor_arm_esc(void)
{
    
    return;
}

void Motor_dshot_packet_send()
{
    /// RESET TIMER
    FTM0->SC   = FTM_SC_TOIE(0) | FTM_SC_CLKS(0) | FTM_SC_PS(0);
    FTM0->SC   = FTM_SC_TOIE(0) | FTM_SC_CLKS(1) | FTM_SC_PS(0);
    
    DMA0->SERQ = DMA_SERQ_SERQ(0); /// 0 = ch! Enable Hardware DMA Requests
    
    return;
}


void Motor_dshot_packet_create(u32 throttle)
{
    volatile u16 packet   = 0;
    volatile u8  checksum = 0;
    
    // FRAME THE PACKET
    packet = (u16)(throttle << 1);
    
    u16 temp_packet = packet;
    
    // Checksum
    for(u32 i = 0; i < 3; i++)
    {
        checksum     ^= temp_packet;
        temp_packet >>=           4; 
    }
    
    checksum &= 0xf; 
    
    packet = (u16)(packet << 4) | checksum;
    
    // NOTE(MIGUEL): Is this correctlly represented in memory?
    // TODO(MIGUEL): CALCULATE CORRECT PACKET VALUES 
    // TRANSFORM PACKET DATA & FILL CMD BUFFER
    for(u32 i = 0; i < DSHOT_CMD_BUFFER_SIZE; i++)
    {
        if((1 << i) & packet)
        {
            //printf("global_mod value: %d * %d  = %d \n\r", global_mod, DSHOT_1_TIMING, global_mod * DSHOT_1_TIMING);
            global_Dshot_command_buffer[15 - i] = (u8)((DSHOT_1_TIMING)); // pack buffer MSB first (expression >> 8 ) = (expression / 2^8)
            //global_Dshot_command_buffer[16U - i] = (u8)('A' + (16 - i));
        }
        else
        {
            //printf("global_mod value: %d \n\r", global_mod);
            
            global_Dshot_command_buffer[15 - i] = (u8)((DSHOT_0_TIMING)) ; // pack buffer MSB first  (expression >> 8 ) = (expression / 2^8)
            //global_Dshot_command_buffer[16U - i] = (u8)('A' + (16 - i));
        }
        //%#2X
        //printf("i = %d | PWM CMD VALUES: %#2X \n\r", i, global_Dshot_command_buffer[16 - i]);
    }
    
    return;
}


void
FTM0_IRQHandler(void)
{
    printf("%#2X \n\r", (u32)(FTM0->CONTROLS[2].CnV & FTM_CnV_VAL_MASK));
    FTM0->CONTROLS[2].CnSC &= ~FTM_CnSC_CHF_MASK;
    // NOTE(MIGUEL): software dma request from here not a feasable solution. main loop gets blocked
    //DMA0->SSRT |= DMA_SSRT_SSRT(0); //Set Start bit in TCD control status register
    //GPIOC->PCOR |= LED_RED;
    
    return;
}


void
DMA0_DMA16_IRQHandler(void)
{
    // TODO(MIGUEL): Figure out how to eliminate this interrupt 
    //NVIC_DisableIRQ      (DMA0_DMA16_IRQn   );
    // NOTE(MIGUEL): Are these usefull
    DMA0->CINT |= DMA_CINT_CINT(0); //Clear interrupt bit in TCD control status register
    //DMA0->CERQ |= DMA_CERQ_CERQ(0); /// 0 = ch! Disable Hardware DMA Requests
    //FTM0->CONTROLS[2].CnV  = 0;
    
    //printf("DMA Major loop complete \n\r");
    //printf("\n\r");
    
    return;
}

void
DMA_Error_IRQHandler(void)
{
    GPIOC->PCOR |= LED_GREEN;
    
    // TODO(MIGUEL): Disable DMA so this doesn't block
    
    /*printf(    "HRS: %#2X \n\r", DMA0->HRS & DMA_HRS_HRS0_MASK);
    printf(    "VLD: %#2X \n\r", DMA0->ES  & DMA_ES_VLD_MASK  );
    printf(    "ECX: %#2X \n\r", DMA0->ES  & DMA_ES_ECX_MASK  );
    printf(    "GPE: %#2X \n\r", DMA0->ES  & DMA_ES_GPE_MASK  );
    printf(    "CPE: %#2X \n\r", DMA0->ES  & DMA_ES_CPE_MASK  );
    printf(    "SAE: %#2X \n\r", DMA0->ES  & DMA_ES_SAE_MASK  );
    printf(    "SOE: %#2X \n\r", DMA0->ES  & DMA_ES_SOE_MASK  );
    printf(    "DAE: %#2X \n\r", DMA0->ES  & DMA_ES_DAE_MASK  );
    printf(    "DOE: %#2X \n\r", DMA0->ES  & DMA_ES_DOE_MASK  );
    printf(    "NCE: %#2X \n\r", DMA0->ES  & DMA_ES_NCE_MASK  );
    printf(    "SGE: %#2X \n\r", DMA0->ES  & DMA_ES_SGE_MASK  );
    printf(    "SBE: %#2X \n\r", DMA0->ES  & DMA_ES_SBE_MASK  );
    printf(    "DBE: %#2X \n\r", DMA0->ES  & DMA_ES_DBE_MASK  );
    printf("DMACH0E: %#2X \n\r", DMA0->ERR & DMA_ERR_ERR0_MASK);
    */
    return;
}

void
Motor_LogFtmStatus(void)
{
    //printf("FTMCH1 : %#2X \n\r", FTM0->STATUS & FTM_STATUS_CH1F_MASK);
    //printf("FTMCH2 : %#2X \n\r", FTM0->STATUS & FTM_STATUS_CH2F_MASK);
    
    //printf("\n\n\r");
    
    return;
}


void
Motor_LogDmaBuffers(void)
{
    local_persist volatile u32 packet_bit_counter = 0;
    
    //SOURCE
    //printf("Dshot CMD Buffer[%2d]: %#2X \n\r",
    //packet_bit_counter, 
    //global_Dshot_command_buffer[packet_bit_counter++]);
    
    //DESTINATION
    //printf("Dshot FTM counter   : %#2X \n\r" , (u32)(FTM0->CONTROLS[2].CnV & FTM_CnV_VAL_MASK));
    
    packet_bit_counter *= (u32)(packet_bit_counter < 17);
    
    return;
}


#endif //FLIGHTCONTROLLER_MOTOR_H