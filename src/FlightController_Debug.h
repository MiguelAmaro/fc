/* date = January 17th 2021 9:01 pm */

#ifndef FLIGHTCONTROLLER_DEBUG_H
#define FLIGHTCONTROLLER_DEBUG_H

#include "MK82F25615.h"
#include "FlightController_System.h"
#include "LAL.h"
#include <stdio.h>
#include "RingBuffer.h"

// TODO(MIGUEL): Make this

// ****************************************
// UART SETUP - DEBUGGING
// ****************************************
#define DEBUG_SERIAL_RX           (0x4000) /// PTC14
#define DEBUG_SERIAL_TX           (0x8000) /// PTC15
#define LPUART_OVERSAMPLE_RATIO   (16)
#define MUX_LPUART4_TX_RX         (3U)
#define LPUART_CLOCK_SOURCE      (48e6)  ///48MHz IRC is selected

internal RingBuffer transmitQueue, receiveQueue;

struct __FILE
{
    int handle;
};

FILE __stdout;  //Use with printf
FILE __stdin ;  //use with fget/sscanf, or scanfb

void
Debug_init_uart(u32 baud_rate)
{
    volatile u8 temp;
    
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM->SCGC2 |= SIM_SCGC2_LPUART4_MASK;
    
    // Disable Transmitter and Reciever for Initilization
    LPUART4->CTRL &= ~LPUART_CTRL_TE_MASK & ~LPUART_CTRL_RE_MASK;
    
    // Set Uart clock source
    SIM->SOPT2 |= SIM_SOPT2_LPUARTSRC(1)  ;
    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK; // NOTE(MIGUEL): 48MHz IRC is selected - affects CLKOUT pin
    
    // Set TX & RX pins on PORTC
    PORTC->PCR[14] = PORT_PCR_MUX(MUX_LPUART4_TX_RX); /// RX
    PORTC->PCR[15] = PORT_PCR_MUX(MUX_LPUART4_TX_RX); /// TX
    
    u16 baud_modulo = (u16)((LPUART_CLOCK_SOURCE)/(baud_rate * LPUART_OVERSAMPLE_RATIO)); 
    LPUART4->BAUD  &= ~LPUART_BAUD_SBR_MASK;
    LPUART4->BAUD  |=  LPUART_BAUD_SBR(baud_modulo);
    LPUART4->BAUD  |=  LPUART_BAUD_OSR(LPUART_OVERSAMPLE_RATIO - 1);
    
    LPUART4->BAUD |= LPUART_BAUD_RXEDGIE(0) | LPUART_BAUD_SBNS(0) | LPUART_BAUD_LBKDIE(0);
    // NOTE(MIGUEL): ARE THESE CORRECT AT RT
    LPUART4->CTRL  = LPUART_CTRL_LOOPS  (0) | LPUART_CTRL_M   (0) | LPUART_CTRL_PE    (0);
    
    /// Enables interrupt on Error Conditions REF pg.1928
    LPUART4->CTRL = 
        LPUART_CTRL_TXINV(0) | 
        LPUART_CTRL_ORIE (0) | 
        LPUART_CTRL_NEIE (0) | 
        LPUART_CTRL_FEIE (0) | 
        LPUART_CTRL_PEIE (0) ;
    
    LPUART4->STAT = LPUART_STAT_OR(1) | LPUART_STAT_NF(1) | LPUART_STAT_FE(1) | LPUART_STAT_PF(1);
    
    LPUART4->STAT |= 
        LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK | 
        LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK ;
    
    LPUART4->STAT |= LPUART_STAT_MSBF(0) | LPUART_STAT_RXINV(0);
    
    RingBuffer_Init(&transmitQueue);
    RingBuffer_Init(&receiveQueue );
    
    NVIC_SetPriority    (LPUART4_IRQn, 2); // 0, 1, 2, or 3
    NVIC_ClearPendingIRQ(LPUART4_IRQn   ); 
    NVIC_EnableIRQ      (LPUART4_IRQn   );
    
    // Enable receive interrupts but not transmit interrupts yet
    LPUART4->CTRL |= LPUART_CTRL_RIE(1);
    
    
    // Enable UART receiver and transmitter
    LPUART4->CTRL |= LPUART_CTRL_RE(1) | LPUART_CTRL_TE(1);    
    
    // Clear the UART RDRF flag
    temp = (u8)LPUART4->DATA;
    
    LPUART4->STAT &= ~LPUART_STAT_RDRF_MASK;
    
    return;
}


void 
LPUART4_IRQHandler(void) 
{
    u8 receivedChar;
    
    
    if (LPUART4->STAT & (LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK | 
                         LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK)) 
    {
        // clear the error flags
        LPUART4->STAT |= LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK | LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK;    
        
        //DASSERT((LPUART4->DATA & 0xFFU) == 'H');
        receivedChar = LPUART4->DATA & 0xFFU;
    }
    /// CASE: RX DATA REGISTER FULL - BYTE RECIEVED
    if (LPUART4->STAT & LPUART_STAT_RDRF_MASK)
    {
        receivedChar = LPUART4->DATA & 0xFFU;
        if (!RingBuffer_Full(&receiveQueue)) 
        {
            RingBuffer_Enqueue_Byte(&receiveQueue, receivedChar);
        } 
        else 
        {
            // error - queue full.
            // discard character
        }
    }
    /// CASE: TRANSMITTER INTERUPT ENABLED And TX DATA REGISTER EMPTY - BYTE TRANSMITTING
    if ((LPUART4->CTRL & LPUART_CTRL_TIE_MASK ) &&
        (LPUART4->STAT & LPUART_STAT_TDRE_MASK) )
    { 
        if (!RingBuffer_Empty(&transmitQueue)) 
        {
            LPUART4->DATA = 0xFFU & RingBuffer_Dequeue_Byte(&transmitQueue);
        }
        else 
        {
            // queue is empty so disable transmitter interrupt
            LPUART4->CTRL &= ~LPUART_CTRL_TIE_MASK;
        }
    }
    
    return;
}


//Retarget the fputc method to use the UART0
s32 
fputc(s32 byte, FILE *f)
{
    while(!(LPUART4->STAT & LPUART_STAT_TDRE_MASK) && !(LPUART4->STAT & LPUART_STAT_TC_MASK ));
    LPUART4->DATA = 0xFFU & byte;
    
    return byte;
}

//Retarget the fgetc method to use the UART0
s32 
fgetc(FILE *f)
{
    while(!(LPUART4->STAT & LPUART_STAT_RDRF_MASK));
    
    return LPUART4->DATA & 0xFF;
}

#endif // FLIGHTCONTROLLER_UART_H


