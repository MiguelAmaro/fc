#include "MK82F25615.h"
#include "LAL.h"
#include <stdio.h>
#include "RingBuffer.h"
//__asm(".global __use_no_semihosting \n\t");

#define LED_RED   (0x100) /// PTC8
#define LED_GREEN (0x200) /// PTC9
#define LED_BLUE  (0x400) /// PTC10

#define MUX_GPIO (0x0100)
#define PORTC_SIM_SCGC5_MASK (0x800)
#define RUNNING (1)
#define GET_UART_BYTE ((u8)LPUART4->DATA & 0xFFU)

#define DASSERT(Expression) if(!(Expression)){GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

internal RingBuffer transmitQueue, receiveQueue;
/*
struct __FILE
{
    int handle;
};

FILE __stdout;  //Use with printf
FILE __stdin ;  //use with fget/sscanf, or scanfb
*/

//Retarget the fputc method to use the UART0
s32 fputc(s32 byte, FILE *f)
{
	while(!(LPUART4->STAT & LPUART_STAT_TDRE_MASK) && !(LPUART4->STAT & LPUART_STAT_TC_MASK));
	LPUART4->DATA |= 0xFFU & byte;
    
	return byte;
}

//Retarget the fgetc method to use the UART0
s32 fgetc(FILE *f)
{
	while(!(LPUART4->STAT & LPUART_STAT_RDRF_MASK));
    
	return LPUART4->DATA & 0xFF;
}


void UART0_IRQHandler(void) 
{
	u8 receivedChar;
	
	if (LPUART4->STAT & (LPUART_STAT_OR_MASK |LPUART_STAT_NF_MASK | 
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
	if ( (LPUART4->CTRL & LPUART_CTRL_TIE_MASK) &&
        (LPUART4->STAT & LPUART_STAT_TDRE_MASK) )
    { 
        GPIOC->PSOR |= LED_GREEN;
		if (!RingBuffer_Empty(&transmitQueue)) 
        {
			LPUART4->DATA |= 0xFFU & RingBuffer_Dequeue_Byte(&transmitQueue);
		}
        else 
        {
			// queue is empty so disable transmitter interrupt
			LPUART4->CTRL &= ~LPUART_CTRL_TIE_MASK;
		}
	}
}


int main(void) 
{
    // ****************************************
    // ONBOARD LED SETUP
    // ****************************************
    /// SIM PTC
    SIM->SCGC5 |= PORTC_SIM_SCGC5_MASK;
    
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
    // UART SETUP - DEBUGGING
    // ****************************************
#define DEBUG_SERIAL_RX           (0x4000) /// PTC14
#define DEBUG_SERIAL_TX           (0x8000) /// PTC15
#define LPUART4_SIM_SCGC2_MASK    (0x400000)
#define UART4_CTRL_TE_MASK        (0x40000)
#define UART4_CTRL_RE_MASK        (0x80000)
#define MUX_LPUART4_TX_RX         (0x200)
#define LPUART_OVERSAMPLE_RATIO    (16)
#define BUS_CLOCK 			    (24e6)
#define UART_SRC_CLOCK		    (48e6)
    
    
    u32 baud_rate = 0;
    volatile u8 temp;
    // TODO(MIGUEL): Enable PortC
    // TODO(MIGUEL): Enable Uart4
    SIM->SCGC5 |= PORTC_SIM_SCGC5_MASK;
    SIM->SCGC2 |= LPUART4_SIM_SCGC2_MASK;
    // TODO(MIGUEL): Disable Transmitter and Reciever
    LPUART4->CTRL |= UART4_CTRL_TE_MASK | UART4_CTRL_RE_MASK;
    // TODO(MIGUEL): Set Uart clock via SIM_SOPT
    SIM->SOPT2 |= SIM_SOPT2_LPUARTSRC(1) |  SIM_SOPT2_PLLFLLSEL_MASK;
    // TODO(MIGUEL): Set TX & RX pin on portc
    PORTC->PCR[14] = MUX_LPUART4_TX_RX; /// RX
    PORTC->PCR[15] = MUX_LPUART4_TX_RX; /// TX
    
    u16 baud_modulo = (u16)((UART_SRC_CLOCK)/(baud_rate * LPUART_OVERSAMPLE_RATIO)); 
    LPUART4->BAUD &= ~LPUART_BAUD_SBR_MASK;
    LPUART4->BAUD &= ~LPUART_BAUD_SBR(baud_modulo);
    LPUART4->BAUD &= ~LPUART_BAUD_OSR(LPUART_OVERSAMPLE_RATIO - 1);
    
    LPUART4->BAUD |= LPUART_BAUD_RXEDGIE(0) | LPUART_BAUD_SBNS(0) | LPUART_BAUD_LBKDIE(0);
    LPUART4->CTRL |= LPUART_CTRL_LOOPS  (0) | LPUART_CTRL_M   (0) | LPUART_CTRL_PE    (0);
    
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
    
    LPUART4->STAT = LPUART_STAT_MSBF(0) | LPUART_STAT_RXINV(0);
    
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
	temp = (u8)LPUART4->DATA & 0xFFU;
    
	LPUART4->STAT &= ~LPUART_STAT_RDRF_MASK;
    
    // ****************************************
    // FLEXTIMER SETUP
    // ****************************************
    
    
    // ****************************************
    // DMA SETUP
    // ****************************************
    
    
    // ****************************************
    // CAMERA SETUP
    // ****************************************
    
    
    //printf("hello world \n");
    
    u32 counter = 0;
    
    while(RUNNING)
    {
        //printf("H \n");
        
        if(counter == 1000000/4)
        {
            /// TOGGLE LED ON
            GPIOC->PSOR |= LED_BLUE;
        }
        if(counter == 1000000/2)
        {
            /// TOGGLE LED OFF
            GPIOC->PCOR |= LED_BLUE;
        }
        else if (counter == 1000000)
        {
            counter = 0;
        }
        counter++;
    }
}

