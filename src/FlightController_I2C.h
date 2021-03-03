/* date = February 15th 2021 2:16 pm */

#ifndef K82F_I2C_H
#define K82F_I2C_H


//~ I2C DEFINITIONS
#define I2C_MASTER_START    (I2C3->C1 |=  I2C_C1_MST_MASK )
#define I2C_MASTER_RESTART  (I2C3->C1 |=  I2C_C1_RSTA_MASK)
#define I2C_MASTER_STOP     (I2C3->C1 &= ~I2C_C1_MST_MASK )
#define I2C_TRANSMIT_MODE   (I2C3->C1 |=  I2C_C1_TX_MASK  ) 
#define I2C_RECIEVE_MODE    (I2C3->C1 &= ~I2C_C1_TX_MASK  ) 
#define I2C_WAIT             while((I2C3->S & I2C_S_IICIF_MASK) == 0){} I2C3->S |= I2C_S_IICIF_MASK

#define I2C_NACK            (I2C3->C1 |=  I2C_C1_TXAK_MASK)
#define I2C_ACK             (I2C3->C1 &= ~I2C_C1_TXAK_MASK)
#define I2C_PUSH_DATA(data) (I2C3->D   = data   )
#define I2C_PULL_DATA(data) (data      = I2C3->D)

#define I2C_RX_ACK_RECIEVED      (I2C3->S   &   I2C_S_RXAK_MASK    )
#define I2C_CLEAR_INTERRUPT_FLAG (I2C3->S   &= ~I2C_S_IICIF_MASK   ) 
#define I2C_CLEAR_START_FLAG     (I2C3->FLT &= ~I2C_FLT_STARTF_MASK)
#define I2C_CLEAR_STOP_FLAG      (I2C3->FLT &= ~I2C_FLT_STOPF_MASK )

// TODO(MIGUEL): Implement some I2C error checking
void I2C_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM->SCGC1 |= SIM_SCGC1_I2C3_MASK ;
    
    // NOTE(MIGUEL): PTA1 & PTA2 are shared with ECompass
    PORTA->PCR[ 1] = PORT_PCR_MUX(4); /// SDATA
    PORTA->PCR[ 2] = PORT_PCR_MUX(4); /// SCLOCK
    
    // TODO(MIGUEL): CALCULATE BAUD RATE
    // NOTE(MIGUEL): BAUD = (I2C MODULE CLK SPEED in HZ) / (MUL * SCL DIVIDER)
    I2C3->F    |= I2C_F_MULT(3)    ; /// MULTIPILER FACTOR
    I2C3->F    |= I2C_F_ICR (0)    ; /// CLOCK 
    I2C3->C1   |= I2C_C1_IICEN_MASK; /// ENABLE I2C 
    I2C3->C1   |= I2C_C1_IICIE_MASK; /// ENABLE INTERRUPTS
    
    //I2C3->SLTH |= 0X01             ; /// SCLK HIGH TIMEOUT
    //I2C3->SLTL |= 0X01             ; /// SCLK HIGH TIMEOUT
    
    I2C3->C2   |= I2C_C2_HDRS_MASK ; /// HIGH DRIVE SELECT
    
    return;
}


u8 I2C_read_byte(u8 device, u8 device_register)
{
    u8 data = 0;
    
    /// Clean Up
    I2C_CLEAR_INTERRUPT_FLAG      ;
    I2C_CLEAR_START_FLAG          ;
    I2C_CLEAR_STOP_FLAG           ;
    
    I2C_TRANSMIT_MODE             ;
    I2C_MASTER_START              ;
    
    printf("RX ACK %d \n\r", I2C_RX_ACK_RECIEVED);
    
    if(I2C_RX_ACK_RECIEVED == 0)
    {
        I2C_PUSH_DATA(device)         ;
        I2C_WAIT                      ;
    }
    else
    {
        I2C_MASTER_STOP               ;
        printf("00 ERROR ACK NOT RECIEVED \n\r");
        return 0;
    }
    
    if(I2C_RX_ACK_RECIEVED == 0)
    {
        I2C_PUSH_DATA(device_register);
        I2C_WAIT                      ;
    }
    else
    {
        I2C_MASTER_STOP               ;
        printf("01 ERROR ACK NOT RECIEVED \n\r");
        return 0;
    }
    
    if(I2C_RX_ACK_RECIEVED == 0)
    {
        I2C_MASTER_RESTART            ;
        I2C_PUSH_DATA(device | 0x1)   ;
        I2C_WAIT                      ;
    }
    else
    {
        I2C_MASTER_STOP               ;
        printf("02 ERROR ACK NOT RECIEVED \n\r");
        return 0;
    }
    
    if(I2C_RX_ACK_RECIEVED == 0)
    {
        I2C_NACK                      ;
        I2C_RECIEVE_MODE              ;
    }
    else
    {
        I2C_MASTER_STOP               ;
        printf("03 ERROR ACK NOT RECIEVED \n\r");
        return 0;
    }
    
    /// Dummy read
    I2C_PULL_DATA(data)           ;
    I2C_WAIT                      ;
    
    I2C_MASTER_STOP               ;
    I2C_PULL_DATA(data)           ;
    
    return data;
}

void I2C_read_nbytes(u8 device, u8 device_register, u8* buffer, u32 size)
{
    u8  dummy      = 0;
    u32 bytes_read = 0;
    
    I2C_TRANSMIT_MODE             ;
    I2C_MASTER_START              ;
    
    I2C_PUSH_DATA(device)         ;
    I2C_WAIT                      ;
    
    I2C_PUSH_DATA(device_register);
    I2C_WAIT                      ;
    
    I2C_MASTER_RESTART            ;
    I2C_PUSH_DATA(device | 0x1)   ;
    I2C_WAIT                      ;
    
    I2C_RECIEVE_MODE              ;
    I2C_ACK                       ;
    
    I2C_PULL_DATA(dummy)          ;
    I2C_WAIT                      ;
    
    do
    {
        I2C_ACK                            ;
        I2C_PULL_DATA(buffer[bytes_read++]);
        
    }while (bytes_read < (size - 2));
    
    I2C_NACK                           ;
    I2C_PULL_DATA(buffer[bytes_read++]);
    
    I2C_WAIT                      ;
    I2C_MASTER_STOP               ;
    
    return;
}

// NOTE(MIGUEL):  Hard crash when using this function after I2C_read_byte
void I2C_write_byte(u8 device, u8 device_register, u8 data)
{
    I2C_TRANSMIT_MODE             ;
    I2C_MASTER_START              ;
    
    I2C_PUSH_DATA(device)         ;
    I2C_WAIT                      ;
    
    I2C_PUSH_DATA(device_register);
    I2C_WAIT                      ;
    
    
    I2C_PUSH_DATA(data)           ;
    I2C_WAIT                      ;
    
    I2C_MASTER_STOP               ;
    
    return;
}

void I2C_debug_log_status(void)
{
    printf("ACK SIGNAL RECIEVED   : %#2X \n\r", I2C3->S   & I2C_S_RXAK_MASK    );
    printf("INTERRUPT PENDING     : %#2X \n\r", I2C3->S   & I2C_S_IICIF_MASK   );
    printf("RANGE ADDRESS MATCH   : %#2X \n\r", I2C3->S   & I2C_S_RAM_MASK     );
    printf("Bus is busy           : %#2X \n\r", I2C3->S   & I2C_S_BUSY_MASK    );
    printf("ADDRESED AS SLAVE     : %#2X \n\r", I2C3->S   & I2C_S_IAAS_MASK    );
    printf("BYTE TRANSFER COMPLETE: %#2X \n\r", I2C3->S   & I2C_S_TCF_MASK     );
    printf("READ WRITE ERRORS     : %#2X \n\r", I2C3->S2  & I2C_S2_ERROR_MASK  );
    printf("TX or RX Buffers Empty: %#2X \n\r", I2C3->S2  & I2C_S2_EMPTY_MASK  );
    printf("FLT_STARTF            : %#2X \n\r", I2C3->FLT & I2C_FLT_STARTF_MASK);
    printf("FLT_STOPF             : %#2X \n\r", I2C3->FLT & I2C_FLT_STOPF_MASK );
    printf("\n\n\r");
    
    
    return;
}


#endif //K82_I2C_H
