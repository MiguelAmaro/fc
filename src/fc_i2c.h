/* date = February 15th 2021 2:16 pm */

#ifndef K82F_I2C_H
#define K82F_I2C_H

typedef enum
{
    mhz_001 = 0,
    khz_400 = 1,
    khz_200 = 0x05,
    khz_100 = 2,
} i2c_baud;

typedef struct
{
    u32 baud;
    b32 isrs_enabled;
    
} i2c_state;
global b32 g_lock_detect = 0;
global b32 g_i2c_locked  = 0;

i2c_state i2cstate = { 0 };

//~ I2C DEFINITIONS
#define I2C_MASTER_START         (I2C3->C1 |=  I2C_C1_MST_MASK )
#define I2C_MASTER_RESTART       (I2C3->C1 |=  I2C_C1_RSTA_MASK)
#define I2C_MASTER_STOP          (I2C3->C1 &= ~I2C_C1_MST_MASK )
#define I2C_TRANSMIT_MODE        (I2C3->C1 |=  I2C_C1_TX_MASK  ) 
#define I2C_RECIEVE_MODE         (I2C3->C1 &= ~I2C_C1_TX_MASK  ) 
#define I2C_WAIT                  while((I2C3->S & I2C_S_IICIF_MASK) == 0){} I2C3->S |= I2C_S_IICIF_MASK

#define I2C_NACK                 (I2C3->C1 |=  I2C_C1_TXAK_MASK)
#define I2C_ACK                  (I2C3->C1 &= ~I2C_C1_TXAK_MASK)
#define I2C_PUSH_DATA(data)      (I2C3->D   = data   )
#define I2C_PULL_DATA(data)      (data      = I2C3->D)

#define I2C_RX_ACK_RECIEVED      (!(I2C3->S &   I2C_S_RXAK_MASK) )
#define I2C_CLEAR_INTERRUPT_FLAG (I2C3->S   &= ~I2C_S_IICIF_MASK   ) 
#define I2C_CLEAR_START_FLAG     (I2C3->FLT &= ~I2C_FLT_STARTF_MASK)
#define I2C_CLEAR_STOP_FLAG      (I2C3->FLT &= ~I2C_FLT_STOPF_MASK )

void I2C_DebugLogStatus(void)
{
    //printf("I2C Status vvvv ");
    
    if(I2C3->S   & I2C_S_RXAK_MASK)
    {
        //printf("no ack signal was reacieved \n\r");
    }
    if(I2C3->S   & I2C_S_IICIF_MASK)
    {
        /*
"- One byte transfer, including ACK/NACK bit, completes if FACK is 0. \n\r"
               "  An ACK or NACK is sent on thebus by writing 0 or 1 to TXAK after - \n\r"
               "  this bit is set in receive mode.\n\r"
               
               "- One byte transfer, excluding ACK/NACK bit, completes if FACK is 1.\n\r"
               
               "- Match of slave address to calling address including primary slave address, \n\r"
               "  range slave address,alert response address, second slave address, \n\r"
               "  or general call address.\n\r"
               
               "- Arbitration lost \n\r"
               "- In SMBus mode, any timeouts except SCL and SDA high timeouts\n\r"
               "- I2C bus stop or start detection if the SSIE bit in the Input Glitch Filter register is 1\n\r"
*/
        //printf("interrupt pending(possible reasons): \n\r");
    }
    if(I2C3->S   & I2C_S_RAM_MASK)
    {
        //printf("addressed as slave\n\r");
    }
    if(I2C3->S   & I2C_S_BUSY_MASK    )
    {
        //printf("Bus is busy \n\r" );
        
    }
    if(!(I2C3->S   & I2C_S_TCF_MASK    ) )
    {
        //printf("byte transfer is in progress\n\r");
    }
    //printf("ADDRESED AS SLAVE     : %#2X \n\r", I2C3->S   & I2C_S_IAAS_MASK    );
    if(I2C3->S2  & I2C_S2_ERROR_MASK  )
    {
        //printf("3 or more read write errors during data transfer phase(empty flag not set & buffer busy)\n\r");
    }
    if(!(I2C3->S2  & I2C_S2_EMPTY_MASK)  )
    {
        //printf("rx or tx buffer has data. cannot write at this time\n\r" );
    }
    if(!(I2C3->FLT & I2C_FLT_STARTF_MASK))
    {
        //printf("no start status was detected on bus\n\r");
        
    }
    if(!(I2C3->FLT & I2C_FLT_STOPF_MASK ))
    {
        //printf("no stop status was detected on bus\n\r");
        
    }
    //printf("\n\n\r");
    
    
    return;
}


void I2C_ReleaseBusDelay()
{
    for(u32 cycle = 0; cycle < 10U; cycle++)
    { __NOP(); }
    
    return;
}

void I2C_ReleaseBus(void)
{
    //printf("releasing bus \n\r");
    
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    
    PORTA->PCR[1] &= ~PORT_PCR_MUX_MASK; /// SDATA
    PORTA->PCR[2] &= ~PORT_PCR_MUX_MASK; /// SDATA
    
    PORTA->PCR[1] = PORT_PCR_MUX(1); /// GPIO SDA
    PORTA->PCR[2] = PORT_PCR_MUX(1); /// GPIO SCK
    
    
    GPIOA->PSOR |= 0x02; /// GPIO SDA
    GPIOA->PDDR |= 0x02; /// OUTPUT SDA
    GPIOA->PSOR |= 0x04; /// GPIO SCK
    GPIOA->PDDR |= 0x04; /// OUTPUT SCK
    
    
    GPIOA->PCOR |= 0x02; /// GPIO SDA
    I2C_ReleaseBusDelay();
    
    for(u32 pulse_num = 0; pulse_num < 9; pulse_num++)
    {
        GPIOA->PCOR |= 0x04; /// GPIO SCK
        I2C_ReleaseBusDelay();
        
        GPIOA->PSOR |= 0x02; /// GPIO SDA
        I2C_ReleaseBusDelay();
        
        GPIOA->PSOR |= 0x04; /// GPIO SCK
        I2C_ReleaseBusDelay();
        I2C_ReleaseBusDelay();
    }
    
    // GEN STOP SIGNAL
    GPIOA->PCOR |= 0x04; /// GPIO SCK
    I2C_ReleaseBusDelay();
    
    GPIOA->PCOR |= 0x02; /// GPIO SDA
    I2C_ReleaseBusDelay();
    
    GPIOA->PSOR |= 0x04; /// GPIO SCK
    I2C_ReleaseBusDelay();
    
    GPIOA->PSOR |= 0x02; /// GPIO SDA
    I2C_ReleaseBusDelay();
    
    return;
}

void I2C_Init(u32 mult, u32 icr)
{
    I2C_ReleaseBusDelay();
    
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM->SCGC1 |= SIM_SCGC1_I2C3_MASK ;
    
    
    
    //printf("configuring I2C \n\r");
#if 1
    PORTA->PCR[1] &= ~PORT_PCR_LK_MASK;
    PORTA->PCR[2] &= ~PORT_PCR_LK_MASK;
    PORTA->PCR[1] &= ~PORT_PCR_PFE_MASK;
    PORTA->PCR[1] &= ~PORT_PCR_DSE_MASK;
    PORTA->PCR[1] &= ~PORT_PCR_LK_MASK;
    PORTA->PCR[1] |=  PORT_PCR_PE(1);
    PORTA->PCR[1] |=  PORT_PCR_ODE(1);
    
    PORTA->PCR[2] &= ~PORT_PCR_PFE_MASK;
    PORTA->PCR[2] &= ~PORT_PCR_DSE_MASK;
    PORTA->PCR[2] &= ~PORT_PCR_LK_MASK;
    PORTA->PCR[2] |=  PORT_PCR_PE(1);
    PORTA->PCR[2] |=  PORT_PCR_ODE(1);
    
    
    /// RESET PREIPHERAL
    I2C3->A1  = 0x00U;
    I2C3->F   = 0x00U;
    I2C3->C1  = 0x00U;
    I2C3->S   = 0x00U;
    I2C3->C2  = 0x00U;
    I2C3->FLT = 0x50U;
    I2C3->RA  = 0x00U;
    
#endif
    PORTA->PCR[1] &= ~PORT_PCR_MUX_MASK; /// SDATA
    PORTA->PCR[2] &= ~PORT_PCR_MUX_MASK; /// SDATA
    
    PORTA->PCR[1] |=  PORT_PCR_MUX(4); /// SDATA
    PORTA->PCR[2] |=  PORT_PCR_MUX(4); /// SCLOCK
    
    I2C3->C1   &= ~I2C_C1_IICEN_MASK; /// DISABLE I2C 
    
    /// CLEAR STATUS FLAGS?
    
    /// SET BAUD RATE 
    /// (I2C MODULE CLK SPEED in HZ) / (MUL * SCL DIVIDER)
    I2C3->F    |= I2C_F_MULT(mult); /// MULTIPILER FACTOR
    I2C3->F    |= I2C_F_ICR (icr) ; /// CLOCK DIVIDER
    
    I2C3->C1   |= I2C_C1_IICIE_MASK; /// ENABLE INTERRUPTS
    I2C3->C1   |= I2C_C1_IICEN_MASK; /// ENABLE I2C 
#if 1
    /// FILTER
    u8 filter_reg;
    
    filter_reg  = I2C3->FLT;
    filter_reg &= ~I2C_FLT_SHEN_MASK;
    filter_reg |=  I2C_FLT_SHEN(0);
    filter_reg &= ~I2C_FLT_FLT_MASK ;
    filter_reg |= I2C_FLT_FLT(0U);
    
    I2C3->FLT = filter_reg;
#endif
    I2C3->C2   |= I2C_C2_HDRS_MASK ; /// HIGH DRIVE SELECT
    
    //printf("I2C config done \n\r");
    return;
}


void I2C_Busy(void)
{
	// Start Signal
	g_lock_detect = 0;
	
    I2C3->C1 &= ~I2C_C1_IICEN_MASK;
	
    I2C_TRANSMIT_MODE;
	I2C_MASTER_START ;
    
	I2C3->C1 |=  I2C_C1_IICEN_MASK;
    
	// Write to clear line
	I2C3->C1 |= I2C_C1_MST_MASK; // set MASTER mode   
	I2C3->C1 |= I2C_C1_TX_MASK ; // Set transmit (TX) mode   
	I2C_PUSH_DATA(0xff);
    
    while ((I2C3->S & I2C_S_IICIF_MASK) == 0U) {} 
    // wait interrupt   
    
	I2C3->S |= I2C_S_IICIF_MASK; // clear interrupt bit   
    
    
    // Clear arbitration error flag  
	I2C3->S |= I2C_S_ARBL_MASK;
    
    
    // Send start   
	I2C3->C1 &= ~I2C_C1_IICEN_MASK;
	I2C3->C1 |= I2C_C1_TX_MASK; // Set transmit (TX) mode   
	I2C3->C1 |= I2C_C1_MST_MASK; // START signal generated   
    
	I2C3->C1 |= I2C_C1_IICEN_MASK;
    //Wait until start is send  
    
    // Send stop   
	I2C3->C1 &= ~I2C_C1_IICEN_MASK;
	I2C3->C1 |=  I2C_C1_MST_MASK;
	I2C3->C1 &= ~I2C_C1_MST_MASK; // set SLAVE mode   
	I2C3->C1 &= ~I2C_C1_TX_MASK ; // Set Rx   
	I2C3->C1 |=  I2C_C1_IICEN_MASK;
    
	
    // wait   
    // Clear arbitration error & interrupt flag  
	I2C3->S |= I2C_S_IICIF_MASK;
	I2C3->S |= I2C_S_ARBL_MASK ;
    
	g_lock_detect = 0;
	g_i2c_locked  = 1;
    
    return;
}

void I2C_Wait()
{
    g_lock_detect = 0;
    
    while(((I2C3->S & I2C_S_IICIF_MASK) == 0) & (g_lock_detect < 200))
    { g_lock_detect++; }
    
    if(g_lock_detect >= 200)
    {
        I2C_Busy();
    }
    
    I2C0->S |= I2C_S_IICIF_MASK;
    
    return;
}

void I2C_Start()
{
    I2C_TRANSMIT_MODE;
    I2C_MASTER_START;
    
    return;
}

void I2C_ReadSetup(u8 device, u8 device_register)
{
    I2C_PUSH_DATA(device);
    I2C_WAIT;
    
    I2C_PUSH_DATA(device_register);
    I2C_WAIT;
    
    I2C_MASTER_START;
    I2C_PUSH_DATA(device | 0x01);
    I2C_WAIT;
    
    I2C_RECIEVE_MODE;
    
    return;
}

u8 I2C_RepeatedRead(u8 is_last_read)
{
    u8  data = 0;
    g_lock_detect = 0;
    
    is_last_read ? I2C_NACK : I2C_ACK;
    
    I2C_PULL_DATA(data);
    I2C_WAIT;
    
    if(is_last_read)
    {
        I2C_MASTER_STOP;
    }
    
    I2C_PULL_DATA(data);
    
    return data;
}

#if 0
u8 I2C_ReadByte(u8 device, u8 device_register)
{
    u8 data = 0;
    
    I2C3->S = 0x0U;
    
    /// WRITE TRANSMITION CYCLE
    /// PHASE 1: SELECT SLAVE
    I2C_TRANSMIT_MODE             ;
    I2C_MASTER_START              ;
    
    I2C_PUSH_DATA(device & ~0x01) ;
    I2C_WAIT                      ;
    
    /// PHASE 2: SELECT SLAVE REGISTER
    I2C_PUSH_DATA(device_register);
    I2C_WAIT                      ;
    /// END OF WRITE TRANSMITION CYCLE
    
    I2C_debug_log_status();
    /// READ TRANSMITION CYCLE
    /// PHASE 1: IDENTIFY SLAVE
    I2C_CLEAR_INTERRUPT_FLAG      ;
    {
        u8 mult = 0;
        mult = I2C3->F;
        I2C3->F = mult & ~I2C_F_ICR_MASK;
        I2C_MASTER_RESTART            ;
        I2C3->F = mult;
        u32 delay = 0;
        while( (delay--) != 0U)
        { __NOP(); }
    }
    I2C_PUSH_DATA(device | 0x01)  ;
    I2C_WAIT                      ;
    I2C_CLEAR_INTERRUPT_FLAG      ;
    
    /// PHASE 2: RECIEVE SLAVE DATA
    I2C_RECIEVE_MODE              ;
    I2C_NACK                      ;
    
    I2C_PULL_DATA(data)           ;
    I2C_WAIT                      ;
    I2C_MASTER_STOP               ;
    I2C_ACK;
    I2C_RECIEVE_MODE;
    
    I2C_PULL_DATA(data)           ;
    
    return data;
}
#endif

#if 1
u8 I2C_ReadByte(u8 device, u8 device_register)
{
    u8 data   = 6;
    
    /// WRITE TRANSMITION CYCLE
    /// PHASE 1: SELECT SLAVE
    I2C_TRANSMIT_MODE             ;
    I2C_MASTER_START              ;
    
    I2C_PUSH_DATA(device & ~0x01) ;
    I2C_WAIT                      ;
    
    /// PHASE 2: SELECT SLAVE REGISTER
    I2C_PUSH_DATA(device_register);
    I2C_WAIT                      ;
    /// END OF WRITE TRANSMITION CYCLE
    
    
    /// READ TRANSMITION CYCLE
    /// PHASE 1: IDENTIFY SLAVE
    I2C_MASTER_RESTART            ;
    I2C_PUSH_DATA(device | 0x01)  ;
    I2C_WAIT                      ;
    
    /// PHASE 2: RECIEVE SLAVE DATA
    I2C_RECIEVE_MODE              ;
    I2C_NACK                      ;
    
    I2C_PULL_DATA(data)           ;
    I2C_WAIT                      ;
    I2C_MASTER_STOP               ;
    
    I2C_PULL_DATA(data)           ;
    
    return data;
}
#endif

void I2C_ReadNBytes(u8 device, u8 device_register, u8* buffer, u32 size)
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
void I2C_WriteByte(u8 device, u8 device_register, u8 data)
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

#if 1
void I2C_scanner()
{
    // TODO(MIGUEL): measuere sclk on the o-scope to comfirm that it what we expect (200khz clock rate) given 
    //               the init params
    // NOTE(MIGUEL): if sclk init param yield expected clk rate scan at 100kz, 200kz, 400kz, 1mhz
    //               check the discord i2c scanner post for ref
    // NOTE(MIGUEL): ditch the for loops
    // NOTE(MIGUEL): no reiniting. disable the i2c module then reset the params or else potential failiure
    b32  ack = 0;
    u32 mult[4] = { 2   , 2   ,    1, 0    };
    u32 icr[4]  = { 0x11, 0x05, 0x05, 0x00 };
    u32 baud[4] = { 100, 200,  400,   1000};
    u8 address = 0;
    
    //printf("scanning.. \n\r");
    for(u32 i = 0; i < 4; i++)
    {
        // NOTE(MIGUEL): try different baud rates
        //baud = query_bus_clock() / (mult * icr);
        //printf("baud rate:%d\n\r", baud);
        I2C3->C1   &= ~I2C_C1_IICEN_MASK; /// DIsabl I2C 
        I2C3->F    |= I2C_F_MULT(mult[i]) ; /// MULTIPILER FACTOR
        I2C3->F    |= I2C_F_ICR (icr[i]) ; /// CLOCK DIVIDER
        I2C3->C1   |= I2C_C1_IICEN_MASK; /// ENABLE I2C 
        
        //printf("baud %d\n\r",baud[i]);
        
        for(address = 0; address < 127; address++)
        {
            // NOTE(MIGUEL): try all possible address for a response
            
            //printf("address: %#2X\n\r", address);
            
            /// WRITE TRANSMITION CYCLE
            /// PHASE 1: SELECT SLAVE
            I2C_TRANSMIT_MODE             ;
            I2C_MASTER_START              ;
            
            I2C_PUSH_DATA(address)         ;
            I2C_WAIT                      ;
            
            I2C_MASTER_STOP               ;
            
            if(I2C_RX_ACK_RECIEVED)
            {
                /*printf("device found \n\r" 
                       "address: %#2X\n\r"
                       "response: %#2X\n\r"
                       "baud %d\n\r",
                       address,
                       baud[i]);*/
            }
            
            
            //printf("delay \n\r");
            u32 limit = 100;
            for(volatile u32 j = 0; j < limit; j++)
            {
                for(volatile u32 k = 0;k < limit; k++);
            }
            //printf("delay complete \n\r");
        }
    }
    //printf("scan done \n\r");
    return;
}
#endif

#endif //K82_I2C_H

