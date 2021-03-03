#include "MK82F25615.h"
#include "LAL.h"
#include "FlightController_Debug.h"
#include "FlightController_OnboardLEDs.h"
#include "FlightController_I2C.h"

#define RUNNING (1)
#define DASSERT(Expression) if(!(Expression)){GPIOC->PSOR |= LED_RED; *(u32 *)0x00 = 0; }

//~ 0V7670 DEFINITIONS
#define OV7670_SLAVE_ADDRESS (0x2A)
#define OV7670_REG_VREF      (0x03)
#define OV7670_REG_PID       (0x0A)
#define OV7670_REG_VER       (0x0B)
#define OV7670_REG_COM7      (0x12)
#define OV7670_REG_COM7      (0x12)
#define OV7670_REG_COM8      (0x13)

//~ FXOS8700CQ DEFINITIONS

// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDRESS (0x1E) // with pins SA0=0, SA1=0
// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS        (0x00)
#define FXOS8700CQ_WHOAMI        (0x0D)
#define FXOS8700CQ_XYZ_DATA_CFG  (0x0E)
#define FXOS8700CQ_CTRL_REG1     (0x2A)
#define FXOS8700CQ_M_CTRL_REG1   (0x5B)
#define FXOS8700CQ_M_CTRL_REG2   (0x5C)
#define FXOS8700CQ_WHOAMI_VAL    (0xC7)

#define FXOS8700CQ_READ_LEN    13   

typedef struct
{
    s16 x;
    s16 y;
    s16 z;
    s16 padding;
} vec3_s16;


vec3_s16 g_mag_buffer;
vec3_s16 g_acc_buffer;

void Ecompass_init(void *pointer)
{
    u8 data = 0;
    
    /// Check the Who Am I register
    // NOTE(MIGUEL): Should I check for I2C error
    data = I2C_read_byte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_WHOAMI_VAL);
    
    
    if(data != FXOS8700CQ_WHOAMI_VAL)
    {
        printf("I2C Error \n\r");
    }
    
    printf("bye %#2X \n\r", data);
    
    I2C_debug_log_status();
    
    /*
    //~ Initialize Accelerometer Control Register 1 to zero to put the accelarometer in standby
    
    data = 0x00;
    I2C_write_byte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_M_CTRL_REG1, data);
    
    // TODO(MIGUEL): I2C Error checking
    
    //~ Initialize Magnetometer Control Register 1 according to the comments below
    
    // [7]  : m_acal =   0: auto calibration disabled
    // [6]  : m_rst  =   0: no one-shot magnetic reset
    // [5]  : m_ost  =   0: no one-shot magnetic measurement
    // [4-2]: m_os   = 111: 8x oversampling (for 200Hz) to reduce magnetometer noise
    // [1-0]: m_hms  =  11: select hybrid mode with accel and magnetometer active
    
    data = 0x1F;
    I2C_write_byte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_M_CTRL_REG1, data);
    
    // TODO(MIGUEL): I2C Error checking
    
    //~ Initialize Magnetometer Control Register 2 according to the comments below
    
    // [7]  : reserved
    // [6]  : reserved
    // [5]  : hyb_autoinc_mode =  1: to map the magnetometer registers to follow theaccelerometer registers
    // [4]  : m_maxmin_dis     =  0: to retain default min/max latching even though not used
    // [3]  : m_maxmin_dis_ths =  0:
    // [2]  : m_maxmin_rst     =  0:
    // [1-0]: m_rst_cnt        = 00: to enable magnetic reset each cycle
    
    data = 0x20;
    I2C_write_byte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_M_CTRL_REG2, data);
    
    // TODO(MIGUEL): I2C Error checking
    
    //~ Initialize XYZ Data Config Register(is this shared between Mag & Accel?) according to the comments below
    
    // [7]  : reserved
    // [6]  : reserved
    // [5]  : reserved
    // [4]  : hpf_out  = 0
    // [3]  : reserved
    // [2]  : reserved
    // [1-0]: fs       = 01 for accelerometer range of +/-4g range with 0.488mg/LSB
    
    data = 0x01;
    I2C_write_byte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_XYZ_DATA_CFG, data);
    
    // TODO(MIGUEL): I2C Error checking
    
    //~ Initialize Accelerometer Control Register 1 according to the comments below
    
    // [7-6]: aslp_rate =  00:
    // [5-3]: dr        = 001: for 200Hz data rate (when in hybrid mode)
    // [2]  : lnoise    =   1: for low noise mode
    // [1]  : f_read    =   0: for normal 16 bit reads
    // [0]  : active    =   1: to take the part out of standby and enable sampling
    
    data = 0x0D;
    I2C_write_byte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_M_CTRL_REG1, data);
    
    // TODO(MIGUEL): I2C Error checking
    
    return;
}

void Ecompass_read_raw_data(vec3_s16 *mag_raw_data, vec3_s16 *acc_raw_data)
{
    // NOTE(MIGUEL): Do I need a file pointer and whY?
    u8 buffer[FXOS8700CQ_READ_LEN];
    
    I2C_read_nbytes(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_STATUS, buffer, FXOS8700CQ_READ_LEN);
    
    //NOTE(MIGUEL): Why are we starting at index 1?
    /// 14bit accel data in 16bit words
    acc_raw_data->x = (u16)(( buffer[ 1] << 8) | buffer[ 2]) >> 2;
    acc_raw_data->y = (u16)(( buffer[ 3] << 8) | buffer[ 4]) >> 2;
    acc_raw_data->z = (u16)(( buffer[ 5] << 8) | buffer[ 6]) >> 2;
    
    mag_raw_data->x = (u16)(( buffer[ 7] << 8) | buffer[ 8]) >> 2;
    mag_raw_data->y = (u16)(( buffer[ 9] << 8) | buffer[10]) >> 2;
    mag_raw_data->z = (u16)(( buffer[11] << 8) | buffer[12]) >> 2;
    
    // TODO(MIGUEL): I2C Error checking if read fails
    */
    return;
}

int main(void) 
{
    
    Debug_init_uart(115200);
    OBLEDs_init();
    
    printf("Core Frequency: %d \n\n\r", SystemCoreClock);
    
    // NOTE(MIGUEL): DUE TO PIN SHARING MODULES MUST BE TESTED INDIVIDUALLY
    // TODO(MIGUEL): CREATE A PIN MUTUAL EXCLUSION SYSTEM
    
    // ************ ECOMPASS CONTROL *************
    {
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
        PORTC->PCR[13] = PORT_PCR_MUX ( 1); /// FXOS8700CQ Interrupt
        GPIOC->PDDR   |= GPIO_PDDR_PDD(13);
        
        Ecompass_init((void *)0x00);
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
        // ****************************************
        // FLEXIO SETUP
        // ****************************************
        // TODO(MIGUEL): Check CLOCKOUT with o-scope
        I2C_init();
        
        //~ GATING
        SIM->SCGC2 |= SIM_SCGC2_FLEXIO_MASK;
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK ;
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK ;
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK ;
        
        //~ FLEXIO SETUP
        FLEXIO0->CTRL |= FLEXIO_CTRL_FLEXEN_MASK;
        
        // NOTE(MIGUEL): PTB23 Shared with Radio 
        PORTB->PCR[ 0] = PORT_PCR_MUX(7); /// FXIO_DATA__0 - CAM_PCLK
        PORTB->PCR[ 2] = PORT_PCR_MUX(7); /// FXIxO_DATA__2 - CAM_VSYNC
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
        
        SIM->SOPT2    |= SIM_SOPT2_CLKOUTSEL(4); /// MCGIRCLK - I THINK ITS THE SAME AS SYSTEM CLOCK FREQ
        
        // ****************************************
        // OV7670 SETUP
        // ****************************************
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
            
            //~ I2C TESTING
            /*
            I2C3->FLT &= ~I2C_FLT_STOPF_MASK;
            I2C_write_byte         (OV7670_SLAVE_ADDRESS, OV7670_REG_PID, 0x1);
            result = I2C_read_byte (OV7670_SLAVE_ADDRESS, OV7670_REG_COM7);
            I2C_write_byte         (OV7670_SLAVE_ADDRESS, OV7670_REG_PID, 0x1);
            printf("I2C read value: %#2X \n\r", (u32)result);
            I2C_debug_log_status();
            */
            //~ ECOMPASS TESTING
            //Ecompass_read_raw_data(&g_mag_buffer, &g_acc_buffer);
            
            printf("Mag X: %d \n\r", g_mag_buffer.x);
            printf("Mag Y: %d \n\r", g_mag_buffer.y);
            printf("Mag Z: %d \n\r", g_mag_buffer.z);
            
            printf("ACC X: %d \n\r", g_acc_buffer.x);
            printf("ACC Y: %d \n\r", g_acc_buffer.y);
            printf("ACC Z: %d \n\r", g_acc_buffer.z);
            printf("\n\n\r");
            
            
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

