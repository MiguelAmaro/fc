#ifndef FLIGHTCONTROLLER_ECOMPASS_H
#define FLIGHTCONTROLLER_ECOMPASS_H


//~ FXOS8700CQ DEFINITIONS

// FXOS8700CQ I2C address // with pins SA0=0, SA1=0 | possible slave addresses 1c(no) 1d(no) || 1e(no) 1f
#define FXOS8700CQ_SLAVE_ADDRESS (0x38)

// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS        (0x00)
#define FXOS8700CQ_WHOAMI        (0x0D)
#define FXOS8700CQ_XYZ_DATA_CFG  (0x0E)
#define FXOS8700CQ_CTRL_REG1     (0x2A)
#define FXOS8700CQ_M_CTRL_REG1   (0x5B)
#define FXOS8700CQ_M_CTRL_REG2   (0x5C)
#define FXOS8700CQ_WHOAMI_VAL    (0xC7)

#define FXOS8700CQ_READ_LEN      (13)   

typedef struct
{
    s16 x;
    s16 y;
    s16 z;
    s16 padding;
} v3s16;


v3s16 g_mag_buffer;
v3s16 g_acc_buffer;


void Ecompass_init(void *pointer)
{
    u8 data = 0;
    
    /// Check the Who Am I register
    // NOTE(MIGUEL): Should I check for I2C error
    data = I2C_ReadByte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_WHOAMI);
    
    if(data != FXOS8700CQ_WHOAMI_VAL)
    {
        //printf("I2C Error:  got %#2X but expected  %#2X  \n\r",
        //data,
        //FXOS8700CQ_WHOAMI_VAL);
        
        I2C_DebugLogStatus();
        
        return;
    }
    
    //printf("byte id correct %#2X \n\r", data);
    
    
    
    //~ Initialize Accelerometer Control Register 1 to zero to put the accelarometer in standby
    
    data = 0x00;
    I2C_WriteByte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_M_CTRL_REG1, data);
    //printf("w done \n\r");
    // TODO(MIGUEL): I2C Error checking
    
    //~ Initialize Magnetometer Control Register 1 according to the comments below
    
    // [7]  : m_acal =   0: auto calibration disabled
    // [6]  : m_rst  =   0: no one-shot magnetic reset
    // [5]  : m_ost  =   0: no one-shot magnetic measurement
    // [4-2]: m_os   = 111: 8x oversampling (for 200Hz) to reduce magnetometer noise
    // [1-0]: m_hms  =  11: select hybrid mode with accel and magnetometer active
    
    data = 0x1F;
    I2C_WriteByte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_M_CTRL_REG1, data);
    //printf("w done \n\r");
    
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
    I2C_WriteByte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_M_CTRL_REG2, data);
    //printf("w done \n\r");
    
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
    I2C_WriteByte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_XYZ_DATA_CFG, data);
    //printf("w done \n\r");
    
    // TODO(MIGUEL): I2C Error checking
    
    //~ Initialize Accelerometer Control Register 1 according to the comments below
    
    // [7-6]: aslp_rate =  00:
    // [5-3]: dr        = 001: for 200Hz data rate (when in hybrid mode)
    // [2]  : lnoise    =   1: for low noise mode
    // [1]  : f_read    =   0: for normal 16 bit reads
    // [0]  : active    =   1: to take the part out of standby and enable sampling
    
    data = 0x0D;
    I2C_WriteByte(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_M_CTRL_REG1, data);
    
    return;
}

void Ecompass_read_raw_data(v3s16 *mag_raw_data, v3s16 *acc_raw_data)
{
    // NOTE(MIGUEL): Do I need a file pointer and whY?
    u8 buffer[FXOS8700CQ_READ_LEN];
    
    // TODO(MIGUEL): simd(neon) wink wink
    
    I2C_ReadNBytes(FXOS8700CQ_SLAVE_ADDRESS, FXOS8700CQ_STATUS, buffer, FXOS8700CQ_READ_LEN);
    
    //NOTE(MIGUEL): Why are we starting at index 1?
    /// 14bit accel data in 16bit words
    acc_raw_data->x = (u16)(( buffer[ 1] << 8) | buffer[ 2]) >> 2;
    acc_raw_data->y = (u16)(( buffer[ 3] << 8) | buffer[ 4]) >> 2;
    acc_raw_data->z = (u16)(( buffer[ 5] << 8) | buffer[ 6]) >> 2;
    
    mag_raw_data->x = (u16)(( buffer[ 7] << 8) | buffer[ 8]) >> 2;
    mag_raw_data->y = (u16)(( buffer[ 9] << 8) | buffer[10]) >> 2;
    mag_raw_data->z = (u16)(( buffer[11] << 8) | buffer[12]) >> 2;
    
    // TODO(MIGUEL): I2C Error checking if read fails
    
    return;
}

void Ecompass_print_debug_info()
{
    //~ ECOMPASS TESTING
    //Ecompass_read_raw_data(&g_mag_buffer, &g_acc_buffer);
    
    
    //printf("Mag X: %d \n\r", g_mag_buffer.x);
    //printf("Mag Y: %d \n\r", g_mag_buffer.y);
    //printf("Mag Z: %d \n\r", g_mag_buffer.z);
    
    //printf("ACC X: %d \n\r", g_acc_buffer.x);
    //printf("ACC Y: %d \n\r", g_acc_buffer.y);
    //printf("ACC Z: %d \n\r", g_acc_buffer.z);
    //printf("\n\n\r");
    
    
    return;
}


#endif // FLIGHTCONTROLLER_ECOMPASS_H


