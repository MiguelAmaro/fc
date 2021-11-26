/* date = November 24th 2021 10:52 am */

#ifndef FC_COMM_H
#define FC_COMM_H


// NOTE(MIGUEL): Metatsyntax
/*
message      : header | payload
header       : message type | sub system
       message type : state | error
sub system   : motor | camera | ecompass
motor        : sys status | driver status
camera       : sys status | driver status
ecompass     : sys status | driver status
driver       : MCU | I2C | SPI | DMA | UART | FTM

*/

/*
header:
sub system
payload:
 code
error

*/

typedef enum telem_alphabet telem_alphabet;
enum telem_alphabet
{
    Telem_Ack,
};


typedef enum telem_state telem_state;
enum telem_state
{
    Telem_NoConnection,
    Telem_Waiting,
};


// TODO(MIGUEL): bits[2]
typedef enum telem_type telem_type;
enum telem_type
{
    Telem_Data    = 2,
    Telem_Status  = 1,
    Telem_Address = 0,
};

// TODO(MIGUEL): bits[3] change to data_src(which subsystem data and data type is implicit)
typedef enum telem_data_type telem_data_type;
enum telem_data_type
{
    Telem_s8    = 0,
    Telem_u8    = 1,
    Telem_u32   = 2,
    Telem_str8  = 3,
    Telem_f32   = 4,
    Telem_v3f32 = 5,
    Telem_s32   = 6,
};

u32 TelemDataTypeLookUpTable[8] =
{
    1, 1, 4, 4, 4, 12, 1, 0,
};


#define COMM_TERMINATE 0xff

void
Telemetry_PackageAndSend(telem_type MsgType,
                         telem_data_type MsgDataType,
                         void *Msg,
                         u32 MsgCount)
{
    u8 Packet[512];
    u32 PacketSize = 0;
    
    if(MsgCount > 255)
    {
        MsgCount = 255;
    }
    
    u8 *Header = Packet;
    *Header++ = (u8)((MsgType     << 6) |
                     (MsgDataType << 3)); 
    *Header++ = (u8)(MsgCount);
    PacketSize = 2;
    
    
    u32 MsgDataTypeSize = TelemDataTypeLookUpTable[MsgDataType];
    
    u8 *Payload = Header;
    u8 *Src  = (u8 *)Msg;
    u8 *Dest = Payload;
    
    for(u32 Index = 0; Index < MsgCount; Index++)
    {
        *Dest++     = *Src++;
        PacketSize += MsgDataTypeSize;
    }
    
    for(u32 Index = 0; Index < PacketSize; Index++)
    {
        fputc(Packet[Index], &__stdout);
    }
    
    return;
}




#endif //FC_COMM_H
