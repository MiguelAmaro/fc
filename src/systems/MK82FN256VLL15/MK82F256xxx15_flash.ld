
m_interrupts_start          = 0x00000000;
m_interrupts_size           = 0x000003C0;

m_bootloader_config_start   = 0x000003C0;
m_bootloader_config_size    = 0x00000040;

m_flash_config_start        = 0x00000400;
m_flash_config_size         = 0x00000010;

m_text_start                = 0x00000410;
m_text_size                 = 0x0003FBF0;

m_data_start                = 0x1FFF0000;
m_data_size                 = 0x00010000;

m_data_2_start              = 0x20000000;
m_data_2_size               = 0x00030000;

Stack_Size = DEFINED(__stack_size__) ? __stack_size__ : 0x0400;
Heap_Size  = DEFINED( __heap_size__) ? __heap_size__  : 0x0400;


/* // NOTE(MIGUEL): Mak this into 4 memory regions.
*  //               ROM, ER, RW, LIB
*/

MEMORY
{
    VECTOR_ROM             (rx) : ORIGIN =        m_interrupts_start, LENGTH = (m_text_start +
                                                                                m_text_size -
                                                                                m_interrupts_start);
    
    ER_m_bootloader_config (rx) : ORIGIN = m_bootloader_config_start, LENGTH = m_bootloader_config_size;
    ER_m_flash_config      (rx) : ORIGIN = m_flash_config_start     , LENGTH = m_flash_config_size;
    ER_m_text              (rx) : ORIGIN = m_text_start             , LENGTH = m_text_size;
    
    RW_m_data              (w) : ORIGIN = m_data_start             , LENGTH = m_data_size;
    RW_m_data_2            (w) : ORIGIN = m_data_2_start           , LENGTH = (m_data_2_size -
                                                                               Stack_Size -
                                                                               Heap_Size); 
    
    ARM_LIB_HEAP           () : ORIGIN = ((ImageLimit(RW_m_data_2) == m_data_2_start) ?
                                          m_data_2_start : +0), LENGTH = Heap_Size;
    ARM_LIB_STACK          () : ORIGIN = (m_data_2_start + m_data_2_size), LENGTH = -Stack_Size); 
}

SECTIONS
{
    
    
}
