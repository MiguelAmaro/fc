/* date = January 22nd 2021 6:12 pm */

#ifndef FLIGHTCONTROLLER_SYSTEM_H
#define FLIGHTCONTROLLER_SYSTEM_H

//24e6
//48e6

// NOTE(MIGUEL): Is this even the frequency? yess, uart use 48Mhz IRC
//#define SYS_CLOCK                 (20971520U) 
//#define BUS_CLOCK                 (20971520U)

//#define SYS_CLOCK                 (48e6)
//#define BUS_CLOCK                 (48e6)


u32 query_system_clock(void)
{
    SystemCoreClockUpdate();
    
    return SystemCoreClock;
}

u32 query_bus_clock(void)
{
    SystemCoreClockUpdate();
    
    f32 divider = ((f32)(0x01U + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)) / 
                   (f32)(0x01U + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT)));
    
    return (u32)((f32)SystemCoreClock * divider);
}

#endif //FLIGHTCONTROLLER_SYSTEM_H

