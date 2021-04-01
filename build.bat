@ECHO OFF
IF NOT EXIST build MKDIR build

REM ************************************************************
REM *********               DEFINITIONS               **********
REM ************************************************************
REM ==================       FILES/BUILD      ==================
SET Build=CMSIS_CORE
SET Project_Name=K82F_FlightController
SET BOARD=MK82F25615
SET Sources= ..\src\%Project_Name%.c
SET Objects= %Project_Name%.o RingBuffer.o startup_%BOARD%.o system_%BOARD%.o


REM ==================         TARGET        ==================
SET TARGET=arm-arm-none-eabi
SET DFP=%BOARD%_DFP/12.2.0
SET MCPU=cortex-m4
SET MFPU=fpv4-sp-d16
SET ARCH=armv7-m

 
REM ==================   COMPILER(ARMCLANG)  ==================
SET Compiler_Common=^
 -c^
 -std=gnu11^
 -O0^
 -gdwarf-4^
 -mfloat-abi=hard^
 -mthumb^
 -fno-rtti^
 -fno-common^
 -fno-builtin^
 -fshort-enums^
 -fshort-wchar^
 -ffreestanding^
 -fdata-sections^
 -funsigned-char^
 -ffunction-sections^
 -MD

SET Warnings=^
 -Weverything^
 -Wno-packed^
 -Wno-missing-variable-declarations^
 -Wno-missing-prototypes^
 -Wno-missing-noreturn^
 -Wno-sign-conversion^
 -Wno-nonportable-include-path^
 -Wno-reserved-id-macro^
 -Wno-unused-macros^
 -Wno-unused-parameter^
 -Wno-documentation-unknown-command^
 -Wno-documentation^
 -Wno-license-management^
 -Wno-parentheses-equality^
 -Wno-shadow

SET Compiler_Flags= %Compiler_Common% %Warnings%

SET Compiler_Macros=^
 -D__EVAL^
 -D__MICROLIB^
 -D__UVISION_VERSION="531"^
 -D_RTE_^
 -DCPU_MK82FN256VLL15^
 -D_RTE_^
 -DDEBUG^
 -DCPU_MK82FN256VLL15^
 -DFRDM_K82F^
 -DFREEDOM^
 -DSERIAL_PORT_TYPE_UART="1"^
 -D%Build%

SET Include_Directories=^
 -I../RTE/_FlightController_debug^
 -I../RTE/Board_Support/MKV31F512VLL12^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/ARM/CMSIS/5.7.0/CMSIS/DSP/Include^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/ARM/CMSIS/5.7.0/CMSIS/DSP/PrivateInclude^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/drivers^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/uart^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/lists^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/serial_manager^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities/str^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities/debug_console^
 -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/Device/Include


REM ==================    LINKER(ARMLINK)   ====================
SET Memory_Layout=^
 --ro-base 0x00000000^
 --entry 0x00000000^
 --rw-base 0x1FFF8000^
 --entry Reset_Handler^
 --first __Vectors

SET Common_Linker_Flags=^
 --library_type=microlib^
 --diag_suppress 6314^
 --strict^
 --scatter "..\RTE\Device\MK82FN256VLL15\MK82FN256xxx15_flash.scf"^
 --keep=*(.FlashConfig)^
 --summary_stderr^
 --remove^
 --map^
 --xref^
 --symbols^
 --callgraph^
 --info sizes^
 --info totals^
 --info unused^
 --info veneers^
 --info summarysizes^
 --load_addr_map_info^
 --list "..\debug\%Project_Name%.map"

SET Libraries=


ECHO ************************************************************
ECHO **********              START BUILD               **********
ECHO ************************************************************
pushd build
echo %CD%
IF %Build%==CMSIS_CORE ( ECHO CMSIS_CORE Build & CALL :Compile_Core & CALL :Link ) ELSE ( ECHO FREESCALE Build & CALL :Compile_Core & CALL :Compile_FSL_Drivers & CALL :Link "pin_mux.o clock_config.o fsl_gpio.o fsl_clock.o fsl_smc.o fsl_assert.o fsl_debug_console.o fsl_common.o serial_manager.o serial_port_uart.o fsl_str.o uart_adapter.o fsl_lpuart.o" )
popd
EXIT /B 0


REM ************************************************************
REM **********               FUNCTIONS                **********
REM ************************************************************
:Compile_Core
ECHO ==================        COMPILE         ==================
REM ----------        COMPILE MAIN SOURCE FILE         ----------
CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH%^
 -mcpu=%MCPU%^
 -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags%^
 %Compiler_Macros%^
 %Include_Directories%^
 %Sources%

REM -----    COMPILE STARTUP AND SYSTEM ASSEMBLY FILES      -----
CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH%^
 -mcpu=%MCPU%^
 -mfpu=%MFPU%^
 -x assembler-with-cpp^
 %Compiler_Flags%^
 %Compiler_Macros%^
 %Include_Directories%^
 ..\RTE\Device\MK82FN256VLL15\startup_MK82F25615.S^
 -x c^
 ..\RTE\Device\MK82FN256VLL15\system_%BOARD%.c

REM ----------       COMPILE OTHER SOURCE FILES        ----------
CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% ..\src\RingBuffer.c

ECHO ============================================================
EXIT /B 0

:Compile_FSL_Drivers
REM ==================        COMPILE         ==================
REM ----------           COMPILE FSL LIBRARY          ----------
REM CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/drivers/fsl_common.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/drivers/fsl_gpio.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/drivers/fsl_clock.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/drivers/fsl_lpuart.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/drivers/fsl_smc.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities/fsl_assert.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities/str/fsl_str.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities/debug_console/fsl_debug_console.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/serial_manager/serial_manager.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/serial_manager/serial_port_uart.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
 C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/uart/uart_adapter.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
  ..\RTE\Device\Board_Support\MK82FN256VLL15\clock_config.c


CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe^
 --target=%TARGET%^
 -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU%^
 -x c^
 %Compiler_Flags% %Compiler_Macros% %Include_Directories%^
  ..\RTE\Device\Board_Support\MK82FN256VLL15\pin_mux.c
REM ============================================================
EXIT /B 0

:Link
ECHO ==================         LINK           ==================
CALL C:\Keil_v5\ARM\ARMCLANG\bin\armlink.exe^
 --cpu=Cortex-M4.fp.sp^
 %Objects% %Common_Linker_Flags%^
 --userlibpath=%Libraries%^
 -o .\%Project_Name%.axf

REM CONVERT OUTPUT TO BINARY
CALL C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe --cpu=Cortex-M4 --bincombined .\%Project_Name%.axf --output=.\%Project_Name%.bin

CALL C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe --text -c *.o --output=./

REM CREATE CORRECT DEBUG INFO
REM F:\Dev_Tools\GNU_Arm_Embedded_Toolchain\bin\arm-none-eabi-objcopy.exe %Project_Name%.axf --update-section ER_RO=main.bin --remove-section=ER_RW  main.gdb.elf
ECHO ============================================================
EXIT /B 0

