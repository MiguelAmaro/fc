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
SET Objects= %Project_Name%.o startup_%BOARD%.o system_%BOARD%.o


REM ==================         TARGET        ==================
SET TARGET=arm-arm-none-eabi
SET DFP=%BOARD%_DFP/12.2.0
SET MCPU=cortex-m4
SET MFPU=fpv4-sp-d16
SET ARCH=armv7-m

 
REM ==================   COMPILER(ARMCLANG)  ================== 
SET Compiler_Common= -c -std=c11 -mfloat-abi=hard -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -gdwarf-3 -O1 -ffunction-sections -fno-common  -fdata-sections  -ffreestanding  -fno-builtin  -mthumb -MD

SET Warnings= -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -Weverything

SET Compiler_Flags= %Compiler_Common% %Warnings%

SET Compiler_Macros= -D__EVAL -D__MICROLIB -D__UVISION_VERSION="531" -D_RTE_ -DCPU_MK82FN256VLL15 -D_RTE_ -DDEBUG -DCPU_MK82FN256VLL15 -DFRDM_K82F -DFREEDOM -DSERIAL_PORT_TYPE_UART="1" -D%Build%

SET Include_Directories= -I../RTE/Board_Support/MKV31F512VLL12 -I../RTE/_K28F -IC:/Users/mAmaro/AppData/Local/Arm/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include -IC:/Users/mAmaro/AppData/Local/Arm/Packs/ARM/CMSIS/5.7.0/CMSIS/DSP/Include -IC:/Users/mAmaro/AppData/Local/Arm/Packs/ARM/CMSIS/5.7.0/CMSIS/DSP/PrivateInclude -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP% -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/lists -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/serial_manager -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/components/uart -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/drivers -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities/debug_console -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities/str -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/utilities -IC:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/%DFP%/Device/Include


REM ==================    LINKER(ARMLINK)   ====================
SET Memory_Layout= --ro-base 0x00000000 --entry 0x00000000 --rw-base 0x1FFF8000 --entry Reset_Handler --first __Vectors

SET Common_Linker_Flags= --library_type=microlib --diag_suppress 6314 --strict --scatter "..\RTE\Device\MK82FN256VLL15\MK82FN256xxx15_flash.scf" --remove --keep=*(.FlashConfig) --summary_stderr --info summarysizes --map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --list "..\debug\%Project_Name%.map"

SET Libraries=


ECHO ************************************************************
ECHO **********              START BUILD               **********
ECHO ************************************************************
pushd build
echo %CD%
IF %Build%==CMSIS_CORE ( ECHO CMSIS_CORE Build & CALL :Compile_Core & CALL :Link ) ELSE ( ECHO FREESCALE Build & CALL :Compile_Core & CALL :Compile_FSL_Drivers & CALL :Link "pin_mux.o clock_config.o fsl_gpio.o fsl_clock.o fsl_smc.o fsl_assert.o fsl_debug_console.o fsl_common.o serial_manager.o serial_port_uart.o fsl_str.o uart_adapter.o fsl_uart.o" )
popd
EXIT /B 0


REM ************************************************************
REM **********               FUNCTIONS                **********
REM ************************************************************
:Compile_Core
ECHO ==================        COMPILE         ================== 
REM ----------        COMPILE MAIN SOURCE FILE         ----------
CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% %Sources%

REM -----    COMPILE STARTUP AND SYSTEM ASSEMBLY FILES      -----
CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x assembler-with-cpp %Compiler_Flags% %Compiler_Macros% %Include_Directories% ..\RTE\Device\MK82FN256VLL15\startup_MK82F25615.S -x c ..\RTE\Device\MK82FN256VLL15\system_%BOARD%.c
ECHO ============================================================
EXIT /B 0

:Compile_FSL_Drivers
REM ==================        COMPILE         ================== 
REM ----------       COMPILE OTHER SOURCE FILES       ----------
REM CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories%

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/drivers/fsl_common.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/drivers/fsl_gpio.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/drivers/fsl_clock.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/drivers/fsl_uart.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/drivers/fsl_smc.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/utilities/fsl_assert.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/utilities/str/fsl_str.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/utilities/debug_console/fsl_debug_console.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/components/serial_manager/serial_manager.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/components/serial_manager/serial_port_uart.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories% C:/Users/mAmaro/AppData/Local/Arm/Packs/NXP/MKV31F51212_DFP/12.2.0/components/uart/uart_adapter.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories%  ..\RTE\Board_Support\MKV31F512VLL12\clock_config.c

CALL C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% -x c %Compiler_Flags% %Compiler_Macros% %Include_Directories%  ..\RTE\Board_Support\MKV31F512VLL12\pin_mux.c
REM ============================================================
EXIT /B 0

:Link
ECHO ==================         LINK           ================== 
CALL C:\Keil_v5\ARM\ARMCLANG\bin\armlink.exe --cpu=Cortex-M4.fp.sp %Objects% %~1 %Common_Linker_Flags% --userlibpath=%Libraries% -o .\%Project_Name%.out

REM CONVERT OUTPUT TO BINARY
CALL C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe --cpu=Cortex-M4 --bincombined .\%Project_Name%.out --output=.\%Project_Name%.bin
CALL C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe --text -c *.o --output=./
ECHO ============================================================
EXIT /B 0


