@echo off
if not exist build mkdir build

rem FILES
rem ************************************************************
set PROJECT_NAME=fc
set BOARD=MK82F25615
set SOURCES= F:\Dev\Embedded\FlightController_K82F\src\k82f_%PROJECT_NAME%.c
set OBJECTS= k82f_%PROJECT_NAME%.o ringbuffer.o startup_%BOARD%.o system_%BOARD%.o


rem TARGET
rem ************************************************************
set TARGET=arm-arm-none-eabi
set DFP=%BOARD%_DFP\12.2.0
set MCPU=cortex-m4
set MFPU=fpv4-sp-d16
set ARCH=armv7-m


rem BUILD TOOLS
rem ************************************************************
set GCC=F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-gcc.exe
set LD=F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-ld.exe
set OBJDUMP=F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-objdump.exe
rem READELF=F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-readelf.exe

set ARMCLANG=C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe
set ARMLINK=C:\Keil_v5\ARM\ARMCLANG\bin\armlink.exe
set READELF=F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-readelf.exe

rem ************************************************************
rem COMPILER(ARMCLANG) OPTIONS
rem ************************************************************
set ARMCLANG_WARNINGS=^
-Weverything ^
-Wno-pedantic ^
-Wno-packed ^
-Wno-missing-variable-declarations ^
-Wno-unsequenced ^
-Wno-missing-prototypes ^
-Wno-strict-prototypes ^
-Wno-missing-noreturn ^
-Wno-sign-conversion ^
-Wno-nonportable-include-path ^
-Wno-reserved-id-macro ^
-Wno-unused-macros ^
-Wno-unused-parameter ^
-Wno-unused-variable ^
-Wno-documentation-unknown-command ^
-Wno-documentation ^
-Wno-license-management ^
-Wno-parentheses-equality ^
-Wno-shadow

set ARMCLANG_FLAGS=^
-c ^
-std=gnu11 ^
-O0 ^
-gdwarf-3 ^
-mfloat-abi=hard ^
-mthumb ^
-fno-rtti ^
-fno-common ^
-fno-builtin ^
-fshort-enums ^
-fshort-wchar ^
-ffreestanding ^
-fdata-sections ^
-funsigned-char ^
-ffunction-sections ^
%ARMCLANG_WARNINGS% ^
-MD

set ARMCLANG_MACROS=^
-D__EVAL ^
-D__MICROLIB ^
-D__UVISION_VERSION="531" ^
-D_RTE_ ^
-DCPU_MK82FN256VLL15 ^
-D_RTE_ ^
-DDEBUG ^
-DCPU_MK82FN256VLL15 ^
-DFRDM_K82F ^
-DFREEDOM ^
-DSERIAL_PORT_TYPE_UART="1"

set ARMCLANG_INCLUDE_DIRS=^
-I..\src\systems\MK82FN256VLL15 ^
-IC:\Users\mAmaro\AppData\Local\Arm\Packs\ARM\CMSIS\5.7.0\CMSIS\DSP\Include ^
-IC:\Users\mAmaro\AppData\Local\Arm\Packs\ARM\CMSIS\5.7.0\CMSIS\Core\Include ^
-IC:\Users\mAmaro\AppData\Local\Arm\Packs\ARM\CMSIS\5.7.0\CMSIS\DSP\PrivateInclude


rem ************************************************************
rem LINKER(ARMLINK) OPTIONS
rem ************************************************************
set MEMORY=^
--ro-base 0x00000000 ^
--entry 0x00000000 ^
--rw-base 0x1FFF8000 ^
--entry Reset_Handler ^
--first __Vectors

set ARMLINK_FLAGS=^
--library_type=microlib ^
--diag_suppress 6314 ^
--strict ^
--scatter "..\src\systems\MK82FN256VLL15\MK82FN256xxx15_flash.scf" ^
--keep=*(.FlashConfig) ^
--summary_stderr ^
--bestdebug ^
--remove ^
--map ^
--xref ^
--symbols ^
--callgraph ^
--info sizes ^
--info totals ^
--info unused ^
--info veneers ^
--info summarysizes ^
--load_addr_map_info ^
--list "..\debug\%PROJECT_NAME%.map"

set LIBRARIES=


rem ************************************************************
rem START BUILD
rem ************************************************************
set path="F:\Dev\Embedded\FlightController_K82F\build";path

pushd build

echo ==================        COMPILE         ==================
rem  ============================================================
call %ARMCLANG% --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% ^
-x c ^
%ARMCLANG_FLAGS% ^
%ARMCLANG_MACROS% ^
%ARMCLANG_INCLUDE_DIRS% ^
%SOURCES%

call %ARMCLANG% --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% ^
-x assembler-with-cpp ^
%ARMCLANG_FLAGS% ^
%ARMCLANG_MACROS% ^
%ARMCLANG_INCLUDE_DIRS% ^
..\src\systems\MK82FN256VLL15\startup_MK82F25615.S ^
-x c ^
..\src\systems\MK82FN256VLL15\system_%BOARD%.c

rem //~COMPILE OTHER SOURCE FILES
call %ARMCLANG% --target=%TARGET% -march=%ARCH% -mcpu=%MCPU% -mfpu=%MFPU% ^
-x c ^
%ARMCLANG_FLAGS% ^
%ARMCLANG_MACROS% ^
%ARMCLANG_INCLUDE_DIRS% ^
..\src\RingBuffer.c


echo ==================         LINK           ==================
rem  ============================================================
call %ARMLINK% ^
--cpu=Cortex-M4.fp.sp ^
%OBJECTS% %ARMLINK_FLAGS% ^
--userlibpath=%LIBRARIES% ^
-o .\k82f_%PROJECT_NAME%.axf

rem //~CONVERT OUTPUT TO BINARY
call C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe ^
--cpu=Cortex-M4 ^
--bincombined .\k82f_%PROJECT_NAME%.axf ^
--output=.\k82f_%PROJECT_NAME%.bin

call C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe --text -c *.o --output=./

rem //~CREATE CORRECT DEBUG INFO
rem F:\Dev_Tools\GNU_Arm_Embedded_Toolchain\bin\arm-none-eabi-objcopy.exe ^
rem k82f_%PROJECT_NAME%.axf ^
rem --update-section ER_RO=main.bin ^
rem --remove-section=ER_RW  main.gdb.elf

popd

pause

