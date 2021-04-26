@ECHO OFF
REM https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

REM ENTER: monitor reset halt
REM AT Reset_Handler() ENTER: start
ECHO %CD%

SET VER=0

IF [%VER%] EQU [0] (CALL :VER0)
IF [%VER%] EQU [1] (CALL :VER1)

GOTO :

:VER0
ECHO ver0
REM ==================================================
REM        OPEN A CMD PROMPT & RUN ARM GBD 
REM ==================================================
START cmd /c F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-gdb.exe ^
-nh ^
-ex "target extended-remote localhost:3333" ^
-f ./build/K82F_FlightController.axf

REM ==================================================
REM         RUN OPENOCD & START GBD SERVER - TELNET
REM ==================================================
F:\Dev_Tools\openocd-0.10.0-15\bin\openocd.exe --debug=2 -f ./debug/openocd.cfg

ECHO returning
EXIT /B

:VER1
ECHO ver1
REM ==================================================
REM        OPEN A CMD PROMPT & RUN ARM GBD 
REM ==================================================

START cmd /k "F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-gdb.exe --quiet -ex "target extended-remote localhost:3333" -f ./build/K82F_FlightController.axf"


REM ==================================================
REM         RUN OPENOCD & START GBD SERVER - PIPES
REM ==================================================
CALL F:\Dev_Tools\openocd-0.10.0-15\bin\openocd.exe --debug=2 -f ./debug/openocd.cfg -c "gdb_port pipe; log_output ./debug/openocd.log"

EXIT /B

REM ------------------------------------------------------------------------
REM .\debug\FC_k82F_Debug\FC_k82F_Debug.uvprojx


