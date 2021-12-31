@echo off
rem https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

set ALACRITTY=F:Dev_Tools/Alacritty/alacritty.exe

rem ENTER: monitor reset halt
rem AT Reset_Handler() ENTER: start
echo %CD%

set VER=1

if [%VER%] EQU [0] (call :VER0)
if [%VER%] EQU [1] (call :VER1)

GOTO :

:VER0
echo ver0
rem ==================================================
rem        OPEN A CMD PROMPT & RUN ARM GBD 
rem ==================================================
start cmd /c F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-gdb.exe -c -nh -ex "target extended-remote localhost:3333" -f ./build/k82f_fc.axf

rem ==================================================
rem         RUN OPENOCD & start GBD SERVER - TELNET
rem ==================================================
F:\Dev_Tools\openocd-0.10.0-15\bin\openocd.exe --debug=2 -f ./debug/openocd.cfg

echo returning
exit /B

:VER1
echo ver1
rem ==================================================
rem        OPEN A CMD PROMPT & RUN ARM GBD 
rem ==================================================

start %ALACRITTY% --command "F:\Dev_Tools\ARMGNU\bin\arm-none-eabi-gdb.exe --quiet -ex \"target extended-remote localhost:3333\" -f ./build/K82F_FlightController.axf"


rem ==================================================
rem         RUN OPENOCD & start GBD SERVER - PIPES
rem ==================================================
call F:\Dev_Tools\openocd-0.10.0-15\bin\openocd.exe --debug=2 -f ./debug/openocd.cfg -c "gdb_port pipe; log_output ./debug/openocd.log"

exit /B

rem ------------------------------------------------------------------------
rem .\debug\FC_k82F_Debug\FC_k82F_Debug.uvprojx


