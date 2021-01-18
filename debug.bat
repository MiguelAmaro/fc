@ECHO OFF

REM ENTER: monitor reset halt
REM AT Reset_Handler() ENTER: start
ECHO %CD%

REM https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
REM ==================================================
REM        OPEN A CMD PROMPT & RUN ARM GBD 
REM ==================================================

REM START cmd /k "F:\Dev_Tools\GNU_Arm_Embedded_Toolchain\bin\arm-none-eabi-gdb.exe --quiet -nh -ex "target extended-remote localhost:3333" -f ./build/K82F_FlightController.axf"

REM ==================================================
REM         RUN OPENOCD & START GBD SERVER - TELNET
REM ==================================================
REM CALL F:\Dev_Tools\openocd-0.10.0-15\bin\openocd.exe --debug=0

REM ------------------------------------------------------------------------

REM ==================================================
REM        OPEN A CMD PROMPT & RUN ARM GBD 
REM ==================================================

REM START cmd /k "F:\Dev_Tools\GNU_Arm_Embedded_Toolchain\bin\arm-none-eabi-gdb.exe --quiet -ex "target extended-remote localhost:3333" -f ./build/K82F_FlightController.axf"


REM ==================================================
REM         RUN OPENOCD & START GBD SERVER - PIPES
REM ==================================================
REM CALL F:\Dev_Tools\openocd-0.10.0-15\bin\openocd.exe --debug=2 -f ./debug/openocd.cfg -c "gdb_port pipe; log_output ./debug/openocd.log"

REM ------------------------------------------------------------------------
.\debug\FC_k82F_Debug\FC_k82F_Debug.uvprojx

PAUSE



