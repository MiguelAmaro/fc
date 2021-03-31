@ECHO OFF

REM ENTER: monitor reset halt
REM AT Reset_Handler() ENTER: start
ECHO %CD%

REM https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
REM ==================================================
REM        OPEN A CMD PROMPT & RUN ARM GBD 
REM ==================================================
START cmd /k "F:\Dev_Tools\GNU_Arm_Embedded_Toolchain\bin\arm-none-eabi-gdb.exe --quiet --nx -ex "target extended-remote localhost:3333" -f ./build/K82F_FlightController.axf"

REM ==================================================
REM         RUN OPENOCD & START GBD SERVER 
REM ==================================================
CALL F:\Dev_Tools\openocd-0.10.0-15\bin\openocd.exe --debug=0



PAUSE



