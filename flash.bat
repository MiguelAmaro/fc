@ECHO OFF

REM ------------------------------------------------------
REM ---------            DEFINITIONS           -----------
REM ------------------------------------------------------ 

SET Command_File_Path= res\Flash_Command_File.jlink
SET Script_File_Path= res\Flash.JLinkScript
SET Settings_File_Path= F:\Dev\4coder\Projects\Nordic24L01P\res\JLinkSettings.ini


ECHO ------------------------------------------------------
ECHO -----------         FLASH METHODS          -----------
ECHO ------------------------------------------------------

REM METHOD 1: FLash Programming Software
REM call C:\"Program Files (x86)"\SEGGER\JLink\JLink.exe -SettingsFile %Settings_File_Path% -CommandFile %Command_File_Path%

REM -SettingsFile %Settings_File_Path%
REM -JLinkScriptFile %Script_File_Path%


ECHO METHOD 2: MSC FUNCTIONALITY( Drag and Drop )
@ECHO ON
COPY /b build\K82F_FlightController.bin E:\
@ECHO OFF

PAUSE


