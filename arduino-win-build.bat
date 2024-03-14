@ECHO OFF
SETLOCAL ENABLEDELAYEDEXPANSION
setlocal
REM go to the folder where this bat script is located
cd /d %~dp0

set /a EXPECTED_CLI_MAJOR=0
set /a EXPECTED_CLI_MINOR=33

set ARDUINO_CORE=arduino:mbed_nicla
set BOARD=arduino:mbed_nicla:nicla_vision
set MBED_VERSION=3.4.1
set ARDUINO_CLI=arduino-cli
set BUILD_OPTION=--build
set FLASH_OPTION=--flash
set ALL_OPTION=--all

IF [%1]==[] (
    GOTO NOPARAMETER
)

set COMMAND=%1

FOR %%I IN (.) DO SET DIRECTORY_NAME=%%~nI%%~xI

where /q arduino-cli
IF ERRORLEVEL 1 (
    GOTO NOTINPATHERROR
)

REM parse arduino-cli version
FOR /F "tokens=1-3 delims==." %%I IN ('arduino-cli version') DO (
    FOR /F "tokens=1-3 delims== " %%X IN ('echo %%I') DO (
        set /A CLI_MAJOR=%%Z
    )
    SET /A CLI_MINOR=%%J
    FOR /F "tokens=1-3 delims== " %%X IN ('echo %%K') DO (
        set /A CLI_REV=%%X
    )
)

if !CLI_MINOR! LSS !EXPECTED_CLI_MINOR! (
    GOTO UPGRADECLI
)

if !CLI_MAJOR! NEQ !EXPECTED_CLI_MAJOR! (
    echo You're using an untested version of Arduino CLI, this might cause issues (found: %CLI_MAJOR%.%CLI_MINOR%.%CLI_REV%, expected: %EXPECTED_CLI_MAJOR%.%EXPECTED_CLI_MINOR%.x )
) else (
    if !CLI_MINOR! NEQ !EXPECTED_CLI_MINOR! (
        echo You're using an untested version of Arduino CLI, this might cause issues (found: %CLI_MAJOR%.%CLI_MINOR%.%CLI_REV%, expected: %EXPECTED_CLI_MAJOR%.%EXPECTED_CLI_MINOR%.x )
    )
)

echo Finding Arduino Mbed core...
(arduino-cli core list  2> nul) | findstr /r "%ARDUINO_CORE% *%MBED_VERSION%"
IF %ERRORLEVEL% NEQ 0 (
    GOTO INSTALLMBEDCORE
)
:AFTERINSTALLMBEDCORE

(arduino-cli lib list VL53L1X 2> nul) | findstr "VL53L1X 1.3.1"
IF %ERRORLEVEL% NEQ 0 (
    GOTO INSTALLPROXIMITY
)
:AFTERINSTALLPROXIMITY

(arduino-cli lib list STM32duino_LSM6DSOX 2> nul) | findstr "LSM6DSOX 2.3.0"
IF %ERRORLEVEL% NEQ 0 (
    GOTO INSTALLINERTIAL
)
:AFTERINSTALLINERTIAL

:: define and include
set DEFINE=-DMBED_HEAP_STATS_ENABLED=1 -DMBED_STACK_STATS_ENABLED=1 -O3 -g3 -DEIDSP_QUANTIZE_FILTERBANK=0 -DEI_CLASSIFIER_SLICES_PER_MODEL_WINDOW=4 -DEIDSP_USE_CMSIS_DSP=1 -DEIDSP_LOAD_CMSIS_DSP_SOURCES=1 -DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
set INCLUDE=-I.\\src\\ -I.\\src\\model-parameters\\ -I.\\src\\inference\\ -I.\\src\\ingestion-sdk-c\\ -I.\\src\\ingestion-sdk-c\\inc\\signing\\ -I.\\src\\ingestion-sdk-platform\\nicla-vision\\ -I.\\src\\sensors\\ -I.\\src\\sensors\\ -I.\\src\\mbedtls_hmac_sha256_sw\\ -I.\\src\\firmware-sdk\\ -I.\\src\\firmware-sdk\\at-server\\

rem CLI v0.14 updates the name of this to --build-property
set BUILD_PROPERTIES_FLAG=--build-property

:: just build
IF %COMMAND% == %BUILD_OPTION% goto :BUILD

echo Finding Arduino Mbed core OK

echo Finding Arduino Nicla Vision...

set COM_PORT=""

for /f "tokens=1" %%i in ('arduino-cli board list ^| findstr "Arduino Nicla Vision"') do (
    set COM_PORT=%%i
)

IF %COM_PORT% == "" (
    GOTO NOTCONNECTED
)

echo Finding Arduino Nicla Vision OK at %COM_PORT%

IF %COMMAND% == %FLASH_OPTION% goto :FLASH

IF %COMMAND% == %ALL_OPTION% goto :ALL else goto :COMMON_EXIT

echo No valid command

goto :COMMON_EXIT

:BUILD
    echo Building %PROJECT%
    %ARDUINO_CLI% compile --fqbn %BOARD% %BUILD_PROPERTIES_FLAG% "build.extra_flags=%DEFINE% %INCLUDE%" --output-dir .
goto :COMMON_EXIT

:FLASH
    echo Flashing %PROJECT%
    CALL %ARDUINO_CLI% upload -p %COM_PORT% --fqbn %BOARD%  --input-dir .
goto :COMMON_EXIT

:ALL
    echo Building %PROJECT%
    %ARDUINO_CLI% compile --fqbn %BOARD% %BUILD_PROPERTIES_FLAG% "build.extra_flags=%DEFINE% %INCLUDE%" --output-dir .
    echo Flashing %PROJECT%
    CALL %ARDUINO_CLI% upload -p %COM_PORT% --fqbn %BOARD%  --input-dir .
goto :COMMON_EXIT


IF %ERRORLEVEL% NEQ 0 (
    GOTO FLASHINGFAILEDERROR
)

echo Flashed your Arduino Nicla Vision development board
echo To set up your development with Edge Impulse, run 'edge-impulse-daemon'
echo To run your impulse on your development board, run 'edge-impulse-run-impulse'

@pause
exit /b 0

:NOTINPATHERROR
echo Cannot find 'arduino-cli' in your PATH. Install the Arduino CLI before you continue
echo Installation instructions: https://arduino.github.io/arduino-cli/latest/
@pause
exit /b 1

:INSTALLMBEDCORE
echo Installing Arduino Mbed core...
arduino-cli core update-index
arduino-cli core install %ARDUINO_CORE%@%MBED_VERSION%
echo Installing Arduino Mbed core OK
GOTO AFTERINSTALLMBEDCORE

:INSTALLPROXIMITY
echo Installing VL53L1X...
arduino-cli lib update-index
arduino-cli lib install VL53L1X@1.3.1
echo Installing VL53L1X OK
GOTO AFTERINSTALLPROXIMITY

:INSTALLINERTIAL
echo Installing STM32duino LSM6DSOX...
arduino-cli lib update-index
arduino-cli lib install "STM32duino LSM6DSOX"@2.3.0
echo Installing STM32duino LSM6DSOX OK
GOTO AFTERINSTALLINERTIAL

:NOTCONNECTED
echo Cannot find a connected Arduino Nicla Vision development board via 'arduino-cli board list'
echo If your board is connected, double-tap on the RESET button to bring the board in recovery mode
@pause
exit /b 1

:UPGRADECLI
echo You need to upgrade your Arduino CLI version (now: %CLI_MAJOR%.%CLI_MINOR%.%CLI_REV%, but required: %EXPECTED_CLI_MAJOR%.%EXPECTED_CLI_MINOR%.x or higher)
echo See https://arduino.github.io/arduino-cli/installation/ for upgrade instructions
@pause
exit /b 1

:FLASHINGFAILEDERROR
echo Flashing failed. Here are some options:
echo If your error is 'incorrect FQBN' you'll need to upgrade the Arduino core via:
echo      $ arduino-cli core update-index
echo      $ arduino-cli core install %ARDUINO_CORE%@%MBED_VERSION%
echo Otherwise, double tap the RESET button to load the bootloader and try again
@pause
exit /b %ERRORLEVEL%

:NOPARAMETER
echo No arguments, pleaase invoke with --build or --flash or --all. See README.md for instructions
exit /b 1

:COMMON_EXIT
