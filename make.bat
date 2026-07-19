@echo off
:: Toolchain auto-detection: honor a predefined MAKE_EXE first, then probe
:: the two known msys64 install roots, otherwise fail with a clear hint.
if not defined MAKE_EXE (
    if exist "D:\0_software\msys64\mingw64\bin\mingw32-make.exe" (
        set "MAKE_EXE=D:\0_software\msys64\mingw64\bin\mingw32-make.exe"
    ) else if exist "D:\software\msys64\mingw64\bin\mingw32-make.exe" (
        set "MAKE_EXE=D:\software\msys64\mingw64\bin\mingw32-make.exe"
    ) else (
        echo [ERROR] mingw32-make.exe not found under D:\0_software or D:\software.
        echo         Set the MAKE_EXE environment variable to its full path.
        exit /b 1
    )
)

if "%1"=="auto" goto auto_flow
if "%1"=="rttv" goto gui_flow

:: Default: Pass all arguments to mingw32-make
"%MAKE_EXE%" %*
goto :eof

:auto_flow
:: Kill old OpenOCD first to avoid port/device conflicts
taskkill /F /IM openocd.exe /T 2>nul
taskkill /F /IM openocd-at32.exe /T 2>nul

:: Apply extra make variables as env vars
if not "%2"=="" set %2
echo [INFO] Starting Full Auto Build, Flash and Debug sequence...
"%MAKE_EXE%" clean
"%MAKE_EXE%" BUILD=debug
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Build failed.
    pause
    exit /b %ERRORLEVEL%
)

"%MAKE_EXE%" flash BUILD=debug
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Flashing failed.
    pause
    exit /b %ERRORLEVEL%
)

echo [INFO] Launching RTT Server in background (no popup)...
:: Use start /b for background execution without a new window
start /b "" "%MAKE_EXE%" rtt BUILD=debug > openocd_rtt.log 2>&1

echo [INFO] Waiting for Server to initialize...
ping 127.0.0.1 -n 4 > nul

:: User uses SuperWaveform, so we skip the default RTT Viewer popup
:: echo [INFO] Launching RTT Viewer...
:: start powershell -NoProfile -ExecutionPolicy Bypass -File ".\tools\rtt_viewer.ps1"
goto :eof

:gui_flow
echo [INFO] Launching RTT Viewer...
start powershell -NoProfile -ExecutionPolicy Bypass -File ".\tools\rtt_viewer.ps1"
goto :eof
