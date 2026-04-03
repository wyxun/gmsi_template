@echo off
set MAKE_EXE="D:\0_software\msys64\mingw64\bin\mingw32-make.exe"

if "%1"=="auto" goto auto_flow
if "%1"=="rttv" goto gui_flow

:: Default: Pass all arguments to mingw32-make
%MAKE_EXE% %*
goto :eof

:auto_flow
echo [INFO] Starting Full Auto Build, Flash and Debug sequence...
%MAKE_EXE% clean
%MAKE_EXE%
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Build failed.
    pause
    exit /b %ERRORLEVEL%
)

%MAKE_EXE% flash
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Flashing failed.
    pause
    exit /b %ERRORLEVEL%
)

echo [INFO] Launching RTT Server in background...
start "ANPM RTT Server" cmd /c %MAKE_EXE% rtt

echo [INFO] Waiting for Server to initialize...
ping 127.0.0.1 -n 4 > nul

echo [INFO] Launching RTT Viewer...
start powershell -NoProfile -ExecutionPolicy Bypass -File ".\.agent\workflows\rtt_viewer.ps1"
goto :eof

:gui_flow
echo [INFO] Launching RTT Viewer...
start powershell -NoProfile -ExecutionPolicy Bypass -File ".\.agent\workflows\rtt_viewer.ps1"
goto :eof
