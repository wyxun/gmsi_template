@echo off
setlocal

set "ENC_CC=%~1"
set "ENC_CFLAGS=%~2"

set "PROBE_OBJ=compile_pass_motor_type.o"
set "FIXTURE_OBJ=compile_fail_motor_member_access.o"
set "FIXTURE_LOG=compile_fail_motor_member_access.log"

%ENC_CC% --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: compiler health check failed
    call :cleanup
    exit /b 2
)

%ENC_CC% %ENC_CFLAGS% -c compile_pass_motor_type.c -o %PROBE_OBJ%
if errorlevel 1 (
    echo ERROR: positive motor type probe failed
    call :cleanup
    exit /b 2
)
del /q %PROBE_OBJ% >nul 2>&1

%ENC_CC% %ENC_CFLAGS% -c compile_fail_motor_member_access.c ^
    -o %FIXTURE_OBJ% 2>%FIXTURE_LOG%
if not errorlevel 1 (
    echo FAIL: motor_handle_t members remain publicly accessible
    call :cleanup
    exit /b 1
)

findstr /R /I /C:"has no member named.*tRt" ^
    /C:"no member named.*tRt" %FIXTURE_LOG% >nul
if not errorlevel 1 (
    echo PASS: motor_handle_t rejects direct member access
    call :cleanup
    exit /b 0
)

echo ERROR: fixture failed for an unexpected reason
type %FIXTURE_LOG%
call :cleanup
exit /b 2

:cleanup
del /q %PROBE_OBJ% %FIXTURE_OBJ% %FIXTURE_LOG% >nul 2>&1
exit /b 0
