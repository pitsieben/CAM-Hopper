@echo off
setlocal

rem Installs the complete CamGh package into Grasshopper's user Libraries folder.
rem Keep Rhino closed while installing so CamGh.gha is not locked.

set "SOURCE=%~dp0publish\package"
set "DEST=%APPDATA%\Grasshopper\Libraries\CamGh"

if not exist "%SOURCE%\CamGh.gha" (
    echo ERROR: Could not find "%SOURCE%\CamGh.gha"
    echo Make sure this script stays beside the publish folder.
    pause
    exit /b 2
)

if not exist "%APPDATA%\Grasshopper\Libraries" (
    echo ERROR: Grasshopper's user Libraries folder was not found:
    echo "%APPDATA%\Grasshopper\Libraries"
    pause
    exit /b 3
)

if not exist "%DEST%" mkdir "%DEST%"
if errorlevel 1 (
    echo ERROR: Could not create "%DEST%"
    pause
    exit /b 4
)

echo Installing CamGh to:
echo "%DEST%"
robocopy "%SOURCE%" "%DEST%" /E /COPY:DAT /R:2 /W:2
set "COPY_RESULT=%ERRORLEVEL%"

if %COPY_RESULT% GEQ 8 (
    echo.
    echo ERROR: Copy failed with Robocopy status %COPY_RESULT%.
    pause
    exit /b %COPY_RESULT%
)

echo.
echo CamGh installation files copied successfully.
echo Restart Rhino, then look for the CamGh tab in Grasshopper.
pause
exit /b 0
