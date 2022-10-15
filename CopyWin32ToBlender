@echo off
:: Modify this according to your blender path
SET blenderpath=C:\Program Files\Blender Foundation\Blender 3.3\3.3\python


echo:
echo -- Starting informations -- (please ensure that this is true)
SET pythonex=%blenderpath%\bin\python.exe
echo Blender's python folder is located at %blenderpath%
echo Blender python path is "%pythonex%" 

echo:
echo -- Checking admin rights --
net session >nul 2>&1

if "%ERRORLEVEL%" == "0" (
    echo Admin rights OK
) else (
    echo Please run this program with administrator rights
    exit /B 1
)

echo:
echo -- Obtaining informations --
FOR /F "delims=" %%G IN ('CALL "%pythonex%" "-c" "import sys,os;print(\"{}{}\".format(sys.version_info.major,sys.version_info.minor))" ') do set PYVERSION=%%G
echo Python version major/minor is %PYVERSION%

echo:
echo -- Setting up module --
CALL "%pythonex%" -c "import win32api"
if "%ERRORLEVEL%" == "0" (
    echo Not installing pywin32 because it is already present: OK
) else (
    echo Installing pywin32...
    CALL "%pythonex%" -m pip install pywin32 --user
)


SET sourcelibs=%appdata%\python\python%PYVERSION%\site-packages
SET destlibs=%blenderpath%\lib\site-packages

SET Arr[0]=win32
SET Arr[1]=win32com
SET Arr[2]=win32comext
SET Arr[3]=pywin32_system32
SET Arr[4]=pywin32.pth
SET Arr[5]=pywin32.version
SET Arr[6]=Pythonwin

echo:
echo -- Checking if module files are present --

set "x=0"
:verifyLoop
if not defined Arr[%x%] goto :endVerifyLoop

call SET dataPath=%sourcelibs%\%%Arr[%x%]%%
if exist "%dataPath%\" (
    echo Folder "%dataPath%" Exists
) else (
    if exist "%dataPath%" (
        echo File "%dataPath%" Exists
    ) else (
        echo ERROR: File/Directory %dataPath% should exist but does not
        exit /B 1
    )
)

set /a "x+=1"
GOTO :verifyLoop
:endVerifyLoop

echo:
echo -- Copy --
echo FROM - "%sourcelibs%" - TO - "%destlibs%" -

set "x=0"
:copyLoop
if not defined Arr[%x%] goto :endCopyLoop

call set VALUE=%%Arr[%x%]%%
call SET dataPath=%sourcelibs%\%VALUE%
call SET destPath=%destlibs%\%VALUE%

if exist "%dataPath%\" (
    ::folder
    xcopy /Y /E "%dataPath%" "%destPath%\"
    if not "%ERRORLEVEL%" == "0" (
        echo Error while copying folder "%dataPath%" to "%destPath%\"
        exit /B 1
    )
    echo Successfully copied folder "%dataPath%" to "%destPath%\"
) else (
    ::file
    echo F | xcopy /Y "%dataPath%" "%destPath%"*
    if not "%ERRORLEVEL%" == "0" (
        echo Error while copying file "%dataPath%" to "%destPath%"
        exit /B 1
    )
    echo Successfully copied file "%dataPath%" to "%destPath%"
)

set /a "x+=1"
GOTO :copyLoop

:endCopyLoop

echo:
echo -- Finished --
