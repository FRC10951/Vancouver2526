@echo off
setlocal
cd /d "%~dp0\.."
call gradlew.bat :constants-editor:run
if errorlevel 1 pause
