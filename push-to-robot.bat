@echo off
REM Double-click to push code to the robot (see push-to-robot.ps1).
REM -KeepVpn: Mullvad stays connected (needs Mullvad's "Local network sharing" turned ON).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0push-to-robot.ps1" -KeepVpn %*
pause
