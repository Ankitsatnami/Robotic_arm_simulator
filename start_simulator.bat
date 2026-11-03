@echo off
title Robotic Arm Simulator Launcher
cd /d "%~dp0"

echo ==========================================================
echo        FANUC COBOT KINEMATICS PLAYGROUND (6-DOF)
echo ==========================================================
echo.
echo Starting local web server on http://localhost:8080 ...
echo.

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0server.ps1"

pause
