@echo off
rem ------------------------------------------------------------------
rem  Project Hayley - run the Dash UI + digital twin in SIMULATION mode
rem  (no robot / no I2C needed). Double-click this file.
rem  First run creates a Python environment in %LOCALAPPDATA%\hayley_twin_venv
rem  and installs requirements-sim.txt (needs internet, ~1-2 min).
rem ------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set "VENV=%LOCALAPPDATA%\hayley_twin_venv"

if not exist "%VENV%\Lib\site-packages\dash" (
  echo Creating Python environment in %VENV% ^(first run only^)...
  where uv >nul 2>nul
  if not errorlevel 1 (
    rem uv + Windows certificate store: works behind VPN/antivirus TLS inspection
    if not exist "%VENV%\Scripts\python.exe" uv venv -q --python 3.11 "%VENV%" || goto :err
    uv pip install -q --system-certs --python "%VENV%\Scripts\python.exe" -r requirements-sim.txt || goto :err
  ) else (
    if not exist "%VENV%\Scripts\python.exe" python -m venv "%VENV%" || goto :err
    "%VENV%\Scripts\python.exe" -m pip install -q truststore
    "%VENV%\Scripts\python.exe" -m pip install --use-feature=truststore -r requirements-sim.txt || goto :err
  )
)

rem open the twin in the default browser once the server answers
start "" /min powershell -NoProfile -WindowStyle Hidden -Command ^
  "for($i=0;$i -lt 90;$i++){try{Invoke-WebRequest -UseBasicParsing http://localhost:8050/twin/api/config -TimeoutSec 2 | Out-Null; Start-Process 'http://localhost:8050/twin/'; break}catch{Start-Sleep 1}}"

echo.
echo Starting simulation server ... the twin opens at http://localhost:8050/twin/
echo (Dash UI at http://localhost:8050/ ). Close this window or press Ctrl+C to stop.
echo.
set HAYLEY_SIM=1
"%VENV%\Scripts\python.exe" Project_Hayley_V5_IK_user_interface_V09.py
goto :eof

:err
echo.
echo Setup failed. Check that Python 3.9+ is on PATH and you are online, then run again.
echo To start over, delete the folder %VENV%
pause
