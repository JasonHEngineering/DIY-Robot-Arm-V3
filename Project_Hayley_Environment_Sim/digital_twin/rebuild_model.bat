@echo off
rem ------------------------------------------------------------------
rem  Rebuild the twin's robot model after a CAD change:
rem    cad_source\Robot_Assembly.step  +  cad_tools\link_overrides.json
rem      -> description\meshes\*.stl, hayley_v3.urdf
rem      -> copied into the Raspberry Pi app (twin_web\description)
rem  Then refresh the twin page (Ctrl+F5).
rem ------------------------------------------------------------------
setlocal
cd /d "%~dp0cad_tools"
set "FCCMD=%LOCALAPPDATA%\Programs\FreeCAD 1.1\bin\freecadcmd.exe"
if not exist "%FCCMD%" set "FCCMD=%ProgramFiles%\FreeCAD 1.1\bin\freecadcmd.exe"
if not exist "%FCCMD%" ( echo FreeCAD 1.1 freecadcmd.exe not found - edit FCCMD in this file. & pause & exit /b 1 )

set "HAYLEY_TOOLS_DIR=%CD%"
echo [1/2] Splitting Robot_Assembly.step into link meshes (FreeCAD, ~1 min)...
"%FCCMD%" -c "exec(open(r'%CD%\export_links.py').read())" 2>&1 | findstr /b /c:"wrote" /c:"done" /c:"excluded" /c:"base frame" /c:"Traceback" /c:"link_overrides" /c:"Error"
if errorlevel 2 goto :err

echo [2/2] Generating URDF and copying to the Raspberry Pi app...
python make_urdf.py || goto :err
echo.
echo Done. Refresh http://localhost:8050/twin/ with Ctrl+F5 (restart run_twin_sim.bat not needed).
pause
exit /b 0

:err
echo Rebuild failed - see messages above.
pause
exit /b 1
