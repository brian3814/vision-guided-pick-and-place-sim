@echo off
REM Run a command with ROS 2 Jazzy for Windows (C:\pixi_ws) and this workspace sourced:
REM   ros2_ws\ros2env.bat ros2 topic list
REM   ros2_ws\ros2env.bat ros2 launch pick_place_ros locator.launch.py
REM With no command it opens a shell in that environment.
REM Fast DDS is the middleware, as Isaac Sim must use too (launch it with RMW_IMPLEMENTATION=rmw_fastrtps_cpp).
setlocal
set "PIXI=%USERPROFILE%\.pixi\bin\pixi.exe"
set PIXI_NO_TELEMETRY=1
set "WS=%~dp0"
REM pixi.toml sets Qt's plugin path from the current directory, so activate from C:\pixi_ws
pushd C:\pixi_ws
"%PIXI%" shell-hook --shell cmd > "%TEMP%\pixi_ros2_hook.bat"
call "%TEMP%\pixi_ros2_hook.bat"
call C:\pixi_ws\ros2-windows\local_setup.bat
popd
if exist "%WS%install\local_setup.bat" call "%WS%install\local_setup.bat"
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if "%~1"=="" (
    cmd /k
) else (
    %*
)
