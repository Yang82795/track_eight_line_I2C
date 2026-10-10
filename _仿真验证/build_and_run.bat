@echo off
REM ===========================================================================
REM  避障逻辑宿主机仿真验证  (Windows + g++ 环境)
REM  用真实的 motor_car.cpp / line_pid.cpp / avoid_obstacle.cpp 配虚拟时钟
REM  和脚本化传感器运行 20 项断言，验证避障判定、转向方向、回线与"不倒转"。
REM  作用：改完避障代码后，先在这里跑一遍再烧板子，避免烧板子试错。
REM
REM  用法：把本文件夹拷到一个【纯英文路径】( mingw 不认中文路径 )，然后运行
REM        build_and_run.bat
REM  需要：任意 g++ (如 VSCode 自带的 mingw-w64 g++)，无需 Arduino 环境
REM ===========================================================================
setlocal
cd /d "%~dp0"

echo === 编译 ===
g++ -std=gnu++11 -Wall -Wextra -c ..\motor_car.cpp       -o motor_car.o       -I stub -I .. || goto :err
g++ -std=gnu++11 -Wall -Wextra -c ..\line_pid.cpp        -o line_pid.o        -I stub -I .. || goto :err
g++ -std=gnu++11 -Wall -Wextra -c ..\avoid_obstacle.cpp  -o avoid_obstacle.o  -I stub -I .. || goto :err
g++ -std=gnu++11 -Wall -Wextra -c sim_main.cpp            -o sim_main.o        -I stub -I .. || goto :err
g++ -std=gnu++11 motor_car.o line_pid.o avoid_obstacle.o sim_main.o -o sim.exe || goto :err

echo.
echo === 运行 ===
sim.exe
echo.
pause
exit /b 0

:err
echo *** 编译失败
pause
exit /b 1