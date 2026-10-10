@echo off
REM ===========================================================================
REM 避障几何参数扫描器
REM 在【含直道 + 90度急弯】的虚拟赛道上暴力搜索参数组合，
REM 用"5 个场景能过几个"当评分，自动找出能绕开障碍并回到线上的参数。
REM
REM 用法：把整个工程拷到纯英文路径后运行 sweep.bat
REM 背景：差速小车横向位移 = 转弯半径 R（硬几何约束），
REM       所以外轮/内轮差速和持续时间必须一起调，不能只调时间。
REM ===========================================================================
setlocal
cd /d "%~dp0"

if not exist ..\track_eight_line_I2C.ino (
  echo 请把工程根目录(有 track_eight_line_I2C.ino 的那层)放到本脚本的上一级
  pause & exit /b 1
)

call :build || goto :err

set BEST=0
set BESTCFG=
for %%O in (62 68 72 76) do (
  for %%I in (24 28 32) do (
    for %%T in (700 900 1100) do (
      for %%P in (700 1000 1300) do (
        set CFG=OUT=%%O/%%I TURN=%%T PASS=%%P
        call :run %%O %%I %%T %%P
        if %%ERRORLEVEL%% GTR 0 (
          for /f "delims=" %%S in ('call :fmt %%O %%I %%T %%P') do set BEST=%%S
          echo ### NEW BEST: %%BEST%%
        )
      )
    )
  )
)

echo.
echo ============================================================
echo   最佳: %BESTCFG%
echo   通过 %BEST% / 5
echo ============================================================
pause
exit /b 0

:fmt
echo %1/%2 TURN=%3 PASS=%4 ^> %1/%2 TURN=%3 PASS=%4 (score %5)
goto :eof

:run
call :build "-DAVOID_TURN_OUT_OUTER=%1" "-DAVOID_TURN_OUT_INNER=%2" "-DAVOID_TURN_OUT_MS=%3" "-DAVOID_PASS_MS=%4" || exit /b 1
track.exe > out.txt 2>&1
set SCORE=0
for /f %%N in ('findstr /c:"到达终点" out.txt') do (
  set S=%%N
  set SCORE=!S!
)
rem 解析 "到达终点 N / 5"
set CNT=0
for /f "tokens=3" %%A in ('findstr /c:"到达终点" out.txt') do set CNT=%%A
echo   [%1/%2 T=%3 P=%4] score=!CNT!
exit /b 0

:build
g++ -std=gnu++11 -O2 -c ..\motor_car.cpp      -o motor_car.o %2 %3 %4 %5 %6 -I stub -I .. > nul 2>&1 || exit /b 1
g++ -std=gnu++11 -O2 -c ..\line_pid.cpp       -o line_pid.o  %2 %3 %4 %5 %6 -I stub -I .. > nul 2>&1 || exit /b 1
g++ -std=gnu++11 -O2 -c ..\avoid_obstacle.cpp -o avoid_obstacle.o %2 %3 %4 %5 %6 -I stub -I .. > nul 2>&1 || exit /b 1
g++ -std=gnu++11 -O2 -c sim_track.cpp -o sim_track.o %2 %3 %4 %5 %6 -I stub -I .. > nul 2>&1 || exit /b 1
g++ -std=gnu++11 motor_car.o line_pid.o avoid_obstacle.o sim_track.o -o track.exe > nul 2>&1 || exit /b 1
exit /b 0

:err
echo *** 构建失败
pause
exit /b 1