@echo off
SetLocal enabledelayedexpansion

:: Speeds
::set A=10 25 40 55 70 85 100
set A=55 70 85 100

:: Weights
set B=45 60 75 90 105 120 135 150

:: Bump Heights
set C=-0.05 -0.04 -0.03 -0.02 -0.01 0.01 0.02 0.03 0.04 0.05

:: Bump Widths
set D=0.33 0.66 1

:: Loop counter
set /a e=1

FOR %%a in (%A%) do (
  FOR %%b in (%B%) DO (
    FOR %%c in (%C%) DO (
      FOR %%d in (%D%) DO (
        echo -----------------------
	  echo Simulation: !e! of 960
        echo -----------------------
        nami_stem.exe %%b %%a square %%c %%d beam 0.35 no
        
        set /a e+=1
    )
   )
  )
)
