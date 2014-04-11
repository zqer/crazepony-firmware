@ECHO OFF

"C:\Program Files\SEGGER\JLinkARM_V420a\JFlashARM.exe" -openprj stm32f10x.jflash -open ..\cflie.bin,0x8000000 -auto -exit

IF ERRORLEVEL 1 goto ERROR
  goto END

:ERROR

ECHO J-Flash ARM: Error!

:END

