@echo off
::echo cd e:>e:\43012.txt
::echo cd %~dp0>>e:\43012.txt
::echo make program >>e:\43012.txt
set /p="cd e:&&cd %~dp0&&make program"<nul>C:\Users\Felix\program.sh
set /p="cd e:&&cd %~dp0&&make build -j8"<nul>C:\Users\Felix\build.sh
set /p="cd e:&&cd %~dp0"<nul>C:\Users\Felix\folderPath.sh
set ProgramFilePath=C:\Users\Felix\program.sh
set BuildFilePath=C:\Users\Felix\build.sh
set ProjectFilePath=C:\Users\Felix\folderPath.sh
set strOld=\
set strNew=/
 
setlocal enabledelayedexpansion
for /f "tokens=*" %%i in (%ProgramFilePath%) do (
  set "var=%%i"
  if not !var!.==. (
    set "var=!var:%strOld%=%strNew%!"
    ::echo !var!!>>%ProgramFilePath%.bk
    set /p=!var!!<nul>%ProgramFilePath%.bk
  )
)
move /y %ProgramFilePath%.bk %ProgramFilePath%

setlocal enabledelayedexpansion
for /f "tokens=*" %%i in (%BuildFilePath%) do (
  set "var=%%i"
  if not !var!.==. (
    set "var=!var:%strOld%=%strNew%!"
    ::echo !var!!>>%BuildFilePath%.bk
    set /p=!var!!<nul>%BuildFilePath%.bk
  )
)
move /y %BuildFilePath%.bk %BuildFilePath%

setlocal enabledelayedexpansion
for /f "tokens=*" %%i in (%ProjectFilePath%) do (
  set "var=%%i"
  if not !var!.==. (
    set "var=!var:%strOld%=%strNew%!"
    ::echo !var!!>>%ProjectFilePath%.bk
    set /p=!var!!<nul>%ProjectFilePath%.bk
  )
)
move /y %ProjectFilePath%.bk %ProjectFilePath%

::move /y %ProgramFilePath% C:\Users\Felix\43012.sh
::start e:\43012.txt
bash --login -i -c ". ./program.sh"

bash --login -i 




