@echo off
:: 빌드 커맨드를 실행하고 결과를 체크
call mingw32-make.exe -j4 C:\KC\build-untitled-Desktop_Qt_5_12_8_MinGW_64_bit-Debug || goto :error

:: 빌드가 성공적으로 완료되었으면 git 명령 실행
git add .
git commit -m "Successful build"
git push upstream master
goto :end

:error
echo Build failed, not committing or pushing.
:end