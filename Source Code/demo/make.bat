@echo off

cls
mkdir build
pushd build
del audio.exe
cl /Zi -I..\include ..\*.cpp ..\*.c ..\libs\*.lib msvcrt.lib kernel32.lib shell32.lib user32.lib gdi32.lib opengl32.lib
popd
