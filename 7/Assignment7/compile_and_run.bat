@echo ***********************  cmake ***********************
mkdir build
cd build
cmake ..
cd ..

@REM MSBuild ./build/Assignment7.sln -p:Configuration=Debug
@REM start /d "./build/Debug" Assignment7.exe

MSBuild ./build/Assignment7.sln -p:Configuration=Release
start /d "./build/Release" Assignment7.exe