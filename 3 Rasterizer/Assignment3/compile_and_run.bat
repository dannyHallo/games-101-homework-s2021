@echo ***********************  cmake ***********************
mkdir build
cd build
cmake ..
cd ..

MSBuild ./build/Assignment3.sln -p:Configuration=Release

start /d "./build/Release" Assignment3.exe