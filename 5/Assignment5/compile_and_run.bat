@echo ***********************  cmake ***********************
mkdir build
cd build
cmake ..
cd ..

MSBuild ./build/Assignment5.sln -p:Configuration=Release

start /d "./build/Release" Assignment5.exe