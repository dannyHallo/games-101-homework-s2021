@echo ***********************  cmake ***********************
mkdir build
cd build
cmake ..
cd ..

MSBuild ./build/Assignment4.sln -p:Configuration=Release

start /d "./build/Release" Assignment4.exe