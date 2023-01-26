@echo ***********************  cmake ***********************
mkdir build
cd build
cmake ..
cd ..

MSBuild ./build/Assignment6.sln -p:Configuration=Release

start /d "./build/Release" Assignment6.exe