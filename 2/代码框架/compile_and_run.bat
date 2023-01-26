@echo ***********************  cmake ***********************
mkdir build
cd build
cmake ..
cd ..

MSBuild ./build/Rasterizer.sln -p:Configuration=Release

start /d "./build/Release" Rasterizer.exe