rm -rf build
mkdir build
cd build
cmake ../ -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cp ./compile_commands.json ../
cd ../
