cd ~/
echo "Setting up Serial Lib"
if [ ! -d "CppLinuxSerial" ]; then
    git clone https://github.com/gbmhunter/CppLinuxSerial.git
fi
mkdir -p CppLinuxSerial/build && cd CppLinuxSerial/build
cmake .. -DBUILD_TESTS=FALSE
sudo make install
cd 
echo "Setting up coils lib"
if [ ! -d "coil_libs" ]; then
    git clone https://github.com/VFrancescon/coil_libs.git
fi
mkdir -p coil_libs/build && cd coil_libs/build
cmake ..
sudo make install
