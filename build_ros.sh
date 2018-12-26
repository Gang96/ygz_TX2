echo "Initializing the TX2"
sudo ~/jetson_clocks.sh
sudo nvpmodel -m 0

cd Thirdparty/DBoW2/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../fast
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo "Building ROS nodes"
cd ../../../ROS/ygz_stereo_inertial
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j4
