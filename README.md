![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Version](https://img.shields.io/badge/Version-1.0.0-blue.svg)

## 【[简体中文版在此](README_CN.md)】

# orbslam3_dense_ros2 (Dense mapping extension of ORB-SLAM3)

## Introduction

This project is a modified version of ORB-SLAM3 that adds dense mapping functionality. It was tested using an Intel machine and NVIDIA Jetson Orin NX, and was developed/tested with ROS 2 Humble.

Key points:

- Dense mapping is supported for RGB-D cameras. The provided example uses an Intel RealSense D455/D435 family camera.
- The main ROS2 integration points are `src/ros2_slam_publisher.cpp` and `include/orbslam3_dense_ros2/ros2_slam_publisher.h` — modify these files to adapt the node to your setup.
- `src/main.cpp` contains a usage example. Only RGB-D input supports the dense mapping feature in this repo.

## Demo

Video demo: (not provided here). You can record a demo using your RGB-D camera and the included example node.

## Requirements and Installation

This repository was developed against ROS 2 Humble. Follow the official ROS 2 Humble installation guide if you haven't installed ROS 2 yet:

- https://docs.ros.org/en/humble/Installation.html

Required external libraries and tools (high level):

- Eigen3
- Pangolin
- OpenCV (this project was tested with OpenCV 4.10 built with CUDA support; you may build a CPU-only OpenCV if you prefer)

Below are concise installation steps and notes.

### Install Eigen3

```bash
sudo apt install libeigen3-dev
```

### Install Pangolin (v0.8)

```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
git checkout v0.8
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### Install dependencies for OpenCV

Update packages and install build dependencies:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential cmake pkg-config unzip yasm git checkinstall \
  libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libxvidcore-dev libx264-dev \
  libmp3lame-dev libopus-dev libvorbis-dev ffmpeg libva-dev libdc1394-25 libdc1394-dev \
  libxine2-dev libv4l-dev v4l-utils libgtk-3-dev libtbb-dev libatlas-base-dev gfortran \
  libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev \
  libeigen3-dev libhdf5-dev doxygen
```

Add a soft link required by some systems:

```bash
sudo ln -s /usr/include/libv4l1-videodev.h /usr/include/linux/videodev.h
```

### Build OpenCV (example: 4.10 with CUDA)

Download and unpack OpenCV and opencv_contrib, then configure with CMake. Be sure to adjust the Python install path and CUDA_ARCH_BIN for your GPU.

```bash
cd ~
mkdir -p opencv && cd opencv
wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.10.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.10.0.zip
unzip opencv.zip
unzip opencv_contrib.zip
cd opencv-4.10.0
mkdir build && cd build

# Example cmake configuration — edit paths and CUDA_ARCH_BIN for your system
cmake .. -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D WITH_TBB=ON \
  -D ENABLE_FAST_MATH=1 \
  -D CUDA_FAST_MATH=1 \
  -D WITH_CUBLAS=1 \
  -D WITH_CUDA=ON \
  -D BUILD_opencv_cudacodec=OFF \
  -D WITH_CUDNN=ON \
  -D OPENCV_DNN_CUDA=ON \
  -D CUDA_ARCH_BIN=7.5 \
  -D WITH_V4L=ON \
  -D WITH_QT=OFF \
  -D WITH_OPENGL=ON \
  -D WITH_GSTREAMER=ON \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D OPENCV_PC_FILE_NAME=opencv.pc \
  -D OPENCV_ENABLE_NONFREE=ON \
  -D OPENCV_PYTHON3_INSTALL_PATH=~/.virtualenvs/opencv/lib/python3.12/site-packages/ \
  -D PYTHON_EXECUTABLE=~/.virtualenvs/opencv/bin/python \
  -D OPENCV_EXTRA_MODULES_PATH=~/Downloads/opencv/opencv_contrib-4.10.0/modules \
  -D INSTALL_PYTHON_EXAMPLES=OFF \
  -D INSTALL_C_EXAMPLES=OFF \
  -D BUILD_EXAMPLES=OFF

make -j$(nproc)
sudo make install
sudo /bin/bash -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```

Notes:

- Replace `OPENCV_PYTHON3_INSTALL_PATH` and `PYTHON_EXECUTABLE` with your actual Python virtual environment or system Python paths if building Python bindings.
- If you want a CPU-only OpenCV, omit CUDA-related options.

## Build and run (ROS 2 workspace)

Put this package into your ROS 2 workspace `src` folder or create a new workspace:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/zgfbupt/orbslam3_dense_ros2.git
cd ../
colcon build --packages-select orbslam3_dense_ros2
```

Source the workspace and run the example node:

```bash
source ~/robot_ws/install/setup.bash
ros2 run orbslam3_dense_ros2 orb_slam3_main
```

## References

1. https://github.com/Mechazo11/ros2_orb_slam3
2. https://github.com/LeonardoDiCaprio1/Map_ORBSLAM_ROS/

## License

This project follows the original project's license (GPLv3). See `LICENSE` for details.
