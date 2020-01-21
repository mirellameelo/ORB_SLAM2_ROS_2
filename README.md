# ubuntu:18.04

# working directory
ENV HOME /root

# general utilities
    wget 
    curl 
    git 
    vim 
    nano 
    python-dev 
    python3-pip 
    ipython

# pip
pip3 install --upgrade pip


#### ROS2 SETUP

# Locale options

apt-get install -y locales
locale-gen en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8



# setup sources

apt-get install -y lsb-release
curl http://repo.ros2.org/repos.key | apt-key add -
sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'



# ROS setup requirements

apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool
  
  python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures



# install additional ubuntu 18.04 requirements

python3 -m pip install -U \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools



# install Fast-RTPS dependencies
apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \  
  libopensplice67



# create ros2 sdk workspace

mkdir -p $HOME/ros2_sdk/src
cd $HOME/ros2_sdk
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs import src < ros2.repos




# initialize rosdep and install dependencies

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"



# build the workspace
colcon build --symlink-install



# OPENCV3



# ORBSLAM2 SETUP

cd $HOME
apt-get -qq update 
apt-get install -q -y software-properties-common



# install pangolin depenendencies

apt-get install -y \
  cmake \
  libgtk-3-dev \
  libglew-dev \
  libgl1-mesa-dev \
  pkg-config \
  libpython2.7-dev \
  ffmpeg

# Eigen 

git clone -b 3.2 --single-branch  https://gitlab.com/libeigen/eigen.git
mkdir -p $HOME/eigen/build
cd $HOME/eigen/build
cmake ..
make install



# get pangolin sources
cd $HOME
git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir -p $HOME/Pangolin/build



# build pangolin
cd $HOME/Pangolin/build
cmake ..
cmake --build .



# ORB-SLAM

# install orbslam depenendencies
apt-get install -y \
  libboost-dev \
  libboost-system-dev \
  libcanberra-gtk-module 
  
cd $HOME

# get orbslam sources
git clone https://github.com/raulmur/ORB_SLAM2.git

# build orbslam
cd $HOME/ORB_SLAM2

rm CMakeLists.txt
_______________
cp $HOME/scripts/CMakeLists.txt .
________________

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

# Uncompress vocabulary
cd ../../../
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz











#"Configuring and building ORB_SLAM2
cd ..
mkdir build
cd build
cmake .. \
  -DROS_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
  -DCMAKE_CXX_STANDARD_LIBRARIES="-lboost_system"

make -j
make install











##### ORBSLAM ROS2

# create a ros2 workspace
mkdir -p $HOME/ws/src
cd $HOME/ws

# get source code
git clone https://github.com/alsora/ros2-ORB_SLAM2 src/ros2-ORB_SLAM2
git clone -b ros2 https://github.com/ros-perception/vision_opencv.git src/vision_opencv
git clone https://github.com/ros2/message_filters src/message_filters


# build the workspace
/bin/bash -c 'source $HOME/ros2_sdk/install/setup.sh; \

#tirar
  export LD_LIBRARY_PATH=~/Pangolin/build/src/:~/ORB_SLAM2/Thirdparty/DBoW2/lib:~/ORB_SLAM2/Thirdparty/g2o/lib:$LD_LIBRARY_PATH; \
  
  
  colcon build'




#### SET ENVIRONMENT


cd $HOME

echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source $HOME/ros2_sdk/install/setup.sh \n\
source $HOME/ws/install/local_setup.sh' >> $HOME/.bashrc


# add orbslam shared libraries to path
RUN echo ' \n\
export LD_LIBRARY_PATH=~/Pangolin/build/src/:~/ORB_SLAM2/Thirdparty/DBoW2/lib:~/ORB_SLAM2/Thirdparty/g2o/lib:~/ORB_SLAM2/lib:$LD_LIBRARY_PATH' >> $HOME/.bashrc

ros2 run orbslam mono ~/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/turtlebot/ORB_SLAM2/astra.yaml 
