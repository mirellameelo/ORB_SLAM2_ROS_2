# Running ORB_SLAM2 with ROS 2
#### Ubuntu: 18.04
#### ROS 2: dashing


## Dependecies

- Eigen 3.2 (upper versions bring up incompatibilities)
- OpenCV
- ROS 2 (dashing)
- ORB_SLAM2
- Pangolin


##### General dependencies

The following dependencies eventually will be necessary. You can install then as you need, or just run the command:

```bash 
sudo apt-get update && sudo apt-get install -y \
	build-essential \
	git \
	wget \
	curl \
	vim \
	nano \
	cmake 
```	

```bash 
sudo apt-get install -y \
	python3-dev \
	python3-pip \
	python3-numpy \
	python-rosdep2 \
	libgtk-3-dev \
	libglew-dev \
	libgl1-mesa-dev \
	pkg-config \
	libpython2.7-dev \
	ffmpeg \
	libboost-dev \
	libboost-system-dev \
	libcanberra-gtk-module \
	software-properties-common
```

## Eigen3

```bash
cd $HOME
git clone -b 3.2 --single-branch  https://gitlab.com/libeigen/eigen.git
mkdir -p $HOME/eigen/build && cd $HOME/eigen/build
cmake ..
sudo make install
```

## Pangolin 

```bash
cd $HOME
git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir -p $HOME/Pangolin/build && cd $HOME/Pangolin/build
cmake ..
cmake --build .
```

## OpenCV

``` bash
cd $HOME
git clone https://github.com/opencv/opencv.git
mkdir -p $HOME/opencv/build && cd $HOME/opencv/build 
cmake ..
make 
sudo make install
```

## ROS 2

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
```

wget, python-rosdep, python3-pip, build-essential, cmake and git are necessary. If you had jumped "general dependencies" section, please, add them in the following command. 
  
 ```bash
sudo apt update && sudo apt install -y \
	python3-colcon-common-extensions \
	python3-vcstool
```
  
``` bash
python3 -m pip install -U \
	pytest \
	pytest-cov \
	pytest-runner \
	setuptools \
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
```

```bash
sudo apt install --no-install-recommends -y \
	libasio-dev \
	libtinyxml2-dev
```

```bash
mkdir -p $HOME/ros2_sdk/src && cd $HOME/ros2_sdk
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs import src < ros2.repos
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
colcon build --symlink-install
```


## ORB_SLAM2_ROS2 

``` bash
mkdir -p $HOME/ws/src && cd $HOME/ws/src
git clone https://github.com/mirellameelo/ORB_SLAM2_ROS_2.git
git clone -b ros2 https://github.com/ros-perception/vision_opencv.git src/vision_opencv
git clone https://github.com/ros2/message_filters src/message_filters
```

Before executing the following commands, execute all the "ORB_SLAM2" section.

``` bash
source $HOME/ros2_sdk/install/setup.sh 
colcon build
```

## ORB_SLAM2

```bash 
cd $HOME
git clone https://github.com/raulmur/ORB_SLAM2.git
cd $HOME/ORB_SLAM2
rm CMakeLists.txt
cp $HOME/ORB_SLAM2_ROS_2/src/CMakeLists.txt .
rm $HOME/ORB_SLAM2_ROS_2/src/CMakeLists.txt
cd Thirdparty/DBoW2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../../g2o
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../../../
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
# Configuring and building ORB_SLAM2
cd ..
mkdir build && cd build
cmake .. \
	-DROS_BUILD_TYPE=Release \
	-DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
	-DCMAKE_CXX_STANDARD_LIBRARIES="-lboost_system"
make -j
make install
```

## Set the environment

-  You can add the following lines in: $HOME/.bashrc or run it **every time you open a new terminal**

``` bash
source $HOME/ros2_sdk/install/setup.sh
source $HOME/ws/install/local_setup.sh
export LD_LIBRARY_PATH=~/Pangolin/build/src/:~/ORB_SLAM2/Thirdparty/DBoW2/lib:~/ORB_SLAM2/Thirdparty/g2o/lib:~/ORB_SLAM2/lib:$LD_LIBRARY_PATH
```

## Running using a camera (example)

You need to edit the .yaml file according the camera you are using. You can take the .yaml file in $HOME/ORB_SLAM2_ROS_2/src/camera_type  as example, and just change the parameters. 

- Mono

``` bash
ros2 run orbslam mono $HOME/ORB_SLAM2/Vocabulary/ORBvoc.txt YAML_FILE_PATH
```

- RGBD 

``` bash
ros2 run orbslam stereo $HOME/ORB_SLAM2/Vocabulary/ORBvoc.txt PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```

-  Stereo

``` bash
ros2 run orbslam stereo $HOME/ORB_SLAM2/Vocabulary/ORBvoc.txt PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```

Now, the ORB SLAM2 keeps waiting until images be published. You can open another terminal, set the environment and run:

``` bash
ros2 run image_tools cam2image -t camera
```

