#!/bin/bash

set -ue

# Install basic packages
sudo apt-get update
sudo apt-get install -y \
        apt-utils \
        ca-certificates \
        wkhtmltopdf

# Install ROS packages
sudo apt-get install -y \
        ros-kinetic-moveit-* \
        ros-kinetic-dynamixel-motor \
        ros-kinetic-usb-cam \
        ros-kinetic-joint-trajectory-controller \
        ros-kinetic-effort-controllers \
        ros-kinetic-vision-opencv \
        ros-kinetic-cv-camera \
        ros-kinetic-camera-calibration \
        ros-kinetic-opencv-apps \
        ros-kinetic-catkin \
        python-catkin-tools \
        python-opencv \
        libopencv-dev

# Download & install for section-3
rm -rf DynamixelSDK
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
(cd DynamixelSDK/c/build/linux64/ && make)

rm -rf dynamixel_servo_check
git clone https://github.com/takahasi/dynamixel_servo_check.git
(cd dynamixel_servo_check && \
    cp -r ../DynamixelSDK/c/include/dynamixel_sdk/ . && \
    cp ../DynamixelSDK/c/build/linux64/libdxl_x64_c.so . && \
    mkdir -p build && cd build && cmake ../ && make)

# Download & install for section-4
source /opt/ros/kinetic/setup.bash
(mkdir -p catkin_ws/src && \
    cd catkin_ws/src && rm -rf CMakeLists.txt && catkin_init_workspace && \
    cd .. && catkin_make && \
    source devel/setup.bash && \
    cd src && \
    rm -rf rsj_tutorial_2017_ros_intro && \
    git clone https://github.com/gbiggs/rsj_tutorial_2017_ros_intro.git && \
    cd .. && catkin_make)

# Download & install for section-5
source /opt/ros/kinetic/setup.bash
(mkdir -p crane_plus_ws/src && \
    cd crane_plus_ws/src && rm -rf CMakeLists.txt && catkin_init_workspace && \
    cd .. && catkin_make && \
    source devel/setup.bash && \
    cd src && \
    rm -rf crane_plus_arm && \
    git clone https://github.com/gbiggs/crane_plus_arm.git && \
    rm -rf rsj_2017_pick_and_placer && \
    git clone https://github.com/gbiggs/rsj_2017_pick_and_placer.git && \
    cd .. && catkin_make)

# Download & install for section-6&7
source /opt/ros/kinetic/setup.bash
(mkdir -p block_finder_ws/src && \
    cd block_finder_ws/src && rm -rf CMakeLists.txt && catkin_init_workspace && \
    cd .. && catkin_make && \
    source devel/setup.bash && \
    cd src && \
    rm -rf rsj_2017_block_finder && \
    git clone https://github.com/Suzuki1984/rsj_2017_block_finder.git && \
    cd .. && catkin_make)

# Download & install for section-9
source /opt/ros/kinetic/setup.bash
(mkdir -p rsj_2017_application_ws/src && \
    cd rsj_2017_application_ws/src && rm -rf CMakeLists.txt && catkin_init_workspace && \
    cd .. && catkin_make && \
    source devel/setup.bash && \
    cd src && \
    rm -rf rsj_2017_pick_and_placer && \
    git clone https://github.com/gbiggs/rsj_2017_pick_and_placer.git && \
    cd rsj_2017_pick_and_placer && git checkout full_application_version && cd ..\
    rm -rf rsj_2017_block_finder && \
    git clone https://github.com/Suzuki1984/rsj_2017_block_finder && \
    rm -rf crane_plus_arm && \
    git clone https://github.com/gbiggs/crane_plus_arm.git && \
    rm -rf rsj_2017_application && \
    git clone https://github.com/gbiggs/rsj_2017_application.git && \
    cd .. && catkin_make)

# Download all cocuments as pdf
#sudo apt-get install -y wkhtmltopdf
#base_url=https://takahasi.github.io/ros_moveit_rsj_tutorial
#rm -rf docs && mkdir -p docs
#wkhtmltopdf $base_url/index.html docs/index.pdf
#wkhtmltopdf $base_url/linux_and_ros_install.html docs/linux_and_ros_install.pdf
#wkhtmltopdf $base_url/linux_basics.html docs/linux_basics.pdf
#wkhtmltopdf $base_url/manipulator_check.html docs/manipulator_check.pdf
#wkhtmltopdf $base_url/ros_basics.html docs/ros_basics.pdf
#wkhtmltopdf $base_url/manipulators_and_moveit.html docs/manipulators_and_moveit.pdf
#wkhtmltopdf $base_url/camera_check.html docs/camera_check.pdf
#wkhtmltopdf $base_url/image_processing_and_opencv.html docs/image_processing_and_opencv.pdf
#wkhtmltopdf $base_url/ros_useful_stuff.html docs/ros_useful_stuff.pdf
#wkhtmltopdf $base_url/full_application.html docs/full_application.pdf

exit 0
