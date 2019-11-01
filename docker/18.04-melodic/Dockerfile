FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO melodic

SHELL ["/bin/bash", "-c"]

# Install basic packages
RUN apt-get update && apt-get install -y \
        apt-utils \
        ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y \
        ros-melodic-moveit-* \
        ros-melodic-usb-cam \
        ros-melodic-joint-trajectory-controller \
        ros-melodic-effort-controllers \
        ros-melodic-vision-opencv \
        ros-melodic-cv-camera \
        ros-melodic-camera-calibration \
        ros-melodic-opencv-apps \
        ros-melodic-catkin \
        python-catkin-tools \
        python-opencv \
        libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

RUN cd ~ && \
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    cd DynamixelSDK/c/build/linux64/ && make

RUN cd ~ && \
    git clone https://github.com/gbiggs/dynamixel_servo_check.git && \
    cd dynamixel_servo_check && \
    cp -r ~/DynamixelSDK/c/include/dynamixel_sdk/ . && \
    cp ~/DynamixelSDK/c/build/linux64/libdxl_x64_c.so . && \
    mkdir -p build && cd build && cmake ../ && make

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    source /opt/ros/melodic/setup.bash && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && catkin_init_workspace && \
    cd ~/catkin_ws && catkin_make && \
    source ~/catkin_ws/devel/setup.bash && \
    cd ~/catkin_ws/src && \
    git clone https://github.com/arebgun/dynamixel_motor.git && \
    git clone https://github.com/gbiggs/rsj_tutorial_2017_ros_intro.git && \
    git clone https://github.com/gbiggs/crane_plus_arm.git && \
    git clone https://github.com/gbiggs/rsj_2017_pick_and_placer.git && \
    git clone https://github.com/Suzuki1984/rsj_2017_block_finder.git && \
    cd rsj_2017_pick_and_placer && \
    git checkout full_application_version && \
    cd ~/catkin_ws/ && catkin_make

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
