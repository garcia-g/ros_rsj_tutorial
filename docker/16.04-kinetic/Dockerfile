FROM osrf/ros:kinetic-desktop-full

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO kinetic

# Install basic packages
RUN apt-get update \
    && apt-get install -y \
        apt-utils \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update \
    && apt-get install -y \
        ros-kinetic-moveit-* \
        ros-kinetic-dynamixel-motor \
        ros-kinetic-usb-cam \
        ros-kinetic-joint-trajectory-controller \
        ros-kinetic-effort-controllers \
        ros-kinetic-vision-opencv \
        ros-kinetic-cv-camera \
        ros-kinetic-camera-calibration \
        ros-kinetic-opencv-apps \
        python-opencv \
        libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

RUN mkdir -p ~/catkin_ws/src \
    && cd ~/catkin_ws/src \
    && catkin_init_workspace

RUN cd ~ \
    && git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
    && cd DynamixelSDK/c/build/linux64/ \
    && make

RUN cd ~ \
    && git clone https://github.com/gbiggs/dynamixel_servo_check.git \
    && cd dynamixel_servo_check \
    && cp -r ~/DynamixelSDK/c/include/dynamixel_sdk/ . \
    && cp ~/DynamixelSDK/c/build/linux64/libdxl_x64_c.so . \
    && mkdir -p build \
    && cd build \
    && cmake ../ \
    && make

RUN cd ~/catkin_ws/src \
    && git clone https://github.com/gbiggs/rsj_tutorial_2017_ros_intro.git \
    && git clone https://github.com/gbiggs/crane_plus_arm.git \
    && git clone https://github.com/gbiggs/rsj_2017_pick_and_placer.git \
    && git checkout moveit_pick_place_plugin \
    && cd rsj_2017_pick_and_placer \
    && git checkout full_application_version \
    && cd ~/catkin_ws/src \
    && git clone https://github.com/Suzuki1984/rsj_2017_block_finder.git \
    && cd ~/catkin_ws/ \
    && catkin_make

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
