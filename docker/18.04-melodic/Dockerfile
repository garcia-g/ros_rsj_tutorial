FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO melodic

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
    ros-melodic-effort-controllers && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

RUN ls ~ && cd ~/catkin_ws/src && \
    git clone https://github.com/arebgun/dynamixel_motor.git && \
    cd .. && catkin_make

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
