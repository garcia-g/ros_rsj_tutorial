FROM osrf/ros:kinetic-desktop-full

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO kinetic

# Install basic packages
RUN apt-get update && apt-get install -y \
    apt-utils \
    ca-certificates \ &&
    rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-moveit-* \
    ros-kinetic-dynamixel-motor \
    ros-kinetic-usb-cam \
    ros-kinetic-joint-trajectory-controller \
    ros-kinetic-effort-controllers \ &&
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
