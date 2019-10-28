FROM osrf/ros:kinetic-desktop-full

ENV ROS_DISTRO kinetic

# Install basic packages
RUN apt-get update && apt-get install -y \
    apt-utils \
    ca-certificates

# Install ROS packages
RUN apt-get update && apt-get install -y \
    python-rosinstall \
    python-rosdep \
    ros-kinetic-moveit-* \
    ros-kinetic-dynamixel-motor \
    ros-kinetic-usb-cam \
    ros-kinetic-joint-trajectory-controller \
    ros-kinetic-effort-controllers

RUN rosdep init && \
    rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
