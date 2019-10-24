FROM osrf/ros:kinetic-desktop-full

ENV ROS_DISTRO kinetic

ARG http_proxy="$http_proxy"
ARG https_proxy="$https_proxy"

RUN env && \
    apt clean && \
    apt update && \
    apt install -y ros-kinetic-desktop-full \
                   ros-kinetic-moveit-* \
                   ros-kinetic-dynamixel-motor \
                   ros-kinetic-usb-cam \
                   ros-kinetic-joint-trajectory-controller \
                   ros-kinetic-effort-controllers \
                   ca-certificates \
                   python-rosinstall && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
