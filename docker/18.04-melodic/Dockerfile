FROM osrf/ros:melodic-desktop-full

ENV ROS_DISTRO melodic

ARG http_proxy="$http_proxy"
ARG https_proxy="$https_proxy"

RUN env && \
    apt-get clean && \
    apt-get update && \
    apt-get install -y apt-utils \
                       ca-certificates \
                       python-rosinstall && \
                       ros-melodic-moveit-* \
                       ros-melodic-dynamixel-motor \
                       ros-melodic-usb-cam \
                       ros-melodic-joint-trajectory-controller \
                       ros-melodic-effort-controllers && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
