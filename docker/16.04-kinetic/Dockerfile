FROM ubuntu:xenial

ENV ROS_DISTRO kinetic

ENV http_proxy $http_proxy
ENV https_proxy $https_proxy

RUN env && \
    apt clean && \
    apt update && \
    apt install -y lsb-release \
                   apt-utils && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key del F42ED6FBAB17C654 && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt update && \
    apt install -y ros-$(ROS_DISTRO)-desktop-full \
                   ros-$(ROS_DISTRO)-moveit-* \
                   ros-$(ROS_DISTRO)-dynamixel-motor \
                   ros-$(ROS_DISTRO)-usb-cam \
                   ros-$(ROS_DISTRO)-joint-trajectory-controller \
                   ros-$(ROS_DISTRO)-effort-controllers \
                   python-rosinstall && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/$(ROS_DISTRO)/setup.bash" >> ~/.bashrc

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
