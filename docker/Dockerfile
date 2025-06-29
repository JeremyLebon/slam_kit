ARG BASE_IMAGE=nvidia/cuda:12.5.1-devel-ubuntu24.04
FROM $BASE_IMAGE

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

    # Set ENV vars
ENV ROS_WS_PATH=ros2_ws


RUN apt-get update
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install --no-install-recommends -y \
    gpg curl libfmt-dev libspdlog-dev libopencv-dev \
    libboost-all-dev libglfw3-dev libmetis-dev 

ARG ROS_DISTRO=jazzy


# install ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
RUN apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-tf2-ros \
    python3-bloom python3-rosdep fakeroot debhelper dh-python ros-${ROS_DISTRO}-rmw-cyclonedds-cpp


WORKDIR /root/ros2_ws
RUN mkdir -p src
RUN rosdep init && rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

COPY slam_kit /root/ros2_ws/src/slam_kit

WORKDIR /root/ros2_ws
COPY  cylconedds_config.xml /root/ros2_ws



#RUN /bin/bash -c "if [[ ${RMW_IMPLEMENTATION} == 'rmw_cyclonedds_cpp' ]]; then mv /${ROS_WS_PATH}/src/cylconedds_config.xml /${ROS_WS_PATH}/cylconedds_config.xml; fi" 
  


# libraries for the velodyne driver
RUN apt-get update \
  && apt-get install -qq -y --no-install-recommends \
    ros-jazzy-diagnostic-updater libpcap-dev

WORKDIR /root/ros2_ws/
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=ON'

RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc 

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
&& echo "set +e" >> /ros_entrypoint.sh \
&& echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /ros_entrypoint.sh \
&& echo "source /root/ros2_ws/install/setup.bash" >> /ros_entrypoint.sh \
&& echo "export GZ_SIM_RESOURCE_PATH=/root/ros2_ws/install/robot_one_sim/share/robot_one_sim/models" >> /ros_entrypoint.sh \
&& echo 'exec "$@"' >> /ros_entrypoint.sh \
&& chmod a+x /ros_entrypoint.sh




 
WORKDIR /root/ros2_ws
  
ENTRYPOINT ["/ros_entrypoint.sh"]

#CMD ["bash"]
