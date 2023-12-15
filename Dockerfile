# Use ROS Noetic official image as the base image
FROM ros:noetic
ENV DEBIAN_FRONTEND=noninteractive
ENV XDG_RUNTIME_DIR=/tmp
# Install dependencies
RUN apt-get update && apt-get install -y \
    gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev \
    cppad libpcl-dev libeigen3-dev 
RUN apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-ackermann-msgs \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf \
    ros-noetic-pcl-conversions \
    ros-noetic-gazebo-ros \
    ros-noetic-xacro \
    ros-noetic-velodyne-description \
    ros-noetic-controller-manager ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-rviz

# Clone and install ThirdParty dependencies and Ipopt
RUN mkdir Ipopt_pkg
WORKDIR /Ipopt_pkg
RUN git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
WORKDIR /Ipopt_pkg/ThirdParty-ASL
RUN ./get.ASL && ./configure && make && make install
WORKDIR /Ipopt_pkg
RUN git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
WORKDIR /Ipopt_pkg/ThirdParty-HSL
RUN git clone https://github.com/Vnu619/coinhsl.git
WORKDIR /Ipopt_pkg/ThirdParty-HSL
RUN ./configure && make && make install
WORKDIR /Ipopt_pkg
RUN git clone https://github.com/coin-or-tools/ThirdParty-MUMPS.git
WORKDIR /Ipopt_pkg/ThirdParty-MUMPS
RUN ./get.Mumps && ./configure && make && make install
WORKDIR /Ipopt_pkg
RUN git clone https://github.com/coin-or/Ipopt.git
WORKDIR /Ipopt_pkg/Ipopt
RUN mkdir build 
WORKDIR /Ipopt_pkg/Ipopt/build
RUN ../configure && make && make test && make install

# Environment setup
WORKDIR /usr/local/include
RUN    cp -r coin-or coin && \
    ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3 && \
    ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2 && \
    ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3

# Setup the workspace and MPC packages
RUN mkdir -p ~/catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone https://github.com/Vnu619/mpc_ros.git
WORKDIR /catkin_ws
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin_make'
WORKDIR /catkin_ws

RUN apt-get install -y qt5-default
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install matplotlib
RUN pip3 install pandas







# Set up the entrypoint
#CMD ["roslaunch", "mpc_gen", "mpc_agv_sim.launch"]

