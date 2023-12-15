# Use ROS Noetic official image as the base image
FROM ros:noetic

# Install dependencies
RUN apt-get update && apt-get install -y \
    gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev \
    cppad libpcl-dev libeigen3-dev

# Clone and install ThirdParty dependencies and Ipopt
RUN mkdir ~/Ipopt_pkg && cd ~/Ipopt_pkg && \
    git clone https://github.com/coin-or-tools/ThirdParty-ASL.git && \
    cd ThirdParty-ASL && ./get.ASL && ./configure && make && make install && \
    cd .. && \
    git clone https://github.com/coin-or-tools/ThirdParty-HSL.git && \
    cd ThirdParty-HSL && ./get.HSL && ./configure && make && make install && \
    cd .. && \
    git clone https://github.com/coin-or-tools/ThirdParty-MUMPS.git && \
    cd ThirdParty-MUMPS && ./get.MUMPS && ./configure && make && make install && \
    cd .. && \
    git clone https://github.com/coin-or/Ipopt.git && \
    cd Ipopt && mkdir build && cd build && ../configure && make && make test && make install

# Environment setup
RUN cd /usr/local/include && \
    cp -r coin-or coin && \
    ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3 && \
    ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2 && \
    ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3

# Setup the workspace and MPC packages
RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    git clone https://github.com/Vnu619/mpc_ros.git && \
    cd ~/catkin_ws && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Source Workspace
RUN cd ~/catkin_ws \
    source devel/setup.bash

# Set up the entrypoint
CMD ["roslaunch", "mpc_gen", "mpc_agv_sim.launch"]

