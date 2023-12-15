# Model Predictive Control for Ground Vehicle Navigation
## Introduction
The steps here will guide the users to setup and install packages to run MPC Controller for groundvehicles (POLARIS GEM E2 Simulator). 
## Dependencies
This Package depends on the following libraries
* Ipopt Solver
* CppAD
* PCL
* Eigen
## Building the package
There are two methods to build the package
* Method 1: Build the dockerfile
* Method 2: Build from Source
## Method 1: Building the dockerfile
```bash
git clone https://github.com/Vnu619/mpc_ros.git
cd mpc_ros
sudo docker build -t ros-mpc .
sudo docker run -it ros-mpc
```
## Method 2: Build from Source
## Installing Dependencies
### Installing Dependencies for Ipopt Solver 
#### Step1:
```bash
sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
```
#### Step2: Installing ASL (Ampl Solver Library)
```bash
mkdir ~/Ipopt_pkg
cd Ipopt_pkg
git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
cd ThirdParty-ASL
sudo ./get.ASL
sudo ./configure
sudo make
sudo make install
cd ..
```
#### Step3: Installing HSL (Harwell Subroutines Library)
```bash
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
cd ThirdParty-HSL
sudo ./get.HSL
sudo ./configure
sudo make
sudo make install
cd ..
```
#### Step4: Installing MUMPS Linear Solver
```bash
git clone https://github.com/coin-or-tools/ThirdParty-MUMPS.git
cd ThirdParty-MUMPS
sudo ./get.MUMPS
sudo ./configure
sudo make
sudo make install
cd ..
```
#### Step5: Installing Ipopt
```bash
git clone https://github.com/coin-or/Ipopt.git
cd Ipopt
mkdir build
cd build
sudo ../configure
sudo make
sudo make test
sudo make install
```
#### Step5: Improve the Environment
```bash
cd /usr/local/include
sudo cp coin-or coin -r
sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3
```
### Installing CppAD
```bash
sudo apt-get install cppad
```
### Installing Eigen and PCL
```bash
sudo apt update
sudo apt install libpcl-dev
sudo apt install libeigen3-dev
```
## Setting up the workspace and MPC packages
```bash
mkdir -p ~/catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Vnu619/mpc_ros.git
cd ..
catkin_make
```
## Running the test Environment
```bash
cd catkin_ws
source devel/setup.bash
roslaunch mpc_gen mpc_agv_sim.launch
```
## Results
# Test result for navigation with no obstacles placed
<img width="540" alt="Screenshot 2023-09-23 at 1 10 03 PM" src="https://github.com/Vnu619/mpc_ros/blob/main/mpc_gen/odom_trajectory_plo-1t.png">
<br>

# Test result for navigation with randomly placed obstacles
<img width="540" alt="Screenshot 2023-09-23 at 1 10 03 PM" src="https://github.com/Vnu619/mpc_ros/blob/main/mpc_gen/odom_trajectory_plot_obs.png">
<br>









