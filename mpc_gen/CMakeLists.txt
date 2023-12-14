cmake_minimum_required(VERSION 3.0.2)
project(mpc_gen)
set(CMAKE_CXX_STANDARD 11)


# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14)


#set(sources src/MPC.cpp)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES wasp_m_controller
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})

include_directories(src/Eigen-3.3)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#add_executable(${PROJECT_NAME}_node src/MPC.cpp)
add_executable(${PROJECT_NAME}_node1 src/MPC_Control.cpp include/mpc_gen/MPC_Control.h)




#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES} ipopt
#${PROJECT_NAME}
#)
target_link_libraries(${PROJECT_NAME}_node1
   ${catkin_LIBRARIES} ipopt ${PCL_LIBRARIES}
#${PROJECT_NAME}
)



