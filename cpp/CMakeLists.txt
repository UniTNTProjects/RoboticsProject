
cmake_minimum_required(VERSION 2.8.3)
project(robot_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs computer_vision)
find_package(Eigen3 3.3 REQUIRED)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

#add_executable(FSM_ass2 src/FSM.cpp src/FSM_ass2.cpp src/controller.cpp src/kinematics/trajectory.cpp src/kinematics/direct.cpp src/kinematics/inverse.cpp src/kinematics/jacobian.cpp)
add_executable(FSM_ass2 src/test.cpp src/controller.cpp src/kinematics/trajectory.cpp src/kinematics/direct.cpp src/kinematics/inverse.cpp src/kinematics/jacobian.cpp )

target_link_libraries(FSM_ass2 ${catkin_LIBRARIES})
install(TARGETS FSM_ass2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

