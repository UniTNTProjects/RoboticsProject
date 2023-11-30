
cmake_minimum_required(VERSION 2.8.3)
project(robot_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs computer_vision )
find_package(Eigen3 3.3 REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)

#target_compile_definitions(my_lib PRIVATE ${PINOCCHIO_CFLAGS_OTHER})

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

#add_executable(FSM_ass2 src/FSM.cpp src/FSM_ass2.cpp src/controller.cpp src/kinematics/trajectory.cpp src/kinematics/direct.cpp src/kinematics/inverse.cpp src/kinematics/jacobian.cpp)
add_executable(FSM_ass2 src/test_new_kinematics.cpp src/controller.cpp src/kinematics/trajectory.cpp src/kinematics/direct.cpp src/kinematics/inverse.cpp src/kinematics/jacobian.cpp src/kinematics/inverse_pinocchio.cpp )

target_link_libraries(FSM_ass2 ${catkin_LIBRARIES})
target_link_libraries(FSM_ass2 pinocchio::pinocchio)

install(TARGETS FSM_ass2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

