cmake_minimum_required(VERSION 2.8.3)
project(robot_project)

# # Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs computer_vision)
find_package(Eigen3 3.3 REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)

# target_compile_definitions(my_lib PRIVATE ${PINOCCHIO_CFLAGS_OTHER})

# # Declare a catkin package
catkin_package()

# # Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(FSM_Main src/FSM/FSM.cpp src/FSM/FSM_Main.cpp src/controller.cpp src/kinematics/direct.cpp src/kinematics/inverse.cpp src/kinematics/jacobian.cpp src/trajectories/movement_checks.cpp src/trajectories/motion_plans.cpp src/trajectories/trajectory_init.cpp)

target_link_libraries(FSM_Main ${catkin_LIBRARIES})
target_link_libraries(FSM_Main pinocchio::pinocchio)

install(TARGETS FSM_Main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test src/test_new_kinematics.cpp src/controller.cpp src/kinematics/direct.cpp src/kinematics/inverse.cpp src/kinematics/jacobian.cpp src/trajectories/movement_checks.cpp src/trajectories/motion_plans.cpp src/trajectories/trajectory_init.cpp)

target_link_libraries(test ${catkin_LIBRARIES})
target_link_libraries(test pinocchio::pinocchio)

install(TARGETS test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_all_poses src/test_all_poses.cpp src/controller.cpp src/controller_tests.cpp src/kinematics/direct.cpp src/kinematics/inverse.cpp src/kinematics/jacobian.cpp src/trajectories/movement_checks.cpp src/trajectories/motion_plans.cpp src/trajectories/trajectory_init.cpp)

target_link_libraries(test_all_poses ${catkin_LIBRARIES})
target_link_libraries(test_all_poses pinocchio::pinocchio)

install(TARGETS test_all_poses
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
