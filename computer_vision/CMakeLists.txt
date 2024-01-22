# cmake_minimum_required(VERSION 3.0.2)
# project(computer_vision)
# ## Find catkin and any catkin packages
# find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation )

# add_compile_options(-std=c++11)

# add_executable(yoloNode src/yoloNode.cpp)

# generate_messages(
# DEPENDENCIES
# std_msgs
# sensor_msgs
# )

# ## Build talker and listener
# include_directories(include ${catkin_INCLUDE_DIRS} )

# ## Declare a catkin package
# catkin_package(
# INCLUDE_DIRS include
# CATKIN_DEPENDS message_runtime
# CATKIN_DEPENDS roscpp
# LIBRARIES ${PROJECT_NAME}
# )

# target_link_libraries(yoloNode ${catkin_LIBRARIES})

# ## Mark libraries for installation
# install(TARGETS yoloNode
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

cmake_minimum_required(VERSION 2.8.3)
project(computer_vision)

# # Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs message_generation)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
  Points.msg
  Instruction.msg
)

add_service_files(
  FILES
  GetInstructions.srv
  PointCloud.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# # Declare a catkin package
catkin_package()

# # Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(yoloNode src/yoloNode.cpp)
target_link_libraries(yoloNode ${catkin_LIBRARIES})
install(TARGETS yoloNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/detect.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})