cmake_minimum_required(VERSION 3.0.2)
project(stack_of_task_space_controllers_ros_control)

set(CATKIN_DEPENDENCIES
  controller_interface
  control_toolbox
  hardware_interface
  pluginlib
  roscpp

  stack_of_task_space_controllers_core
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})
find_package(Eigen3 REQUIRED)

# Export catkin package
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS pluginlib
)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

###########
## Build ##
###########
add_library(${PROJECT_NAME} src/stack_of_task_space_controllers_ros_control.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wno-unknown-pragmas)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES StackOfTaskSpaceControllersController.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
