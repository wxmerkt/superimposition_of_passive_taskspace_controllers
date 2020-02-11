cmake_minimum_required(VERSION 2.8.12)
project(stack_of_passive_controllers_controller)

set(CATKIN_DEPENDENCIES
  controller_interface
  control_toolbox
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  urdf
  exotica_core
  ddynamic_reconfigure

  ipab_lwr_msgs
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
add_library(${PROJECT_NAME} src/StackOfPassiveControllersController.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++14 -Wall)

add_executable(fic_node src/stack_of_fic_node.cpp)
add_dependencies(fic_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fic_node ${catkin_LIBRARIES})
target_include_directories(fic_node PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_compile_options(fic_node PUBLIC -std=c++14 -Wall -Wno-unknown-pragmas)


#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES StackOfPassiveControllersController.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
