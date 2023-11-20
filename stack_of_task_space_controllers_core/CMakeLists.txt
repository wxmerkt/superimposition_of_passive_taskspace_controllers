cmake_minimum_required(VERSION 3.0.2)
project(stack_of_task_space_controllers_core)

set(CATKIN_DEPENDENCIES
  realtime_tools
  roscpp
  std_msgs
  urdf
  exotica_core
  ddynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})
find_package(Eigen3 REQUIRED)

# Export catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

###########
## Build ##
###########
add_library(${PROJECT_NAME} src/stack_of_task_space_controllers.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE include)
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
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
