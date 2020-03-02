cmake_minimum_required(VERSION 2.8.12)
project(stack_of_task_space_controllers_kuka_lwr)

set(CATKIN_DEPENDENCIES
  stack_of_task_space_controllers_core
  stack_of_task_space_controllers_ros_control
  ipab_lwr_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})
find_package(Eigen3 REQUIRED)

# Export catkin package
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

###########
## Build ##
###########
add_executable(stack_of_fic_node src/stack_of_fic_node.cpp)
add_dependencies(stack_of_fic_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(stack_of_fic_node ${catkin_LIBRARIES})
target_include_directories(stack_of_fic_node PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_compile_options(stack_of_fic_node PUBLIC -std=c++14 -Wall -Wno-unknown-pragmas)

#############
## Install ##
#############
install(TARGETS stack_of_fic_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config/ launch/ resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# TODO: Install scripts/

