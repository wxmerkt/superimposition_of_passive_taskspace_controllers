cmake_minimum_required(VERSION 2.8.11)
project(stack_of_task_space_controllers_rocoma)

set(PACKAGE_DEPENDENCIES
  anymal_roco
  message_logger
  anymal_description
  anymal_model
  signal_logger
  realtime_tools
  std_msgs
  stack_of_task_space_controllers_core
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# Export catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########
add_definitions(-std=c++14 -Wall -Wno-unknown-pragmas)

# Build the core library.
add_library(${PROJECT_NAME}
  src/stack_of_task_space_controllers_rocoma.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

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

#############
## Plugin ##
#############

# Build and install the plugin dependent library if rocoma_plugin exists
find_package(catkin QUIET COMPONENTS rocoma_plugin ${PACKAGE_DEPENDENCIES})
if(${rocoma_plugin_FOUND})
  add_library(${PROJECT_NAME}_plugin src/rocoma_plugin.cpp)
  target_include_directories(${PROJECT_NAME}_plugin PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  install(TARGETS ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(FILES rocoma_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif(${rocoma_plugin_FOUND})
