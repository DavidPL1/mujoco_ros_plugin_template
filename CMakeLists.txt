cmake_minimum_required(VERSION 3.0.2)

# TODO: Set the actual project name
project(mujoco_ros_plugin_template)

# Optional: change/add more flags
set( CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined" )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE not set. Defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_SANITIZE "-fsanitize=address -g -O1 -fno-inline -fno-omit-frame-pointer -fno-optimize-sibling-calls")

# TODO: add more required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  mujoco_ros
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pluginlib
    mujoco_ros
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

# TODO: add other source files
add_library(${PROJECT_NAME}
  src/main.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Optional: add more files that need to be installed
install(FILES
  plugin_template.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
