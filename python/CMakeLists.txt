file(GLOB PYTHON_SOURCES
  "bindings/src/*.cpp"
)

pybind_add_module(pymujoco_ros_template
  ${PYTHON_SOURCES}
)

target_include_directories(pymujoco_ros_template SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(pymujoco_ros_template PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS pymujoco_ros_template LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
