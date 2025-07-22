file(GLOB PYTHON_SOURCES
  "*.cpp"
)

pybind_add_module(_mujoco_ros_template_python
  ${PYTHON_SOURCES}
)

target_include_directories(_mujoco_ros_template_python SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(_mujoco_ros_template_python PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS _mujoco_ros_template_python LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
