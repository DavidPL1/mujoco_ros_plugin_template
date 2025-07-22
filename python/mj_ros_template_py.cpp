/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David P. Leins */


/*
Plugin customization TODOs:

1. change include to the correct header file(s)
2. replace `plugin_namespace` with actual sub-namespace
3. replace PluginTemplate with plugin class name
4. Implement accessor class to expose private members and functions to Python bindings
5. Bind properties and methods in the Python module
*/

#include <mujoco_ros_plugin_template/plugin_template.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// Uncomment if you need automatic conversion of ROS messages from C++ to 
// Python and vice versa and import the necessary message headers.
// #include <py_binding_tools/ros_msg_typecasters.h>

#include <mujoco_ros/mujoco_env.h>

namespace py = pybind11;
namespace mujoco_ros::plugin_namespace {

// Example accessor class to expose private members to Python bindings
class PluginTemplateAccessor {
public:
    static bool& getExampleBool(mujoco_ros::plugin_namespace::PluginTemplate &plugin) {
        return plugin.example_bool_;
    }

    static void setExampleBool(mujoco_ros::plugin_namespace::PluginTemplate &plugin, bool value) {
        plugin.example_bool_ = value;
    }

    static int& getExampleInt(mujoco_ros::plugin_namespace::PluginTemplate &plugin) {
        return plugin.example_int_;
    }

    static void setExampleInt(mujoco_ros::plugin_namespace::PluginTemplate &plugin, int value) {
        plugin.example_int_ = value;
    }

};

} // namespace mujoco_ros::plugin_namespace

namespace mujoco_ros::python::plugin_namespace {

PYBIND11_MODULE(_mujoco_ros_template_python, m)
{
    py::class_<mujoco_ros::plugin_namespace::PluginTemplate, mujoco_ros::MujocoPlugin, std::shared_ptr<mujoco_ros::plugin_namespace::PluginTemplate>>(m, "MujocoRosTemplatePlugin")
    .def(py::init<>())
    .def_property(
        "example_bool",
        &mujoco_ros::plugin_namespace::PluginTemplateAccessor::getExampleBool,
        &mujoco_ros::plugin_namespace::PluginTemplateAccessor::setExampleBool,
        py::return_value_policy::reference_internal
    )
    .def_property(
        "example_int",
        &mujoco_ros::plugin_namespace::PluginTemplateAccessor::getExampleInt,
        &mujoco_ros::plugin_namespace::PluginTemplateAccessor::setExampleInt,
        py::return_value_policy::reference_internal
    )
    .def("__repr__", [](const mujoco_ros::plugin_namespace::PluginTemplate &plugin) {
        bool example_bool = mujoco_ros::plugin_namespace::PluginTemplateAccessor::getExampleBool(const_cast<mujoco_ros::plugin_namespace::PluginTemplate &>(plugin));
        int example_int = mujoco_ros::plugin_namespace::PluginTemplateAccessor::getExampleInt(const_cast<mujoco_ros::plugin_namespace::PluginTemplate &>(plugin));
        return "<MujocoRosTemplatePlugin bool: " + std::to_string(example_bool) +
               ", int: " + std::to_string(example_int) + ">";
    })
    .def("extended_functionality", &mujoco_ros::plugin_namespace::PluginTemplate::extendedFunctionality);
}

}