/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
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

#include <pluginlib/class_list_macros.h>

/*
Plugin customization TODOs:

1. change include to the correct header file(s)
2. replace `plugin_namespace` with actual sub-namespace
3. replace PluginTemplate with plugin class name 
4. code constructor and destructor
5. delete functions that will not be used (which where deleted in the header file)
*/ 

#include <mujoco_ros_plugin_template/plugin_template.h>

using namespace mujoco_ros;

namespace mujoco_ros::plugin_namespace {

bool PluginTemplate::load(const mjModel *m, mjData *d)
{
	if (rosparam_config_.hasMember("example_param")) {
		// use parameter or fail if it is needed
	}

	if (rosparam_config_.hasMember("nested_array_param_1")) {
		if (rosparam_config_["nested_array_param_1"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
			// use lvl1 nested param
			if (rosparam_config_["nested_array_param_1"][0].hasMember("nested_array_param_2")) {
				// use lvl2 nested param
			}
		}
	}

	if (rosparam_config_.hasMember("nested_struct_param_1")) {
		if (rosparam_config_["nested_struct_param_1"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			// use lvl1 nested struct param
			if (rosparam_config_["nested_struct_param_1"].hasMember("nested_struct_param_2")) {
				// use lvl2 nested struct param
			}
		}
	}

	m_ = m;
	d_ = d;
	return true;
}

void PluginTemplate::reset()
{
	// reset plugin state if not stateless
}

void PluginTemplate::controlCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	// compute and apply control
}

void PluginTemplate::passiveCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	// compute and apply passive forces
}

void PluginTemplate::renderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene * /*scene*/)
{
	// add visual geoms to render to the scene
}

void PluginTemplate::lastStageCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	// run last stage code here
}

void PluginTemplate::onGeomChanged(const mjModel * /*model*/, mjData * /*data*/, const int /*geom_id*/)
{
	// update information if a geom has been changed
}
} // namespace mujoco_ros::plugin_namespace

PLUGINLIB_EXPORT_CLASS(mujoco_ros::plugin_namespace::PluginTemplate, mujoco_ros::MujocoPlugin)
