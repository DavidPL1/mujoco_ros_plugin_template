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

#pragma once

/*
Plugin customization TODOs:

1. rename folder containing this file
2. replace `plugin_namespace` with actual sub-namespace
3. replace PluginTemplate with plugin class name 
4. delete functions that will not be overridden
5. add plugin specific functions and member definitions
6. Rename or remove friend class (PluginTemplateAccessor)
*/

#include "mujoco_ros/plugin_utils.h"
#include "mujoco_ros/common_types.h"
#include "mujoco_ros/mujoco_env.h"

using namespace mujoco_ros;

namespace mujoco_ros::plugin_namespace {

class PluginTemplate : public MujocoPlugin
{
public:
	// Friend definition to access private/protected members in python bindings
	// Remove if not implementing python bindings
	friend class PluginTemplateAccessor;

	PluginTemplate()           = default;
	~PluginTemplate() override = default;

	bool load(const mjModel *m, mjData *d) override;
	void reset() override;
	void controlCallback(const mjModel *model, mjData *data) override;
	void passiveCallback(const mjModel *model, mjData *data) override;
	void renderCallback(const mjModel *model, mjData *data, mjvScene *scene) override;
	void lastStageCallback(const mjModel *model, mjData *data) override;
	void onGeomChanged(const mjModel *model, mjData *data, const int geom_id) override;

	// Any additional functionality that should be exposed in the python bindings of the plugin
	void extendedFunctionality();

	// The env_ptr_ (shared_ptr) in the parent class ensures mjModel and mjData are not destroyed
	const mjModel *m_;
	mjData *d_;

private:

	// Some private member variables can be defined here
	bool example_bool_ = false; // Example boolean member variable
	int example_int_ = 0; // Example integer member variable
};
} // namespace mujoco_ros::plugin_namespace
