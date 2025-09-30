// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "smart_diffbot_navigation/bt_plugins/abstract_condition.hpp"

namespace smart_diffbot_navigation
{

AbstractCondition::AbstractCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
}

AbstractCondition::~AbstractCondition()
{
  cleanup();
}

BT::NodeStatus AbstractCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  if (isConditionMet()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void AbstractCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_);

  initialized_ = true;
}

bool AbstractCondition::isConditionMet()
{
  
    RCLCPP_INFO(node_->get_logger(), "Current robot pose is not available.");
    return true;
  
}

}  // namespace smart_diffbot_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<smart_diffbot_navigation::AbstractCondition>("AbstractConditionMet");
}
