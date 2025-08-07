//  Copyright 2025 Robust Robotic Systems, Cognitive Robotics, TU Delft
// 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
// 
//  http://www.apache.org/licenses/LICENSE-2.0
// 
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
#pragma once

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class ArmHomePublisher : public rclcpp::Node
{
public:
    ArmHomePublisher();

private:
    void timer_callback();

    trajectory_msgs::msg::JointTrajectory traj_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::CallbackGroup::SharedPtr callback_group_action_client_;
  	rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_cli_;
	bool arm_goal_sent_ = false;
};