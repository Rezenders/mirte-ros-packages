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
#include <chrono>
#include "mirte_master_arm_control/arm_home_publisher.hpp"

using namespace std::chrono_literals;

using FollowGoalHandle =    
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;

ArmHomePublisher::ArmHomePublisher()
    : Node("arm_home_publisher")
{
    // Declare params with defaults if needed
    this->declare_parameter("joint_names", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("positions", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("time_from_start", 5.0);

    auto joint_names = this->get_parameter("joint_names").as_string_array();
    auto positions = this->get_parameter("positions").as_double_array();
    double time_from_start = this->get_parameter("time_from_start").as_double();

    if (joint_names.size() != positions.size() || joint_names.empty()) {
		RCLCPP_ERROR(this->get_logger(), "joint_names and positions must have same nonzero length.");
		rclcpp::shutdown();
		return;
    }

    traj_msg_.joint_names.assign(joint_names.begin(), joint_names.end());

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(positions.begin(), positions.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
    traj_msg_.points.push_back(pt);

    action_cli_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this,
      "mirte_master_arm_controller/follow_joint_trajectory",
      callback_group_action_client_
    );

    timer_ = this->create_wall_timer(
        500ms, std::bind(&ArmHomePublisher::timer_callback, this));
}

void ArmHomePublisher::timer_callback()
{
    if(action_cli_->action_server_is_ready() && !arm_goal_sent_){
        control_msgs::action::FollowJointTrajectory::Goal arm_position_goal;
        arm_position_goal.trajectory = traj_msg_;

        auto send_goal_options =
            rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](auto) {
            RCLCPP_INFO(this->get_logger(), "Arm in initial arm pose!");
            timer_->cancel();
            arm_goal_sent_ = false;
            rclcpp::shutdown();
        };

        send_goal_options.goal_response_callback = [this](FollowGoalHandle::SharedPtr goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server. Retrying.");
                arm_goal_sent_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        action_cli_->async_send_goal(arm_position_goal, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Published initial arm pose!");
        arm_goal_sent_ = true;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmHomePublisher>();
    rclcpp::spin(node);
    return 0;
}
