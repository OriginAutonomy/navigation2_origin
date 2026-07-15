// Copyright (c) 2026 Origin
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

#include "nav2_rviz_plugins/compute_path_tool.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_rviz_plugins {

ComputePathTool::ComputePathTool() : rviz_default_plugins::tools::PoseTool() {
    shortcut_key_ = 'p';
}

ComputePathTool::~ComputePathTool() {}

void ComputePathTool::onInitialize() {
    PoseTool::onInitialize();
    setName("Compute Path");
    setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));

    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    action_client_ = rclcpp_action::create_client<ComputePathToPose>(node_, "compute_path_to_pose");

    // Runtime-tunable via `ros2 param set`; guard against re-declaration on the shared RViz node.
    if (!node_->has_parameter("compute_path_tool.plan_in_static")) {
        node_->declare_parameter("compute_path_tool.plan_in_static", true);
    }
    if (!node_->has_parameter("compute_path_tool.classify_paths")) {
        node_->declare_parameter("compute_path_tool.classify_paths", true);
    }
}

void ComputePathTool::onPoseSet(double x, double y, double theta) {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "compute_path_to_pose action server is not available.");
        return;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = context_->getFixedFrame().toStdString();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    goal.pose.orientation = tf2::toMsg(q);

    ComputePathToPose::Goal goal_msg;
    goal_msg.goal = goal;
    goal_msg.planner_id = "";
    goal_msg.use_start = false;
    goal_msg.plan_in_static = node_->get_parameter("compute_path_tool.plan_in_static").as_bool();
    goal_msg.classify_paths = node_->get_parameter("compute_path_tool.classify_paths").as_bool();

    RCLCPP_INFO(
            node_->get_logger(),
            "Sending ComputePathToPose goal: (%.2f, %.2f, %.2f) in frame '%s'",
            x,
            y,
            theta,
            goal.header.frame_id.c_str());

    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<
                                                ComputePathToPose>::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: {
                const auto& poses = result.result->path.poses;
                double path_length = 0.0;
                for (size_t i = 1; i < poses.size(); ++i) {
                    const auto& a = poses[i - 1].pose.position;
                    const auto& b = poses[i].pose.position;
                    path_length += std::hypot(b.x - a.x, b.y - a.y);
                }
                const double planning_time =
                        rclcpp::Duration(result.result->planning_time).seconds();
                const bool classify_paths =
                        node_->get_parameter("compute_path_tool.classify_paths").as_bool();
                RCLCPP_INFO(
                        node_->get_logger(),
                        "ComputePathToPose succeeded: %zu poses, length %.3f m, planning time %.4f "
                        "s, classify_paths %s",
                        poses.size(),
                        path_length,
                        planning_time,
                        classify_paths ? "on" : "off");
                break;
            }
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(node_->get_logger(), "ComputePathToPose canceled");
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose returned unknown result code");
                break;
        }
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::ComputePathTool, rviz_common::Tool)
