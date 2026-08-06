#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

class VisualToolsHelper
{
public:
  VisualToolsHelper(
    const rclcpp::Node::SharedPtr & node,
    const moveit::core::RobotModelConstPtr & robot_model,
    const std::string & planning_group)
  : visual_tools_(node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, robot_model),
    joint_model_group_(robot_model->getJointModelGroup(planning_group))
  {
    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();
  }

  void drawTitle(const std::string & text)
  {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 2.0;
      return msg;
    }();
    visual_tools_.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  }

  void prompt(const std::string & text)
  {
    visual_tools_.prompt(text);
  }

  void drawTrajectoryToolPath(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    visual_tools_.publishTrajectoryLine(trajectory, joint_model_group_);
  }

  void trigger()
  {
    visual_tools_.trigger();
  }

private:
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  const moveit::core::JointModelGroup * joint_model_group_;
};