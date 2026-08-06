#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cling_control/visual_tools_helper.hpp"

class ArmController
{
public:
  ArmController(
    const rclcpp::Node::SharedPtr & node,
    const rclcpp::Logger & logger,
    const std::string & arm_group,
    const std::string & gripper_group)
  : logger_(logger),
    move_group_interface_(node, arm_group),
    visual_tools_(node, move_group_interface_.getRobotModel(), arm_group),
    gripper_(node, gripper_group),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    gripper_.setMaxVelocityScalingFactor(0.5);
    gripper_.setMaxAccelerationScalingFactor(0.5);

    move_group_interface_.setPoseReferenceFrame("ur5e_base_link");
    move_group_interface_.setPlannerId("RRTConnect");
    move_group_interface_.setPlanningTime(10.0);
    move_group_interface_.setNumPlanningAttempts(10);
    move_group_interface_.setMaxVelocityScalingFactor(0.2);
    move_group_interface_.setMaxAccelerationScalingFactor(0.2);
  }

  bool prompt(const std::string & text)
  {
    visual_tools_.prompt(text);
    return rclcpp::ok();
  }

  bool gripperAction(const std::string & named_target, const std::string & label)
  {
    gripper_.setNamedTarget(named_target);
    visual_tools_.drawTitle("Gripper: " + label);
    visual_tools_.trigger();
    bool ok = static_cast<bool>(gripper_.move());
    if (!ok) RCLCPP_ERROR(logger_, "Gripper '%s' fehlgeschlagen!", label.c_str());
    return ok;
  }

  bool moveToJoints(const std::map<std::string, double> & joints, const std::string & label)
  {
    move_group_interface_.setJointValueTarget(joints);
    visual_tools_.drawTitle("Planning: " + label);
    visual_tools_.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group_interface_.plan(plan))) {
      visual_tools_.drawTitle(label + " Planning Failed!");
      visual_tools_.trigger();
      RCLCPP_ERROR(logger_, "%s planning failed!", label.c_str());
      return false;
    }
    visual_tools_.drawTrajectoryToolPath(plan.trajectory);
    visual_tools_.trigger();
    visual_tools_.drawTitle("Executing: " + label);
    visual_tools_.trigger();
    move_group_interface_.execute(plan);
    return true;
  }

  bool moveCartesianDelta(double dx, double dy, double dz, const std::string & label)
  {
    auto current_stamped = move_group_interface_.getCurrentPose();
    geometry_msgs::msg::PoseStamped pose_in_base;
    try {
      pose_in_base = tf_buffer_->transform(
          current_stamped, "ur5e_base_link", tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "TF transform failed: %s", ex.what());
      return false;
    }
    geometry_msgs::msg::Pose target = pose_in_base.pose;
    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;

    RCLCPP_INFO(logger_, "%s: delta (%.3f, %.3f, %.3f) -> [%.3f, %.3f, %.3f]",
                label.c_str(), dx, dy, dz,
                target.position.x, target.position.y, target.position.z);

    return executeCartesian(target, label);
  }

  bool moveCartesianTo(
    double x, double y, double z,
    double qx, double qy, double qz, double qw,
    const std::string & label)
  {
    geometry_msgs::msg::Pose target;
    target.position.x = x; target.position.y = y; target.position.z = z;
    target.orientation.x = qx; target.orientation.y = qy;
    target.orientation.z = qz; target.orientation.w = qw;

    return executeCartesian(target, label);
  }

private:
  bool executeCartesian(const geometry_msgs::msg::Pose & target, const std::string & label)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface_.computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory);

    RCLCPP_INFO(logger_, "%s: Cartesian %.1f%% achieved", label.c_str(), fraction * 100.0);
    if (fraction > 0.95) {
      visual_tools_.drawTrajectoryToolPath(trajectory);
      visual_tools_.trigger();
      visual_tools_.drawTitle("Executing: " + label);
      visual_tools_.trigger();
      move_group_interface_.execute(trajectory);
      return true;
    }
    visual_tools_.drawTitle(label + " Cartesian Failed!");
    visual_tools_.trigger();
    RCLCPP_ERROR(logger_, "%s cartesian failed (%.1f%%)", label.c_str(), fraction * 100.0);
    return false;
  }

  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  VisualToolsHelper visual_tools_;
  moveit::planning_interface::MoveGroupInterface gripper_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
