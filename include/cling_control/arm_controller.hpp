#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2/LinearMath/Quaternion.h>
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
    move_group_interface_.setMaxVelocityScalingFactor(0.5);
    move_group_interface_.setMaxAccelerationScalingFactor(0.05);
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

  // Wie moveCartesianDelta, dreht zusaetzlich das Wrist-3-Gelenk (lokale Tool-Z-Achse)
  // um den Winkel des Bewegungspfades in der X-Z-Ebene, sodass der Greifer waehrend
  // des Reissens entlang des kartesischen Pfades ausgerichtet bleibt.
  bool moveCartesianDeltaAligned(double dx, double dy, double dz, const std::string & label)
  {
    geometry_msgs::msg::Pose target;
    if (!translatedTarget(dx, dy, dz, target)) {
      return false;
    }

    // atan2(dz, dx) direkt wuerde bei Vorzeichenwechsel von dx (z.B. Reissen LINKS
    // statt RECHTS) in den zweiten Quadranten springen (z.B. 140 statt -40 Grad) und
    // so eine unnoetig grosse Zusatzdrehung erzeugen. Stattdessen den Neigungswinkel
    // vorzeichenunabhaengig von dx berechnen und danach das Vorzeichen von dx wieder
    // aufpraegen, damit RECHTS/LINKS symmetrisch gespiegelt werden.
    const double path_angle_rad = std::copysign(std::atan2(dz, std::abs(dx)), dx);
    applyLocalRotation(target, 0.0, 0.0, path_angle_rad);

    RCLCPP_INFO(logger_, "%s: delta (%.3f, %.3f, %.3f), wrist3 %.1f deg -> [%.3f, %.3f, %.3f]",
                label.c_str(), dx, dy, dz, path_angle_rad * 180.0 / M_PI,
                target.position.x, target.position.y, target.position.z);

    return executeCartesian(target, label);
  }

  // Wie moveCartesianDelta, dreht zusaetzlich das Wrist-2-Gelenk (lokale Tool-Y-Achse,
  // solange Wrist-3 ~0 steht) um den Neigungswinkel des Pfades in der Y-Z-Ebene, sodass
  // der Greifer beim Rueckzug/Vorspannen je nach vertikalem Pfadanteil nach oben oder
  // unten ausgerichtet wird. dy ist hierbei die dominante (Rueckzugs-)Achse.
  bool moveCartesianDeltaAlignedVertical(double dx, double dy, double dz, const std::string & label)
  {
    geometry_msgs::msg::Pose target;
    if (!translatedTarget(dx, dy, dz, target)) {
      return false;
    }

    const double vertical_path_angle_rad = -std::atan2(dz, -dy);
    applyLocalRotation(target, 0.0, vertical_path_angle_rad, 0.0);

    RCLCPP_INFO(logger_, "%s: delta (%.3f, %.3f, %.3f), wrist2 %.1f deg -> [%.3f, %.3f, %.3f]",
                label.c_str(), dx, dy, dz, vertical_path_angle_rad * 180.0 / M_PI,
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
  bool translatedTarget(double dx, double dy, double dz, geometry_msgs::msg::Pose & target)
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
    target = pose_in_base.pose;
    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;
    return true;
  }

  // Dreht target.orientation um roll/pitch/yaw im lokalen Tool-Frame (Wrist-3-Achse =
  // lokales Z fuer yaw, Wrist-2-Achse bei Wrist-3~0 = lokales Y fuer pitch).
  void applyLocalRotation(geometry_msgs::msg::Pose & target, double roll, double pitch, double yaw)
  {
    tf2::Quaternion current_orientation;
    tf2::fromMsg(target.orientation, current_orientation);
    tf2::Quaternion local_rotation;
    local_rotation.setRPY(roll, pitch, yaw);
    tf2::Quaternion target_orientation = current_orientation * local_rotation;
    target_orientation.normalize();
    target.orientation = tf2::toMsg(target_orientation);
  }

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
