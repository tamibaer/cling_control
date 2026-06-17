#include <memory>
#include <map>
#include <string>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "cling_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("cling_control_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur5e_arm");

  auto gripper = MoveGroupInterface(node, "robotiq_gripper");
  gripper.setMaxVelocityScalingFactor(0.5);
  gripper.setMaxAccelerationScalingFactor(0.5);

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()
  };

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = []{
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 2.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text){
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur5e_arm")](auto const trajectory) {
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };

  move_group_interface.setPoseReferenceFrame("ur5e_base_link");
  move_group_interface.setPlannerId("RRTConnect");
  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setNumPlanningAttempts(10);
  move_group_interface.setMaxVelocityScalingFactor(0.2);
  move_group_interface.setMaxAccelerationScalingFactor(0.2);

  // ===== Gripper-Steuerung =====  <-- "close"/"open" ggf. an SRDF anpassen
  auto gripper_action = [&](const std::string& named_target, const std::string& label) -> bool {
    gripper.setNamedTarget(named_target);
    draw_title("Gripper: " + label);
    moveit_visual_tools.trigger();
    bool ok = static_cast<bool>(gripper.move());
    if (!ok) RCLCPP_ERROR(logger, "Gripper '%s' fehlgeschlagen!", label.c_str());
    return ok;
  };

  // ===== Ziel-Konfigurationen =====
  const std::map<std::string, double> pick_joints = {
    {"ur5e_shoulder_pan_joint",  -1.1327},
    {"ur5e_shoulder_lift_joint", -1.6820},
    {"ur5e_elbow_joint",         -2.1871},
    {"ur5e_wrist_1_joint",        0.7017},
    {"ur5e_wrist_2_joint",        1.1130},
    {"ur5e_wrist_3_joint",       -1.5570}
  };
  // Pose 1: Greifposition (erste Messung)
  const std::map<std::string, double> pose1_joints = {
    {"ur5e_shoulder_pan_joint",  -1.3274},
    {"ur5e_shoulder_lift_joint", -1.6607},
    {"ur5e_elbow_joint",         -1.6084},
    {"ur5e_wrist_1_joint",       -0.8260},
    {"ur5e_wrist_2_joint",        1.4179},
    {"ur5e_wrist_3_joint",       -1.3555}
  };
  const std::map<std::string, double> home_joints = {
    {"ur5e_shoulder_pan_joint",   0.0},
    {"ur5e_shoulder_lift_joint", -1.5708},
    {"ur5e_elbow_joint",          0.0},
    {"ur5e_wrist_1_joint",       -1.5708},
    {"ur5e_wrist_2_joint",        0.0},
    {"ur5e_wrist_3_joint",        0.0}
  };

  // ===== Hilfsfunktion: zu einer Joint-Konfiguration planen + ausführen =====
  auto move_to_joints = [&](const std::map<std::string, double>& joints,
                            const std::string& label) -> bool {
    move_group_interface.setJointValueTarget(joints);
    draw_title("Planning: " + label);
    moveit_visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group_interface.plan(plan))) {
      draw_title(label + " Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "%s planning failed!", label.c_str());
      return false;
    }
    draw_trajectory_tool_path(plan.trajectory);
    moveit_visual_tools.trigger();
    draw_title("Executing: " + label);
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
    return true;
  };

  // ===== Hilfsfunktion: kartesisch um (dx, dy, dz) verschieben =====
  auto move_cartesian_delta = [&](double dx, double dy, double dz, const std::string& label) -> bool {
    auto current_stamped = move_group_interface.getCurrentPose();
    geometry_msgs::msg::PoseStamped pose_in_base;
    try {
      pose_in_base = tf_buffer->transform(
          current_stamped, "ur5e_base_link", tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger, "TF transform failed: %s", ex.what());
      return false;
    }
    geometry_msgs::msg::Pose target = pose_in_base.pose;
    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;

    RCLCPP_INFO(logger, "%s: delta (%.3f, %.3f, %.3f) -> [%.3f, %.3f, %.3f]",
                label.c_str(), dx, dy, dz,
                target.position.x, target.position.y, target.position.z);

    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory);

    RCLCPP_INFO(logger, "%s: Cartesian %.1f%% achieved", label.c_str(), fraction * 100.0);
    if (fraction > 0.95) {
      draw_trajectory_tool_path(trajectory);
      moveit_visual_tools.trigger();
      draw_title("Executing: " + label);
      moveit_visual_tools.trigger();
      move_group_interface.execute(trajectory);
      return true;
    }
    draw_title(label + " Cartesian Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "%s cartesian failed (%.1f%%)", label.c_str(), fraction * 100.0);
    return false;
  };

  // ===== Hilfsfunktion: kartesisch zu einer Ziel-TCP-Pose (Koordinaten) =====
  auto move_cartesian_to = [&](double x, double y, double z,
                               double qx, double qy, double qz, double qw,
                               const std::string& label) -> bool {
    geometry_msgs::msg::Pose target;
    target.position.x = x; target.position.y = y; target.position.z = z;
    target.orientation.x = qx; target.orientation.y = qy;
    target.orientation.z = qz; target.orientation.w = qw;

    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory);

    RCLCPP_INFO(logger, "%s: Cartesian %.1f%% achieved", label.c_str(), fraction * 100.0);
    if (fraction > 0.95) {
      draw_trajectory_tool_path(trajectory);
      moveit_visual_tools.trigger();
      draw_title("Executing: " + label);
      moveit_visual_tools.trigger();
      move_group_interface.execute(trajectory);
      return true;
    }
    draw_title(label + " Cartesian Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "%s cartesian failed (%.1f%%)", label.c_str(), fraction * 100.0);
    return false;
  };

  // ===== Gripper am Anfang schließen =====
  prompt("Press 'Next' to CLOSE gripper (start)");
  if (rclcpp::ok()) gripper_action("close", "Close");

  // ===== Dauerschleife =====
  int cycle = 0;
  while (rclcpp::ok()) {
    cycle++;
    RCLCPP_INFO(logger, "=== Zyklus %d ===", cycle);

    prompt("Press 'Next' to move to PICK pose (or Ctrl+C to quit)");
    if (!rclcpp::ok()) break;
    if (!move_to_joints(pick_joints, "Pick")) break;

    prompt("Press 'Next' for cartesian DOWN");
    if (!rclcpp::ok()) break;
    move_cartesian_delta(0.0, 0.0, -0.1, "Down");

    // Nach dem Runterfahren: Gripper öffnen
    prompt("Press 'Next' to OPEN gripper");
    if (!rclcpp::ok()) break;
    gripper_action("open", "Open");

    // Zu Pose 1 (Greifposition) fahren
    prompt("Press 'Next' to move to POSE 1");
    if (!rclcpp::ok()) break;
    if (!move_to_joints(pose1_joints, "Pose1")) break;

    // Gripper schließen (greifen)
    prompt("Press 'Next' to CLOSE gripper");
    if (!rclcpp::ok()) break;
    gripper_action("close", "Close");

    // Kartesisch zu Pose 2 (Koordinaten aus tf2_echo)
    prompt("Press 'Next' for cartesian move to POSE 2");
    if (!rclcpp::ok()) break;
    move_cartesian_to(0.007, 0.657, 0.728,
                      0.680, 0.664, 0.222, -0.219,
                      "Pose2 (cartesian)");

    // Kartesisch diagonal (X + nach oben)
    prompt("Press 'Next' for diagonal cartesian move");
    if (!rclcpp::ok()) break;
    move_cartesian_delta(0.0, -0.1, 0.1, "Diagonal");

    prompt("Press 'Next' to OPEN gripper");
    if (!rclcpp::ok()) break;
    gripper_action("open", "Open");

    // Zurück zu Home
    prompt("Press 'Next' to return to HOME");
    if (!rclcpp::ok()) break;
    if (!move_to_joints(home_joints, "Home")) break;
  }

  RCLCPP_INFO(logger, "Beendet.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}