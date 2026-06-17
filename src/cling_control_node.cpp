#include <memory>
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

  move_group_interface.setWorkspace(0.0, -0.5, 0.0,  0.8, 0.5, 1.0);

  move_group_interface.setGoalPositionTolerance(0.005);
  move_group_interface.setGoalOrientationTolerance(0.01);

  auto const approach_pose = [] {
    geometry_msgs::msg::Pose msg;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);  
    msg.orientation = tf2::toMsg(q);
    msg.position.x = -0.3;
    msg.position.y = 0.4;
    msg.position.z = 0.5;
    return msg;
  }();

move_group_interface.setPoseTarget(approach_pose);

  auto make_jc = [](const std::string& name, double pos, double tol) {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = name;
    jc.position = pos;
    jc.tolerance_above = tol;
    jc.tolerance_below = tol;
    jc.weight = 1.0;
    return jc;
  };

  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.joint_constraints.push_back(make_jc("ur5e_elbow_joint",        1.2625, 1.5));
  path_constraints.joint_constraints.push_back(make_jc("ur5e_shoulder_pan_joint", 1.4160, 1.5));
  path_constraints.joint_constraints.push_back(make_jc("ur5e_wrist_1_joint",     -1.4886, 1.5));
  path_constraints.joint_constraints.push_back(make_jc("ur5e_wrist_2_joint",     -1.4, 1.5));

  move_group_interface.setPathConstraints(path_constraints);

  prompt("Press 'Next' to plan the approach pose");
  draw_title("Planning Approach");
  moveit_visual_tools.trigger();

  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (!success) {
    draw_title("Approach Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Approach planning failed!");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  draw_trajectory_tool_path(plan.trajectory);
  moveit_visual_tools.trigger();

  prompt("Press 'Next' to execute the approach pose");
  draw_title("Executing Approach");
  moveit_visual_tools.trigger();
  move_group_interface.execute(plan);

auto current_stamped = move_group_interface.getCurrentPose();
  geometry_msgs::msg::PoseStamped pose_in_base;   // <-- PoseStamped statt Pose
  try {
    pose_in_base = tf_buffer->transform(
        current_stamped, "ur5e_base_link", tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "TF transform failed: %s", ex.what());
    rclcpp::shutdown(); spinner.join(); return 1;
  }
  auto current_pose = pose_in_base.pose;  

  geometry_msgs::msg::Pose down_pose = current_pose;
  down_pose.position.z = current_pose.position.z - 0.1;  

  RCLCPP_INFO(logger, "Current Pose: [%.2f, %.2f, %.2f]", current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(logger, "Target Pose:  [%.2f, %.2f, %.2f]", down_pose.position.x, down_pose.position.y, down_pose.position.z);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(down_pose);

  // --- Klick 3: Plan Cartesian ---
  prompt("Press 'Next' to plan the cartesian descent");
  draw_title("Planning Descent (Cartesian)");
  moveit_visual_tools.trigger();

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;
  double fraction = move_group_interface.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Cartesian path: %.2f%% achieved", fraction * 100.0);

  if (fraction > 0.0) {
    draw_trajectory_tool_path(trajectory);
    moveit_visual_tools.trigger();

    // --- Klick 4: Execute Cartesian ---
    prompt("Press 'Next' to execute the cartesian descent");
    draw_title("Executing Descent");
    moveit_visual_tools.trigger();
    move_group_interface.execute(trajectory);
  } else {
    draw_title("Cartesian Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Cartesian planning failed! Only %.2f%% achieved", fraction * 100.0);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}