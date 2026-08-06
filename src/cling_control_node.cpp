#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cling_control/arm_controller.hpp"

namespace
{
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

const std::map<std::string, double> tear_end_pose_joints = {
  {"ur5e_shoulder_pan_joint",  -0.9412},
  {"ur5e_shoulder_lift_joint", -1.6168},
  {"ur5e_elbow_joint",         -1.2161},
  {"ur5e_wrist_1_joint",       -0.6703},
  {"ur5e_wrist_2_joint",        1.4334},
  {"ur5e_wrist_3_joint",       -0.5687}
};


const std::map<std::string, double> tear_joints = {
  {"ur5e_shoulder_pan_joint",  -1.4927},
  {"ur5e_shoulder_lift_joint", -1.4078},
  {"ur5e_elbow_joint",         -1.5761},
  {"ur5e_wrist_1_joint",       -0.4706},
  {"ur5e_wrist_2_joint",       1.8433},
  {"ur5e_wrist_3_joint",       -0.7474}
};

}  // namespace

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

  ArmController arm(node, logger, "ur5e_arm", "robotiq_gripper");


  // ===== Dauerschleife =====
  int cycle = 0;
  while (rclcpp::ok()) {
    cycle++;
    RCLCPP_INFO(logger, "=== Zyklus %d ===", cycle);

    if (!arm.prompt("Press 'Next' to move to PICK pose (or Ctrl+C to quit)")) break;
    if (!arm.moveToJoints(pick_joints, "Pick")) break;

    if (!arm.prompt("Press 'Next' for cartesian LEFT")) break;
    arm.moveCartesianDelta(-0.13, 0.05, 0.0, "Left");
    //arm.moveCartesianDelta(0.0, 0.05, 0.0, "Down");

    if (!arm.prompt("Press 'Next' to CLOSE gripper")) break;
    arm.gripperAction("close", "Close");

   
    if (!arm.prompt("Press 'Next' for cartesian DOWN")) break;
    arm.moveCartesianDelta(0.0, -0.1, 0.0, "Down");

    //if (!arm.prompt("Press 'Next' to move to PICK pose (or Ctrl+C to quit)")) break;
    //if (!arm.moveToJoints( tear_joints, "Pick")) break;

    //if (!arm.prompt("Press 'Next' to move to PICK pose (or Ctrl+C to quit)")) break;
    //if (!arm.moveToJoints( tear_end_pose_joints, "Pick")) break;

    if (!arm.prompt("Press 'Next' for cartesian move to POSE 2")) break;
     arm.moveCartesianDelta(0.35, 0.0, 0.3, "Left");



    if (!arm.prompt("Press 'Next' to OPEN gripper")) break;
    arm.gripperAction("open", "Open");

    // 0.680, 0.664, 0.222, -0.219
    //if (!arm.prompt("Press 'Next' for cartesian DOWN")) break;
    //arm.moveCartesianDelta(0.0, 0.0, -0.1, "Down");

    //if (!arm.prompt("Press 'Next' for cartesian DOWN")) break;
    //arm.moveCartesianDelta(0.0, 0.1, 0.0, "Forward");

    // Nach dem Runterfahren: Gripper öffnen
    //if (!arm.prompt("Press 'Next' to OPEN gripper")) break;
    //arm.gripperAction("open", "Open");

    // Zu Pose 1 (Greifposition) fahren
    //if (!arm.prompt("Press 'Next' to move to POSE 1")) break;
    //if (!arm.moveToJoints(pose1_joints, "Pose1")) break;

    // Gripper schließen (greifen)
    //if (!arm.prompt("Press 'Next' to CLOSE gripper")) break;
    //arm.gripperAction("close", "Close");

    // Kartesisch zu Pose 2 (Koordinaten aus tf2_echo)
    //if (!arm.prompt("Press 'Next' for cartesian move to POSE 2")) break;
    //arm.moveCartesianTo(0.007, 0.657, 0.728,
    //                    0.680, 0.664, 0.222, -0.219,
    //                    "Pose2 (cartesian)");

    // Kartesisch diagonal (X + nach oben)
    //if (!arm.prompt("Press 'Next' for diagonal cartesian move")) break;
    //arm.moveCartesianDelta(0.0, -0.1, 0.1, "Diagonal");

    //if (!arm.prompt("Press 'Next' to OPEN gripper")) break;
    //arm.gripperAction("open", "Open");

    // Zurück zu Home
    if (!arm.prompt("Press 'Next' to return to HOME")) break;
    if (!arm.moveToJoints(home_joints, "Home")) break;
  }

  RCLCPP_INFO(logger, "Beendet.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
