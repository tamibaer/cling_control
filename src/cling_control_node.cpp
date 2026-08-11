#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

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
const std::map<std::string, double> home_joints = {
  {"ur5e_shoulder_pan_joint",   0.0},
  {"ur5e_shoulder_lift_joint", -1.5708},
  {"ur5e_elbow_joint",          0.0},
  {"ur5e_wrist_1_joint",       -1.5708},
  {"ur5e_wrist_2_joint",        0.0},
  {"ur5e_wrist_3_joint",        0.0}
};

}  // namespace

bool zeroFtSensor(const rclcpp::Node::SharedPtr & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>(
    "/io_and_status_controller/zero_ftsensor");
  client->wait_for_service();

  auto future = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  future.wait();

  return future.get()->success;
}

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
    if (!arm.moveToJoints(pick_joints, "PICK")) break;

    if (!arm.prompt("Press 'Next' for cartesian LEFT")) break;
    arm.moveCartesianDelta(-0.13, 0.05, 0.0, "LEFT");

    if (!arm.prompt("Press 'Next' to CLOSE gripper")) break;
    arm.gripperAction("close", "CLOSE");

    if (!arm.prompt("Press 'Next' for cartesian BACK")) break;
    arm.moveCartesianDelta(0.0, -0.1, 0.0, "BACK");

    if (!zeroFtSensor(node)) break;

    if (!arm.prompt("Press 'Next' for cartesian move to TEAR")) break;
     arm.moveCartesianDelta(0.4, 0.0, 0.35, "TEARING");

    if (!zeroFtSensor(node)) break;

    if (!arm.prompt("Press 'Next' to OPEN gripper")) break;
    arm.gripperAction("open", "OPEN");

    if (!arm.prompt("Press 'Next' to return to HOME")) break;
    if (!arm.moveToJoints(home_joints, "HOME")) break;
  }

  RCLCPP_INFO(logger, "Beendet.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
