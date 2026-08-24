#include <chrono>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cling_control/arm_controller.hpp"
#include "cling_control/rosbag_recorder.hpp"


const std::map<std::string, double> start_joints = {
  {"ur5e_shoulder_pan_joint", -1.1327},
  {"ur5e_shoulder_lift_joint", -1.6820},
  {"ur5e_elbow_joint", -2.1871},
  {"ur5e_wrist_1_joint", 0.7017},
  {"ur5e_wrist_2_joint", 1.1130},
  {"ur5e_wrist_3_joint", -1.5570}
};

const std::map<std::string, double> home_joints = {
  {"ur5e_shoulder_pan_joint", 0.0},
  {"ur5e_shoulder_lift_joint", -1.5708},
  {"ur5e_elbow_joint", 0.0},
  {"ur5e_wrist_1_joint", -1.5708},
  {"ur5e_wrist_2_joint", 0.0},
  {"ur5e_wrist_3_joint", 0.0}
};

enum class TearDirection
{
  LEFT,
  RIGHT
};

enum class GraspMode
{
  OPPOSITE_SIDE,
  CENTER
};

struct RunConfig
{
  double tension_angle_deg{0.0};
  double tension_vertical_angle_deg{0.0};

  double pull_back_distance{0.10};

  double tear_angle_deg{40.0};
  double tear_distance{0.50};

  TearDirection tear_direction{TearDirection::RIGHT};
  GraspMode grasp_mode{GraspMode::OPPOSITE_SIDE};
};

struct CartesianDelta
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double degreesToRadians(double angle_deg){
  constexpr double kPi = 3.14159265358979323846;
  return angle_deg * kPi / 180.0;
}

std::string directionToString(TearDirection direction){
  return direction == TearDirection::LEFT ? "left" : "right";
}

std::string graspPositionToString(const RunConfig & config){
  if (config.grasp_mode == GraspMode::CENTER) {
    return "center";
  }

  return config.tear_direction == TearDirection::LEFT ? "right" : "left";
}

CartesianDelta calculateMoveToGraspDelta(const RunConfig & config, double lateral_offset, double approach_distance){
  if (config.grasp_mode == GraspMode::CENTER) {
    return {0.0, approach_distance, 0.0};
  }

  const double x = config.tear_direction == TearDirection::LEFT ? lateral_offset : -lateral_offset;

  return {x, approach_distance, 0.0};
}


CartesianDelta calculateTensionDelta(const RunConfig & config){
  const double side_angle_rad = degreesToRadians(config.tension_angle_deg);
  const double vertical_angle_rad = degreesToRadians(config.tension_vertical_angle_deg);


  return {
    config.pull_back_distance * std::tan(side_angle_rad),
    -config.pull_back_distance,
    config.pull_back_distance * std::tan(vertical_angle_rad)
  };
}

CartesianDelta calculateTearDelta(const RunConfig & config){
  const double angle_rad = degreesToRadians(config.tear_angle_deg);
  const double direction_sign = config.tear_direction == TearDirection::RIGHT ? 1.0 : -1.0;

  return {direction_sign * config.tear_distance * std::cos(angle_rad), 0.0, config.tear_distance * std::sin(angle_rad)};
}

bool zeroFtSensor(const rclcpp::Node::SharedPtr & node){
  auto client = node->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/zero_ftsensor");

  client->wait_for_service();

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  future.wait();

  return future.get()->success;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>("cling_control_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("cling_control_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto spinner = std::thread([&executor]() {executor.spin();});

  ArmController arm(node, logger, "ur5e_arm", "robotiq_gripper");

  const std::string bag_base_dir = "/mnt/dattelspeicher/rosbags/tearing_experiment_test";

  std::filesystem::create_directories(bag_base_dir);

  const auto gripper_settle_time = std::chrono::seconds(2);
  const auto tension_hold_time = std::chrono::seconds(2);
  constexpr double grasp_lateral_offset = 0.13;
  constexpr double grasp_approach_distance = 0.05;

  RosbagRecorder::Options bag_options;
  bag_options.all_topics = false;
  bag_options.topics = {
    "/force_torque_sensor_broadcaster/wrench",
    "/force_torque_sensor_broadcaster/wrench_filtered",
    "/joint_states",
    "/robot_description",
    "/tf",
    "/tf_static"
  };

  const std::vector<RunConfig> run_configs = {
    {0.0, 0.0, 0.10, 40.0, 0.50, TearDirection::RIGHT, GraspMode::OPPOSITE_SIDE},
    {0.0, 0.0, 0.10, 40.0, 0.50, TearDirection::LEFT, GraspMode::OPPOSITE_SIDE},
    {0.0, 0.0, 0.10, 40.0, 0.50, TearDirection::RIGHT, GraspMode::CENTER},
    {0.0, 0.0, 0.10, 40.0, 0.50, TearDirection::LEFT, GraspMode::CENTER},
    {20.0, 0.0, 0.08, 40.0, 0.50, TearDirection::RIGHT, GraspMode::OPPOSITE_SIDE},
    {-20.0, -15.0, 0.12, 40.0, 0.50, TearDirection::LEFT, GraspMode::OPPOSITE_SIDE}
  };

  int cycle = 0;

  for (const auto & config : run_configs) {
    if (!rclcpp::ok()) break;
    
    cycle++;

    const CartesianDelta tension_delta = calculateTensionDelta(config);
    const CartesianDelta tear_delta = calculateTearDelta(config);
    const CartesianDelta grasp_delta = calculateMoveToGraspDelta(config, grasp_lateral_offset, grasp_approach_distance);

    RCLCPP_INFO(
      logger,
      "=== Zyklus %d/%zu: Greifen %s, Vorspannung Seite %.1f deg / "
      "Vertikal %.1f deg / Rueckzug %.3f m, Reissen %s %.1f deg ===",
      cycle,
      run_configs.size(),
      graspPositionToString(config).c_str(),
      config.tension_angle_deg,
      config.tension_vertical_angle_deg,
      config.pull_back_distance,
      directionToString(config.tear_direction).c_str(),
      config.tear_angle_deg);

    const std::string bag_path =
      bag_base_dir +
      "/run_" + std::to_string(cycle) +
      "_tension_side_" +
      std::to_string(static_cast<int>(config.tension_angle_deg)) +
      "_vertical_" +
      std::to_string(static_cast<int>(config.tension_vertical_angle_deg)) +
      "_back_mm_" +
      std::to_string(static_cast<int>(config.pull_back_distance * 1000.0)) +
      "_grasp_" + graspPositionToString(config) +
      "_tear_" + directionToString(config.tear_direction) +
      "_" + std::to_string(static_cast<int>(config.tear_angle_deg)) +
      "_" + std::to_string(node->now().nanoseconds());

    if (!arm.prompt(
        "Press 'Next' to move to START pose (or Ctrl+C to quit)"))
    {
      break;
    }

    if (!arm.moveToJoints(start_joints, "START")) {
      break;
    }

    RCLCPP_INFO(
      logger,
      "Fahre zum Greifpunkt %s: Delta [%.3f, %.3f, %.3f] m",
      graspPositionToString(config).c_str(),
      grasp_delta.x,
      grasp_delta.y,
      grasp_delta.z);

    if (!arm.moveCartesianDelta(
        grasp_delta.x,
        grasp_delta.y,
        grasp_delta.z,
        "MOVE TO GRASP"))
    {
      break;
    }

    std::this_thread::sleep_for(gripper_settle_time);

    if (!arm.gripperAction("close", "CLOSE")) {
      break;
    }

    if (!zeroFtSensor(node)) {
      RCLCPP_ERROR(logger, "FT-Sensor konnte nicht genullt werden.");
      break;
    }

    if (!arm.prompt("Press 'Next' for ROSBAG RECORD")) {
      break;
    }

    {
      RCLCPP_INFO(
        logger,
        "\033[1;32mRosbag-Aufnahme: %s\033[0m",
        bag_path.c_str());

      RosbagRecorder bag(bag_path, bag_options);

      RCLCPP_INFO(
        logger,
        "Vorspannung: Seite %.1f deg, Vertikal %.1f deg, Rueckzug %.3f m, "
        "Delta [%.3f, %.3f, %.3f] m",
        config.tension_angle_deg,
        config.tension_vertical_angle_deg,
        config.pull_back_distance,
        tension_delta.x,
        tension_delta.y,
        tension_delta.z);

      if (!arm.moveCartesianDelta(
          tension_delta.x,
          tension_delta.y,
          tension_delta.z,
          "FOIL TENSIONING"))
      {
        break;
      }

      RCLCPP_INFO(
        logger,
        "\033[1;33mWarte 2 Sekunden vor dem Reissen ...\033[0m");

      std::this_thread::sleep_for(tension_hold_time);

      RCLCPP_INFO(
        logger,
        "Reissen: %s, Winkel %.1f deg, Delta [%.3f, %.3f, %.3f] m",
        directionToString(config.tear_direction).c_str(),
        config.tear_angle_deg,
        tear_delta.x,
        tear_delta.y,
        tear_delta.z);

      if (!arm.moveCartesianDelta(
          tear_delta.x,
          tear_delta.y,
          tear_delta.z,
          "FOIL TEARING"))
      {
        break;
      }

      bag.stop();

      RCLCPP_INFO(
        logger,
        "\033[1;31mRosbag fuer Zyklus %d abgeschlossen.\033[0m",
        cycle);
    }

    if (!arm.gripperAction("open", "OPEN")) {
      break;
    }

    // Optional nach jedem Versuch zur sicheren HOME-Pose zurueckfahren:
    // if (!arm.moveToJoints(home_joints, "HOME")) {
    //   break;
    // }
  }

  RCLCPP_INFO(logger, "Beendet.");

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return 0;
}
