#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cling_control/arm_controller.hpp"
#include "cling_control/rosbag_recorder.hpp"

// ===== Ziel-Konfigurationen =====
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
  // Vor dem Greifen auf die der Reissrichtung gegenueberliegende Seite fahren.
  OPPOSITE_SIDE,

  // Direkt von der mittleren START-Position zur Folie fahren.
  CENTER
};

enum class TearOutcome
{
  COMPLETE_EDGE_TEAR,
  PARTIAL_EDGE_TEAR,
  WRONG_TEAR_PATH,
  TEAR_AT_GRIPPER,
  NO_TEAR
};

struct RunConfig
{
  // Seitlicher Winkel der Vorspannung in der x-y-Ebene.
  double tension_angle_deg{0.0};

  // Vertikaler Winkel: positiv nach oben, negativ nach unten.
  double tension_vertical_angle_deg{0.0};

  // Exakte Strecke, die in -y-Richtung nach hinten gezogen wird.
  double pull_back_distance{0.10};

  double tear_angle_deg{40.0};
  double tear_distance{0.50};

  TearDirection tear_direction{TearDirection::RIGHT};
  GraspMode grasp_mode{GraspMode::OPPOSITE_SIDE};
};

struct ScheduledRun
{
  RunConfig config;
  std::size_t config_number{0};
  int repetition{0};
};

struct CartesianDelta
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double degreesToRadians(double angle_deg)
{
  constexpr double kPi = 3.14159265358979323846;
  return angle_deg * kPi / 180.0;
}

std::string directionToString(TearDirection direction)
{
  return direction == TearDirection::LEFT ? "left" : "right";
}

std::string graspPositionToString(const RunConfig & config)
{
  if (config.grasp_mode == GraspMode::CENTER) {
    return "center";
  }

  return config.tear_direction == TearDirection::LEFT ? "right" : "left";
}

std::string outcomeToString(TearOutcome outcome)
{
  switch (outcome) {
    case TearOutcome::COMPLETE_EDGE_TEAR:
      return "COMPLETE_EDGE_TEAR";
    case TearOutcome::PARTIAL_EDGE_TEAR:
      return "PARTIAL_EDGE_TEAR";
    case TearOutcome::WRONG_TEAR_PATH:
      return "WRONG_TEAR_PATH";
    case TearOutcome::TEAR_AT_GRIPPER:
      return "TEAR_AT_GRIPPER";
    case TearOutcome::NO_TEAR:
      return "NO_TEAR";
  }

  return "UNKNOWN";
}

std::optional<TearOutcome> askTearOutcome()
{
  // Bei einem ueber ros2 launch gestarteten Node ist std::cin haeufig nicht
  // mit dem sichtbaren Terminal verbunden. /dev/tty greift direkt auf das
  // steuernde Terminal zu und erlaubt dort weiterhin die Auswahl 1 bis 5.
  std::ifstream terminal_input("/dev/tty");
  if (!terminal_input.is_open()) {
    return std::nullopt;
  }

  while (true) {
    std::cout
      << "\n===== Ergebnis des Versuchs =====\n"
      << "1: Vollstaendig entlang der Kante gerissen\n"
      << "2: Teilweise entlang der Kante gerissen\n"
      << "3: Falscher Risspfad / nicht an der Kante\n"
      << "4: Folie direkt am Greifer gerissen\n"
      << "5: Kein Riss\n"
      << "Auswahl: "
      << std::flush;

    int selection = 0;
    if (terminal_input >> selection) {
      switch (selection) {
        case 1:
          return TearOutcome::COMPLETE_EDGE_TEAR;
        case 2:
          return TearOutcome::PARTIAL_EDGE_TEAR;
        case 3:
          return TearOutcome::WRONG_TEAR_PATH;
        case 4:
          return TearOutcome::TEAR_AT_GRIPPER;
        case 5:
          return TearOutcome::NO_TEAR;
        default:
          break;
      }
    } else if (terminal_input.eof()) {
      return std::nullopt;
    }

    terminal_input.clear();
    terminal_input.ignore(
      std::numeric_limits<std::streamsize>::max(),
      '\n');

    std::cout << "Ungueltige Eingabe. Bitte 1 bis 5 eingeben.\n";
  }
}

void appendResultCsv(
  const std::string & csv_path,
  const std::string & bag_path,
  const ScheduledRun & run,
  const std::string & outcome)
{
  const bool file_exists = std::filesystem::exists(csv_path);
  std::ofstream file(csv_path, std::ios::app);

  if (!file.is_open()) {
    throw std::runtime_error("Ergebnisdatei konnte nicht geoeffnet werden: " + csv_path);
  }

  if (!file_exists) {
    file
      << "bag_path,config,run,tension_side_deg,tension_vertical_deg,"
      << "pull_back_m,tear_angle_deg,tear_distance_m,tear_direction,"
      << "grasp_position,outcome\n";
  }

  const RunConfig & config = run.config;
  file
    << bag_path << ','
    << run.config_number << ','
    << run.repetition << ','
    << config.tension_angle_deg << ','
    << config.tension_vertical_angle_deg << ','
    << config.pull_back_distance << ','
    << config.tear_angle_deg << ','
    << config.tear_distance << ','
    << directionToString(config.tear_direction) << ','
    << graspPositionToString(config) << ','
    << outcome << '\n';
}

CartesianDelta calculateMoveToGraspDelta(
  const RunConfig & config,
  double lateral_offset,
  double approach_distance)
{
  if (config.grasp_mode == GraspMode::CENTER) {
    return {0.0, approach_distance, 0.0};
  }

  // Zum Reissen nach links wird rechts gegriffen und umgekehrt.
  const double x = config.tear_direction == TearDirection::LEFT ?
    lateral_offset : -lateral_offset;

  return {x, approach_distance, 0.0};
}

// Die Vorspannrichtung wird durch die Rueckzugsstrecke und zwei Winkel beschrieben:
//   tension_angle_deg:
//     0 Grad  -> gerade nach hinten in -y-Richtung
//     > 0 Grad -> zusaetzliche positive x-Komponente
//     < 0 Grad -> zusaetzliche negative x-Komponente
//   tension_vertical_angle_deg:
//     0 Grad  -> horizontal
//     > 0 Grad -> nach oben (+z)
//     < 0 Grad -> nach unten (-z)
CartesianDelta calculateTensionDelta(const RunConfig & config)
{
  const double side_angle_rad = degreesToRadians(config.tension_angle_deg);
  const double vertical_angle_rad =
    degreesToRadians(config.tension_vertical_angle_deg);

  // Die y-Komponente bleibt immer exakt -pull_back_distance. Die beiden
  // Winkel erzeugen unabhaengig davon seitliche und vertikale Komponenten.
  return {
    config.pull_back_distance * std::tan(side_angle_rad),
    -config.pull_back_distance,
    config.pull_back_distance * std::tan(vertical_angle_rad)
  };
}

// Der Reisswinkel liegt in der x-z-Ebene:
//   0 Grad  -> horizontal entlang x
//   90 Grad -> senkrecht nach oben
// LEFT/RIGHT bestimmt das Vorzeichen der x-Komponente.
CartesianDelta calculateTearDelta(const RunConfig & config)
{
  const double angle_rad = degreesToRadians(config.tear_angle_deg);
  const double direction_sign =
    config.tear_direction == TearDirection::RIGHT ? 1.0 : -1.0;

  return {
    direction_sign * config.tear_distance * std::cos(angle_rad),
    0.0,
    config.tear_distance * std::sin(angle_rad)
  };
}

bool zeroFtSensor(const rclcpp::Node::SharedPtr & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>(
    "/io_and_status_controller/zero_ftsensor");

  client->wait_for_service();

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  future.wait();

  return future.get()->success;
}

void waitForNextStep(
  const rclcpp::Logger & logger,
  const std::chrono::seconds & delay,
  const std::string & next_step)
{
  RCLCPP_INFO(
    logger,
    "\033[1;33mWarte %lld Sekunden: danach %s ...\033[0m",
    static_cast<long long>(delay.count()),
    next_step.c_str());

  std::this_thread::sleep_for(delay);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "cling_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("cling_control_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto spinner = std::thread([&executor]() {
      executor.spin();
    });

  ArmController arm(
    node,
    logger,
    "ur5e_arm",
    "robotiq_gripper");

  const std::string bag_base_dir =
    "/mnt/dattelspeicher/rosbags/tearing_experiment_test";

  const std::string results_csv = bag_base_dir + "/results.csv";

  std::filesystem::create_directories(bag_base_dir);

  const auto step_delay = std::chrono::seconds(1);
  const auto tension_hold_time = std::chrono::seconds(2);
  const auto post_tear_recording_time = std::chrono::milliseconds(500);
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
    {-20.0, -15.0, 0.12, 40.0, 0.50, TearDirection::LEFT, GraspMode::OPPOSITE_SIDE},
    {20.0, 0.0, 0.08, 40.0, 0.50, TearDirection::RIGHT, GraspMode::OPPOSITE_SIDE},
    {0.0, 0.0, 0.10, 40.0, 0.50, TearDirection::RIGHT, GraspMode::OPPOSITE_SIDE},
    {0.0, 0.0, 0.10, 40.0, 0.50, TearDirection::LEFT, GraspMode::OPPOSITE_SIDE},
    {0.0, 0.0, 0.10, 40.0, 0.50, TearDirection::RIGHT, GraspMode::CENTER},
    {0.0, 0.0, 0.10, 40.0, 0.50, TearDirection::LEFT, GraspMode::CENTER}
  };

  constexpr int repetitions_per_config = 5;

  // Jede Konfiguration wird fuenfmal direkt hintereinander eingeplant.
  std::vector<ScheduledRun> scheduled_runs;
  scheduled_runs.reserve(run_configs.size() * repetitions_per_config);

  for (std::size_t config_index = 0;
    config_index < run_configs.size();
    ++config_index)
  {
    for (int repetition = 1;
      repetition <= repetitions_per_config;
      ++repetition)
    {
      scheduled_runs.push_back(
        {run_configs[config_index], config_index + 1, repetition});
    }
  }

  int cycle = 0;

  for (const auto & scheduled_run : scheduled_runs) {
    if (!rclcpp::ok()) {
      break;
    }

    cycle++;

    const RunConfig & config = scheduled_run.config;

    const CartesianDelta tension_delta = calculateTensionDelta(config);
    const CartesianDelta tear_delta = calculateTearDelta(config);
    const CartesianDelta grasp_delta = calculateMoveToGraspDelta(
      config, grasp_lateral_offset, grasp_approach_distance);

    RCLCPP_INFO(
      logger,
      "=== Zyklus %d/%zu: Config %zu/%zu, Wiederholung %d/%d, Greifen %s, "
      "Vorspannung Seite %.1f deg / "
      "Vertikal %.1f deg / Rueckzug %.3f m, Reissen %s %.1f deg ===",
      cycle,
      scheduled_runs.size(),
      scheduled_run.config_number,
      run_configs.size(),
      scheduled_run.repetition,
      repetitions_per_config,
      graspPositionToString(config).c_str(),
      config.tension_angle_deg,
      config.tension_vertical_angle_deg,
      config.pull_back_distance,
      directionToString(config.tear_direction).c_str(),
      config.tear_angle_deg);

    const std::string config_dir =
      bag_base_dir + "/config_" + std::to_string(scheduled_run.config_number);

    std::filesystem::create_directories(config_dir);

    const std::string bag_path =
      config_dir +
      "/run_" + std::to_string(scheduled_run.repetition) +
      "_" + std::to_string(node->now().nanoseconds());

    waitForNextStep(logger, step_delay, "Fahrt zur START-Pose");

    if (!arm.moveToJoints(start_joints, "START")) {
      break;
    }

    //waitForNextStep(logger, step_delay, "Fahrt zum Greifpunkt");

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

    waitForNextStep(logger, step_delay, "Greifer oeffnen");

    if (!arm.gripperAction("open", "OPEN")) {
      break;
    }

    if (!arm.prompt(
        "Folie an der Greifposition einlegen/vorbereiten und dann 'Next' druecken"))
    {
      break;
    }

    waitForNextStep(logger, step_delay, "Greifer schliessen");

    if (!arm.gripperAction("close", "CLOSE")) {
      break;
    }

    waitForNextStep(logger, step_delay, "FT-Sensor nullen");

    if (!zeroFtSensor(node)) {
      RCLCPP_ERROR(logger, "FT-Sensor konnte nicht genullt werden.");
      break;
    }

    waitForNextStep(logger, step_delay, "Rosbag-Aufnahme und Vorspannung");

    {
      RCLCPP_INFO(
        logger,
        "\033[1;32mRosbag-Aufnahme: %s\033[0m",
        bag_path.c_str());

      RosbagRecorder bag(bag_path, bag_options);

      // Orientierung vor der (moeglicherweise geneigten) Vorspannung merken, um sie
      // vor dem eigentlichen Reissen wieder herstellen zu koennen.
      const auto pre_tension_orientation = arm.getCurrentOrientation();

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

      if (!arm.moveCartesianDeltaAlignedVertical(
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
        "\033[1;33mGreifer wird vor dem Reissen wieder gerade gestellt ...\033[0m");

      if (!arm.moveToOrientation(pre_tension_orientation, "STRAIGHTEN BEFORE TEAR")) {
        break;
      }

      RCLCPP_INFO(
        logger,
        "Reissen: %s, Winkel %.1f deg, Delta [%.3f, %.3f, %.3f] m",
        directionToString(config.tear_direction).c_str(),
        config.tear_angle_deg,
        tear_delta.x,
        tear_delta.y,
        tear_delta.z);

      if (!arm.moveCartesianDeltaAligned(
          tear_delta.x,
          tear_delta.y,
          tear_delta.z,
          "FOIL TEARING"))
      {
        break;
      }

      RCLCPP_INFO(
        logger,
        "\033[1;33mZeichne noch 500 ms Nachlauf auf ...\033[0m");

      std::this_thread::sleep_for(post_tear_recording_time);

      bag.stop();

      RCLCPP_INFO(
        logger,
        "\033[1;31mRosbag fuer Zyklus %d abgeschlossen.\033[0m",
        cycle);

      // Erst nach dem Schliessen der Bag labeln. Die Dauer der manuellen
      // Eingabe beeinflusst dadurch nicht die Laenge der Aufnahme.
      const auto outcome = askTearOutcome();
      if (!outcome.has_value()) {
        RCLCPP_ERROR(
          logger,
          "Keine Konsoleneingabe fuer das Ergebnis verfuegbar. Versuch wird beendet.");
        break;
      }

      const std::string outcome_text = outcomeToString(*outcome);

      RCLCPP_INFO(
        logger,
        "\033[1;36mVersuch gelabelt als: %s\033[0m",
        outcome_text.c_str());

      try {
        appendResultCsv(
          results_csv,
          bag_path,
          scheduled_run,
          outcome_text);
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          logger,
          "Ergebnis konnte nicht in CSV gespeichert werden: %s",
          exception.what());
      }
    }

    waitForNextStep(logger, step_delay, "naechsten Versuch");

    // Optional nach jedem Versuch zur sicheren HOME-Pose zurueckfahren:
    // if (!arm.moveToJoints(home_joints, "HOME")) {
    //   break;
    // }
  }

  // Nach dem letzten regulaeren Lauf gibt es keine naechste START-Pose mehr.
  // Deshalb den Greifer vor dem Beenden noch einmal kontrolliert oeffnen.
  if (rclcpp::ok()) {
    waitForNextStep(logger, step_delay, "Greifer abschliessend oeffnen");
    if (!arm.gripperAction("open", "FINAL OPEN")) {
      RCLCPP_WARN(logger, "Abschliessendes Oeffnen des Greifers fehlgeschlagen.");
    }
  }

  RCLCPP_INFO(logger, "Beendet.");

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return 0;
}