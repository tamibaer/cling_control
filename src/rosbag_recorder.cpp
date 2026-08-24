#include "cling_control/rosbag_recorder.hpp"

#include <stdexcept>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>

RosbagRecorder::RosbagRecorder(std::string uri)
: RosbagRecorder(std::move(uri), Options{})
{
}

RosbagRecorder::RosbagRecorder(std::string uri, Options options)
: uri_(std::move(uri)), options_(std::move(options))
{
  if (uri_.empty()) {
    throw std::invalid_argument("RosbagRecorder: uri must not be empty");
  }
  if (!options_.all_topics && options_.topics.empty()) {
    throw std::invalid_argument(
            "RosbagRecorder: topics must not be empty when all_topics is false");
  }

  start();
}

RosbagRecorder::~RosbagRecorder() noexcept
{
  stop();
}

void RosbagRecorder::start()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);

  if (recording_.load()) {
    return;
  }
  if (!rclcpp::ok()) {
    throw std::runtime_error(
            "RosbagRecorder: rclcpp is not initialized or has already shut down");
  }

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri_;
  storage_options.storage_id = options_.storage_id;
  storage_options.max_cache_size = options_.max_cache_size;

  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = options_.all_topics;
  record_options.topics = options_.topics;
  record_options.is_discovery_disabled = false;
  record_options.rmw_serialization_format = "cdr";
  record_options.topic_polling_interval = options_.topic_polling_interval;
  record_options.include_hidden_topics = options_.include_hidden_topics;
  record_options.include_unpublished_topics = options_.include_unpublished_topics;

  // Wichtig fuer einen eingebetteten Recorder: keine Tastatur- oder
  // Signalbehandlung des ros2-bag-CLI im Steuerungsprozess installieren.
  record_options.disable_keyboard_controls = true;

  try {
    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(writer, storage_options, record_options, node_name_);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(recorder_);
    executor_thread_ = std::thread([this]() {executor_->spin();});

    // In ROS 2 Jazzy startet record() die Aufnahme und kehrt danach zurueck.
    recorder_->record();
    recording_.store(true);
  } catch (...) {
    if (recorder_) {
      try {
        recorder_->stop();
      } catch (...) {
        // Die urspruengliche Exception aus start() wird weitergereicht.
      }
    }
    cleanup_after_failed_start();
    throw;
  }

  // Gibt der Topic-Discovery Zeit, bevor die eigentliche Roboterbewegung
  // gestartet wird. Nach Rueckkehr aus dem Konstruktor kann aufgezeichnet werden.
  std::this_thread::sleep_for(options_.warmup);
}

void RosbagRecorder::stop() noexcept
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);

  if (!recording_.exchange(false)) {
    cleanup_after_failed_start();
    return;
  }

  std::this_thread::sleep_for(options_.shutdown_delay);

  // stop() flusht die Caches und schliesst den Writer. Der Executor muss dabei
  // noch laufen, damit eingehende Samples bis zum Stop verarbeitet werden.
  if (recorder_) {
    try {
      recorder_->stop();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RosbagRecorder"),
        "Could not stop rosbag recorder cleanly: %s", exception.what());
    } catch (...) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RosbagRecorder"),
        "Could not stop rosbag recorder cleanly: unknown exception");
    }
  }

  cleanup_after_failed_start();
}

bool RosbagRecorder::recording() const noexcept
{
  return recording_.load();
}

const std::string & RosbagRecorder::uri() const noexcept
{
  return uri_;
}

void RosbagRecorder::cleanup_after_failed_start() noexcept
{
  recording_.store(false);

  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  if (executor_ && recorder_) {
    try {
      executor_->remove_node(recorder_);
    } catch (...) {
      // Aufraeumen im noexcept-Pfad darf keine weitere Exception ausloesen.
    }
  }

  executor_.reset();
  recorder_.reset();
}
