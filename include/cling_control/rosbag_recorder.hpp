#ifndef CLING_CONTROL__ROSBAG_RECORDER_HPP_
#define CLING_CONTROL__ROSBAG_RECORDER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rosbag2_transport/recorder.hpp>


class RosbagRecorder
{
public:
  struct Options
  {
    std::string storage_id{"mcap"};
    bool all_topics{true};
    std::vector<std::string> topics{};

    std::uint64_t max_cache_size{100ULL * 1024ULL * 1024ULL};
    std::chrono::milliseconds topic_polling_interval{50};
    std::chrono::milliseconds warmup{1200};
    std::chrono::milliseconds shutdown_delay{300};

    bool include_hidden_topics{false};
    bool include_unpublished_topics{false};
  };

  /// uri ist der Zielordner der Bag, beispielsweise /data/bags/run_001.
  explicit RosbagRecorder(std::string uri);
  RosbagRecorder(std::string uri, Options options);
  ~RosbagRecorder() noexcept;

  RosbagRecorder(const RosbagRecorder &) = delete;
  RosbagRecorder & operator=(const RosbagRecorder &) = delete;
  RosbagRecorder(RosbagRecorder &&) = delete;
  RosbagRecorder & operator=(RosbagRecorder &&) = delete;

  void start();
  void stop() noexcept;

  [[nodiscard]] bool recording() const noexcept;
  [[nodiscard]] const std::string & uri() const noexcept;

private:
  void cleanup_after_failed_start() noexcept;

  std::string uri_;
  Options options_;
  std::string node_name_{"cling_control_rosbag_recorder"};

  std::atomic_bool recording_{false};
  mutable std::mutex lifecycle_mutex_;

  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

#endif  // CLING_CONTROL__ROSBAG_RECORDER_HPP_
