#pragma once

#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>

#include <string>

#include "../util/QueuedFiducialData.h"

namespace config {
// NOTE: This is a placeholder for your actual ConfigStore implementation.
// It is assumed to have a structure that can provide a 'device_id'.
struct ConfigStore {
  struct LocalConfig {
    std::string device_id;
  } local_config;
};
}  // namespace config

/**
 * @class OutputPublisher
 * @brief Abstract base class for publishing vision pipeline results.
 */
class OutputPublisher {
 public:
  virtual ~OutputPublisher() = default;

  /**
   * @brief Sends a complete AprilTag result packet.
   * @param config_store The system configuration.
   * @param result The AprilTag result data for a single frame.
   */
  virtual void SendAprilTagResult(const config::ConfigStore& config_store,
                                  const AprilTagResult& result) = 0;
};

/**
 * @class NTOutputPublisher
 * @brief Publishes vision pipeline results to NetworkTables.
 */
class NTOutputPublisher final : public OutputPublisher {
 public:
  void SendAprilTagResult(const config::ConfigStore& config_store,
                          const AprilTagResult& result) override;

 private:
  void CheckInit(const config::ConfigStore& config_store);

  bool init_complete_ = false;
  nt::DoubleArrayPublisher observations_pub_;
  nt::IntegerPublisher apriltags_fps_pub_;
  // nt::IntegerPublisher objdetect_fps_pub_;
  // nt::DoubleArrayPublisher objdetect_observations_pub_;
};