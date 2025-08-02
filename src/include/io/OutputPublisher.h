#pragma once

#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>

#include <string>
#include <string_view>

#include "../util/QueuedFiducialData.h"

/**
 * @class OutputPublisher
 * @brief Abstract base class for publishing vision pipeline results.
 */
class OutputPublisher {
 public:
  virtual ~OutputPublisher() = default;

  /**
   * @brief Sends a complete AprilTag result packet.
   * @param result The AprilTag result data for a single frame.
   */
  virtual void SendAprilTagResult(const AprilTagResult& result) = 0;
};

/**
 * @class NTOutputPublisher
 * @brief Publishes vision pipeline results to NetworkTables.
 */
class NTOutputPublisher final : public OutputPublisher {
 public:
  explicit NTOutputPublisher(std::string_view hardware_id);
  void SendAprilTagResult(const AprilTagResult& result) override;

 private:
  nt::DoubleArrayPublisher observations_pub_;
  nt::IntegerPublisher apriltags_fps_pub_;
};
