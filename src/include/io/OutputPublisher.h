#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>

#include <string>
#include <string_view>

#include "../util/QueuedFiducialData.h"
#include "../util/QueuedObjectData.h"

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

  /**
   * @brief Sends a complete object detection result packet.
   * @param result The object detection result data for a single frame.
   */
  virtual void SendObjectDetectResult(const ObjectDetectResult& result) = 0;

  /**
   * @brief Sends the current camera connection status.
   * @param isConnected True if the camera is connected, false otherwise.
   */
  virtual void sendConnectionStatus(bool isConnected) = 0;
};

/**
 * @class NTOutputPublisher
 * @brief Publishes vision pipeline results to NetworkTables.
 */
class NTOutputPublisher final : public OutputPublisher {
 public:
  explicit NTOutputPublisher(const std::string_view hardware_id, const nt::NetworkTableInstance& nt_inst);
  void SendAprilTagResult(const AprilTagResult& result) override;
  void SendObjectDetectResult(const ObjectDetectResult& result) override;
  void sendConnectionStatus(bool isConnected) override;

 private:
  nt::DoubleArrayPublisher observations_pub_;
  nt::DoubleArrayPublisher objects_pub_;
  nt::IntegerPublisher apriltags_fps_pub_;
  nt::IntegerPublisher objects_fps_pub_;
  nt::BooleanPublisher connection_status_pub_;
};
