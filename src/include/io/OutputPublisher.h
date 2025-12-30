#pragma once

#include <networktables/BooleanTopic.h>
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
   * @brief Sends the current camera connection status.
   * @param isConnected True if the camera is connected, false otherwise.
   */
  virtual void SendConnectionStatus(bool isConnected) = 0;

  /**
   * @brief Sends a complete AprilTag result packet.
   * @param result The AprilTag result data for a single frame.
   */
  virtual void SendAprilTagResult(const AprilTagResult& result) = 0;

  virtual void SendCaptureFPS(const uint8_t& fps) = 0;
  virtual void SendProcessingFPS(const uint8_t& fps) = 0;
};

/**
 * @class NTOutputPublisher
 * @brief Publishes vision pipeline results to NetworkTables.
 */
class NTOutputPublisher final : public OutputPublisher {
 public:
  explicit NTOutputPublisher(const std::string_view hardware_id, const nt::NetworkTableInstance& nt_inst);
  void SendConnectionStatus(bool isConnected) override;
  void SendAprilTagResult(const AprilTagResult& result) override;
  void SendCaptureFPS(const uint8_t& fps) override;
  void SendProcessingFPS(const uint8_t& fps) override;

 private:
  nt::BooleanPublisher m_connection_status_pub;
  nt::DoubleArrayPublisher m_observations_pub;
  nt::IntegerPublisher m_capture_fps_pub;
  nt::IntegerPublisher m_processing_fps_pub;
};
