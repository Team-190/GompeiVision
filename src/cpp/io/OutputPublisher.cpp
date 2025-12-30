#include "io/OutputPublisher.h"

#include <networktables/NetworkTableInstance.h>

#include <chrono>
#include <vector>

#include "util/PoseUtils.h"

namespace {
/**
 * @brief Helper function to serialize a Pose3d into a vector of doubles.
 * Appends translation (x,y,z) and rotation quaternion (w,x,y,z).
 * @param data The vector to append data to.
 * @param pose The pose to serialize.
 */
void AppendPoseData(std::vector<double>& data, const frc::Pose3d& pose) {
  data.push_back(pose.Translation().X().value());
  data.push_back(pose.Translation().Y().value());
  data.push_back(pose.Translation().Z().value());
  const auto& q = pose.Rotation().GetQuaternion();
  data.push_back(q.W());
  data.push_back(q.X());
  data.push_back(q.Y());
  data.push_back(q.Z());
}
}  // namespace

NTOutputPublisher::NTOutputPublisher(const std::string_view hardware_id,
                                     const nt::NetworkTableInstance& nt_inst) {
  const auto table =
      nt_inst.GetTable("/cameras/" + std::string(hardware_id) + "/output");

  constexpr nt::PubSubOptions options{
      .periodic = 0.01, .sendAll = true, .keepDuplicates = true};

  m_connection_status_pub = table->GetBooleanTopic("connected").Publish();
  m_observations_pub =
      table->GetDoubleArrayTopic("observations").Publish(options);
  m_capture_fps_pub = table->GetIntegerTopic("capture_fps").Publish();
  m_processing_fps_pub = table->GetIntegerTopic("processing_fps").Publish();
}

void NTOutputPublisher::SendConnectionStatus(const bool isConnected) {
  m_connection_status_pub.Set(isConnected);
}

void NTOutputPublisher::SendAprilTagResult(const AprilTagResult& result) {
  // --- Main observation data ---
  // This contains the multi-tag pose estimate and corner angles.
  std::vector<double> observation_data = {0.0};
  if (!PoseUtils::isPoseZero(result.multi_tag_pose.pose_0)) {
    const auto& obs = result.multi_tag_pose;
    // Data format: [pose_count, error_0, x, y, z, qw, qx, qy, qz, (error_1,
    // ...)]
    if (obs.pose_1.has_value()) {
      observation_data[0] = 2;  // Flag for two poses
    } else {
      observation_data[0] = 1;  // Flag for one pose
    }
    observation_data.push_back(obs.error_0);
    AppendPoseData(observation_data, obs.pose_0);

    if (obs.pose_1.has_value() && obs.error_1.has_value()) {
      observation_data.push_back(obs.error_1.value());
      AppendPoseData(observation_data, obs.pose_1.value());
    }
  }

  // Append tag angles data
  // Data format: [tag_id, c0_x, c0_y, c1_x, c1_y, ..., distance]
  for (const auto& [tag_id, corners_angles, distance] : result.tag_angles) {
    observation_data.push_back(tag_id);
    observation_data.insert(observation_data.end(), corners_angles.begin(),
                            corners_angles.end());
    observation_data.push_back(distance);
  }

  // Publish all data
  m_observations_pub.Set(observation_data, result.timestamp);
}

void NTOutputPublisher::SendCaptureFPS(const uint8_t& fps) {
  m_capture_fps_pub.Set(fps);
}

void NTOutputPublisher::SendProcessingFPS(const uint8_t& fps) {
  m_processing_fps_pub.Set(fps);
}
