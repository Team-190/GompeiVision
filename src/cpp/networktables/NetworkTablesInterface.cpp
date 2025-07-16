#include "networktables//NetworkTablesInterface.h"

#include <algorithm>
#include <iostream>
#include <vector>

NetworkTablesInterface::NetworkTablesInterface(const std::string& role) {
  m_nt_inst = nt::NetworkTableInstance::GetDefault();
  m_nt_inst.StartClient4("GompeiVision");
  // For FRC, use m_nt_inst.SetServerTeam(YOUR_TEAM_NUMBER);
  // For local testing, use a specific IP or "localhost".
  m_nt_inst.SetServer("localhost");

  const auto table = m_nt_inst.GetTable("GompeiVision")->GetSubTable(role);
  nt::PubSubOptions options;
  options.keepDuplicates = true;  // Send data even if it hasn't changed
  options.sendAll = true;

  m_fps_pub = table->GetDoubleTopic("fps").Publish(options);
  m_latency_pub = table->GetDoubleTopic("latency_ms").Publish(options);
  m_target_present_pub =
      table->GetBooleanTopic("target_present").Publish(options);
  // Pose is published as a flat array: [x, y, z, qW, qX, qY, qZ]
  m_pose_pub = table->GetDoubleArrayTopic("pose").Publish(options);
  m_tag_ids_pub = table->GetIntegerArrayTopic("tag_ids_used").Publish(options);
  // Using reprojection error as an ambiguity/quality metric
  m_pose_ambiguity_pub =
      table->GetDoubleTopic("pose_ambiguity").Publish(options);

  std::cout << "[" << role << "] NetworkTables interface initialized."
            << std::endl;
}

void NetworkTablesInterface::publish_data(const AprilTagResult& result) {
  // --- Publish Telemetry ---
  m_fps_pub.Set(result.fps);

  const auto now = std::chrono::steady_clock::now();
  const auto latency =
      std::chrono::duration<double, std::milli>(now - result.timestamp).count();
  m_latency_pub.Set(latency);

  // --- Determine Best Pose to Publish ---
  std::optional<frc::Pose3d> best_pose;
  std::vector<int> ids_used;
  double best_error = 1e6;

  // A multi-tag solution is always preferred as it's non-ambiguous.
  if (result.multi_tag_pose && !result.multi_tag_pose->tag_ids.empty()) {
    best_pose = result.multi_tag_pose->pose_0;
    ids_used = result.multi_tag_pose->tag_ids;
    best_error = result.multi_tag_pose->error_0;
  } else if (!result.single_tag_poses.empty()) {
    // If no multi-tag, find the best single-tag observation (lowest error).
    const auto best_obs_it = std::ranges::min_element(
        result.single_tag_poses,
        [](const auto& a, const auto& b) { return a.error_0 < b.error_0; });

    // From that best observation, choose the pose with the lower error.
    if (best_obs_it->error_1 && *best_obs_it->error_1 < best_obs_it->error_0) {
      best_pose = best_obs_it->pose_1;
      best_error = *best_obs_it->error_1;
    } else {
      best_pose = best_obs_it->pose_0;
      best_error = best_obs_it->error_0;
    }
    ids_used.push_back(best_obs_it->tag_id);
  }

  // --- Publish Pose Data ---
  if (best_pose) {
    m_target_present_pub.Set(true);

    const auto& translation = best_pose->Translation();
    const auto& quaternion = best_pose->Rotation().GetQuaternion();
    // Create a vector to hold the data. The Set method expects a std::span,
    // and a std::vector can be implicitly converted to it.
    std::vector<double> pose_data = {
        translation.X().value(), translation.Y().value(),
        translation.Z().value(), quaternion.W(),      quaternion.X(),
        quaternion.Y(),          quaternion.Z()};
    m_pose_pub.Set(pose_data);

    m_tag_ids_pub.Set(std::vector<int64_t>(ids_used.begin(), ids_used.end()));
    m_pose_ambiguity_pub.Set(best_error);
  } else {
    m_target_present_pub.Set(false);
    m_pose_pub.Set({});
    m_tag_ids_pub.Set({});
    m_pose_ambiguity_pub.Set(0.0);
  }
}