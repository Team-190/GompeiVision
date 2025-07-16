#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTableInstance.h>

#include <string>

#include "util/QueuedFiducialData.h"

class NetworkTablesInterface {
 public:
  NetworkTablesInterface(const std::string& role);
  ~NetworkTablesInterface() = default;

  NetworkTablesInterface(const NetworkTablesInterface&) = delete;
  NetworkTablesInterface& operator=(const NetworkTablesInterface&) = delete;

  void publish_data(const AprilTagResult& result);

 private:
  nt::NetworkTableInstance m_nt_inst;
  nt::DoublePublisher m_fps_pub;
  nt::DoublePublisher m_latency_pub;
  nt::BooleanPublisher m_target_present_pub;
  nt::DoubleArrayPublisher m_pose_pub;
  nt::IntegerArrayPublisher m_tag_ids_pub;
  nt::DoublePublisher m_pose_ambiguity_pub;
};