#pragma once
#include <frc/geometry/Pose3d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>

#include <map>

class FieldInterface {
public:
  // All-static interface
  static void initialize(const nt::NetworkTableInstance& nt_inst);
  static bool update();
  static std::map<int, frc::Pose3d> getMap();
  static bool isInitialized();

  static bool m_initialized;

private:
  inline static int numTags = 0;
  inline static std::map<int, frc::Pose3d> tagMap = {};
  inline static std::map<int, nt::DoubleArraySubscriber> subscribers = {};
};
