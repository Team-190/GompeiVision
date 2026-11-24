#pragma once
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/nt/DoubleArrayTopic.hpp>

#include <map>

class FieldInterface {
public:
  // All-static interface
  static void initialize(const wpi::nt::NetworkTableInstance& nt_inst);
  static bool update();
  static std::map<int, wpi::math::Pose3d> getMap();
  static bool isInitialized();

  static bool m_initialized;

private:
  inline static int numTags = 0;
  inline static std::map<int, wpi::math::Pose3d> tagMap = {};
  inline static std::map<int, wpi::nt::DoubleArraySubscriber> subscribers = {};
};
