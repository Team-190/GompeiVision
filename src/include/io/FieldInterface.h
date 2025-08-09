#pragma once
#include <frc/geometry/Pose3d.h>

#include "ConfigInterface.h"

class FieldInterface {
 public:
  FieldInterface();
  ~FieldInterface();

  void update();

  std::map<int, frc::Pose3d> getMap();

 private:
  int numTags;
  std::map<int, frc::Pose3d> tagMap;
};