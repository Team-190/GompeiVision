#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>

#include "frc/geometry/Pose3d.h"
#include "io/ConfigInterface.h"
#include "nlohmann/json.hpp"

class PipelineHelper {
 public:
  PipelineHelper();
  ~PipelineHelper();

  static bool load_camera_intrinsics(const ConfigInterface& configInterface,
                                     cv::Mat& cameraMatrix,
                                     cv::Mat& distortionCoefficients);

  static std::map<int, frc::Pose3d> load_field_layout();
};