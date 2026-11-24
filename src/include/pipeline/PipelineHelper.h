#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>

#include <wpi/math/geometry/Pose3d.hpp>
#include "io/ConfigInterface.h"
#include "nlohmann/json.hpp"

class PipelineHelper {
 public:
  PipelineHelper();
  ~PipelineHelper();

  static bool load_camera_intrinsics(const ConfigInterface& configInterface,
                                     cv::Mat& cameraMatrix,
                                     cv::Mat& distortionCoefficients);

  static std::map<int, wpi::math::Pose3d> load_field_layout();
};