#include "pipeline/PipelineHelper.h"

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Quaternion.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/nt/NetworkTableType.hpp>
#include <wpi/units/length.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "io/ConfigInterface.h"

PipelineHelper::PipelineHelper() {}
PipelineHelper::~PipelineHelper() {}

bool PipelineHelper::load_camera_intrinsics(
    const ConfigInterface& configInterface, cv::Mat& cameraMatrix,
    cv::Mat& distortionCoefficients) {
  cameraMatrix = configInterface.getCameraMatrix();
  distortionCoefficients = configInterface.getDistortionCoeffs();

  if (cameraMatrix.empty() || distortionCoefficients.empty()) {
    std::cerr << "[PipelineHelper] ERROR: Camera intrinsics (matrix or "
                 "distortion coefficients) are empty."
              << std::endl;
    return false;
  }

  return true;
}
