#include "pipeline/PipelineHelper.h"

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <networktables/NetworkTableType.h>
#include <units/length.h>

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
