#include "pipeline/PipelineHelper.h"

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

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
