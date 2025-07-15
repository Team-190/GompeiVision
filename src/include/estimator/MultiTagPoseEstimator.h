#pragma once

#include <map>
#include <optional>
#include <vector>

#include "fieldImages/fields/fields.h"
#include "opencv2/opencv.hpp"
#include "util/QueuedFiducialData.h"

/**
 * @class MultiTagPoseEstimator
 * @brief Estimates the camera's 3D pose using all visible AprilTags.
 *
 * This class performs the final, high-quality pose estimation by using all
 * visible tags in a single solvePnP calculation. This provides a more stable
 * and accurate result than any single tag could.
 */
class MultiTagPoseEstimator {
 public:
  MultiTagPoseEstimator() = default;

  /**
   * @brief Estimates the camera's pose relative to the field origin.
   * @param observation A representation of all the 2D tag observations from a
   * single frame.
   * @param result Reference to the current AprilTagResult object.
   * @param cameraMatrix Reference to the current camera matrix
   * @param distCoeffs Reference to the current distortion coefficients
   * @param tag_size_m The size of the tags in meters.
   * @param field_layout The map defining the 3D location of every tag on the
   * field.
   * @return A single, globally optimized camera pose observation.
   * Returns an empty optional if a pose cannot be calculated.
   */
  static void estimatePose(const FiducialImageObservation& observation,
                           AprilTagResult& result, const cv::Mat& cameraMatrix,
                           const cv::Mat& distCoeffs, double tag_size_m,
                           const std::map<int, frc::Pose3d>& field_layout);
};
