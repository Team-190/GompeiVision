#pragma once

#include <optional>

#include "detector/FiducialDetector.h"

/**
 * @class SingleTagPoseEstimator
 * @brief Estimates the 3D pose of a single AprilTag.
 *
 * This class takes the 2D corners of a single detected tag and calculates
 * its 3D position and orientation relative to the camera. It handles the
 * ambiguity problem that occurs when a tag is viewed nearly head-on by
 * returning both possible solutions.
 */
class SingleTagPoseEstimator {
 public:
  SingleTagPoseEstimator() = default;

  /**
   * @brief Estimates the 3D pose(s) for a single fiducial marker.
   * @param observation The 2D observation of the tag's corners in the image.
   * @param result Reference to the current AprilTagResult object
   * @param cameraMatrix The camera matrix
   * @param distCoeffs The distortion coefficients
   * @param tag_size_m The size of the tag in meters.
   * @return A struct containing the two possible poses and their respective
   * errors. Returns an empty optional if a pose cannot be calculated.
   */
  static void estimatePose(const FiducialImageObservation& observation,
                           AprilTagResult& result, const cv::Mat& cameraMatrix,
                           const cv::Mat& distCoeffs, double tag_size_m);
};
