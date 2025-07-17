#pragma once

#include <opencv2/core/mat.hpp>

#include "util/QueuedFiducialData.h"

/**
 * @class TagAngleCalculator
 * @brief Calculates the viewing angles to the corners of an AprilTag and its
 * distance from the camera. This is ported from the Python implementation.
 */
class TagAngleCalculator {
 public:
  /**
   * @brief Calculates angles and distances for all detected tags.
   *
   * For each tag in the observation, this function:
   * 1. Undistorts the 2D corner points.
   * 2. Calculates the horizontal and vertical viewing angle to each corner.
   * 3. Performs a single-tag pose estimation (IPPE) to find the distance.
   * 4. Populates the `tag_angles` vector in the provided `AprilTagResult`.
   *
   * @param observation The raw fiducial detection from the camera.
   * @param result The result packet to be populated.
   * @param cameraMatrix The camera's intrinsic matrix.
   * @param distCoeffs The camera's distortion coefficients.
   * @param tag_size_m The physical size of the tags in meters.
   */
  static void calculate(const FiducialImageObservation& observation,
                        AprilTagResult& result, const cv::Mat& cameraMatrix,
                        const cv::Mat& distCoeffs, double tag_size_m);
};