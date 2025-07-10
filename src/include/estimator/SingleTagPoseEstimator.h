#pragma once

#include <optional>

#include "detector/FiducialDetector.h"
#include "util/CameraIntrinsics.h"

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
   * @param intrinsics The camera's calibration parameters.
   * @param tag_size_m The size of the tag in meters.
   * @return A struct containing the two possible poses and their respective
   * errors. Returns an empty optional if a pose cannot be calculated.
   */
  static std::optional<FiducialPoseObservation> estimatePose(
      const FiducialImageObservation& observation,
      const CameraIntrinsics& intrinsics, double tag_size_m);
};
