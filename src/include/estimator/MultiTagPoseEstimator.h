#pragma once

#include <map>
#include <optional>
#include <vector>

#include "util/CameraIntrinsics.h"
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
  // The FieldLayout is a map from a tag's integer ID to its known 3D pose on
  // the field.
  using FieldLayout = std::map<int, frc::Pose3d>;

  MultiTagPoseEstimator() = default;

  /**
   * @brief Estimates the camera's pose relative to the field origin.
   * @param observations A vector of all the 2D tag observations from a single
   * frame.
   * @param intrinsics The camera's calibration parameters.
   * @param field_layout The map defining the 3D location of every tag on the
   * field.
   * @param tag_size_m The size of the tags in meters.
   * @return A single, globally optimized camera pose observation.
   * Returns an empty optional if a pose cannot be calculated.
   */
  static std::optional<CameraPoseObservation> estimatePose(
      const std::vector<FiducialImageObservation>& observations,
      const CameraIntrinsics& intrinsics, const FieldLayout& field_layout,
      double tag_size_m);
};
