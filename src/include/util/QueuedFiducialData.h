#pragma once

#include <frc/geometry/Pose3d.h>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

/**
 * @struct FiducialImageObservation
 * @brief The raw 2D detection of tags in an image.
 */
struct FiducialImageObservation {
  std::vector<int> tag_ids;
  // Corners stored as as vectors [x1, y1, x2, y2, x3, y3, x4, y4]
  std::vector<std::vector<double>> corners_pixels;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
  std::string camera_role;
};

/**
 * @struct FiducialPoseObservation
 * @brief The estimated 3D pose(s) of a single tag relative to the camera.
 */
struct FiducialPoseObservation {
  int tag_id;
  frc::Pose3d pose_0;
  double error_0;
  std::optional<frc::Pose3d> pose_1;  // Second ambiguous pose
  std::optional<double> error_1;
};

/**
 * @struct CameraPoseObservation
 * @brief The final estimated 3D pose of the camera relative to the field,
 * calculated from multiple tags.
 */
struct CameraPoseObservation {
  std::vector<int> tag_ids;
  frc::Pose3d pose_0;
  double error_0;
  std::optional<frc::Pose3d> pose_1;
  std::optional<double> error_1;
};

/**
 * @struct TagAngleObservation
 * @brief The observed angles and distance to a single AprilTag.
 */
struct TagAngleObservation {
  int tag_id;
  std::vector<double> corners_angles;
  double distance;
};

// --- Main Result Data Packet ---
// These are the structs that will be placed into the ThreadSafeQueue
// for the NetworkTables thread to consume.

/**
 * @struct AprilTagResult
 * @brief A complete data packet containing all results from the AprilTag
 * pipeline for a single captured frame.
 */
struct AprilTagResult {
  std::vector<int> tags;
  std::string camera_role;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
  int fps;

  // The final, high-quality multi-tag pose estimate.
  std::optional<CameraPoseObservation> multi_tag_pose;

  // A list of all the individual tag poses found in the frame.
  std::vector<FiducialPoseObservation> single_tag_poses;

  // A list of all tags seen in the frame, with their corner angles and
  // distances.
  std::vector<TagAngleObservation> tag_angles;
};