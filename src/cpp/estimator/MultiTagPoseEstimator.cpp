#include "estimator/MultiTagPoseEstimator.h"

#include <frc/geometry/Transform3d.h>

#include <opencv2/calib3d.hpp>
#include <vector>

#include "util/PoseUtils.h"

/**
 * @brief Helper function to get the 3D coordinates of a tag's corners in the
 * world frame.
 * @param tag_pose The known 3D pose of the tag on the field.
 * @param tag_size_m The size of the tag in meters.
 * @return A vector of 4 cv::Point3d objects representing the world coordinates
 * of the corners.
 */
static std::vector<cv::Point3d> get_world_corners(const frc::Pose3d& tag_pose,
                                                  double tag_size_m) {
  const double half_size = tag_size_m / 2.0;

  // Define the corners in the tag's own local coordinate system (origin at
  // center, on XY plane)
  std::vector<frc::Translation3d> local_corners;
  local_corners.push_back(frc::Translation3d(units::meter_t(-half_size),
                                             units::meter_t(-half_size),
                                             units::meter_t(0)));
  local_corners.push_back(frc::Translation3d(units::meter_t(half_size),
                                             units::meter_t(-half_size),
                                             units::meter_t(0)));
  local_corners.push_back(frc::Translation3d(
      units::meter_t(half_size), units::meter_t(half_size), units::meter_t(0)));
  local_corners.push_back(frc::Translation3d(units::meter_t(-half_size),
                                             units::meter_t(half_size),
                                             units::meter_t(0)));

  // Transform each local corner point by the tag's pose to get its world
  // coordinate
  std::vector<cv::Point3d> world_corners;
  for (const auto& corner_translation : local_corners) {
    // A transform representing just the translation from the tag center to the
    // corner
    frc::Transform3d local_transform(corner_translation, frc::Rotation3d());

    // Apply this transform to the tag's world pose
    frc::Pose3d world_corner_pose = tag_pose + local_transform;

    world_corners.push_back(cv::Point3d(world_corner_pose.X().value(),
                                        world_corner_pose.Y().value(),
                                        world_corner_pose.Z().value()));
  }
  return world_corners;
}

std::optional<CameraPoseObservation> MultiTagPoseEstimator::estimatePose(
    const std::vector<FiducialImageObservation>& observations,
    const CameraIntrinsics& intrinsics, const FieldLayout& field_layout,
    double tag_size_m) {
  std::vector<cv::Point3d> all_object_points;
  std::vector<cv::Point2d> all_image_points;
  std::vector<int> all_tag_ids;

  // 1. Collect all the 3D world points and 2D image points from the
  // observations.
  for (const auto& [tag_id, corners_pixels] : observations) {
    auto it = field_layout.find(tag_id);
    if (it == field_layout.end()) {
      continue;  // This tag is not on the field layout, so we can't use it.
    }
    const frc::Pose3d& tag_pose = it->second;

    std::vector<cv::Point3d> world_corners =
        get_world_corners(tag_pose, tag_size_m);
    all_object_points.insert(all_object_points.end(), world_corners.begin(),
                             world_corners.end());

    for (size_t i = 0; i < corners_pixels.size(); i += 2) {
      all_image_points.push_back(
          cv::Point2d(corners_pixels[i], corners_pixels[i + 1]));
    }

    all_tag_ids.push_back(tag_id);
  }

  if (all_object_points.size() < 4) {
    return std::nullopt;
  }

  // 2. Prepare the camera matrix and distortion coefficients.
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.cx, 0,
       intrinsics.fy, intrinsics.cy, 0, 0, 1);
  auto dist_coeffs = cv::Mat(intrinsics.dist_coeffs);

  // 3. Run a single, high-quality solvePnP with all the points.
  cv::Mat rvec, tvec;
  cv::solvePnP(all_object_points, all_image_points, camera_matrix, dist_coeffs,
               rvec, tvec, false, cv::SOLVEPNP_SQPNP);

  // 4. Convert the result to the final camera pose.
  frc::Pose3d field_origin_in_camera_frame =
      PoseUtils::rvec_tvec_to_pose3d(rvec, tvec);
  frc::Transform3d transform_field_to_camera(
      field_origin_in_camera_frame.Translation(),
      field_origin_in_camera_frame.Rotation());
  frc::Transform3d transform_camera_to_field =
      transform_field_to_camera.Inverse();
  frc::Pose3d camera_in_field_pose(transform_camera_to_field.Translation(),
                                   transform_camera_to_field.Rotation());

  std::vector<cv::Point2d> reprojected_points;
  cv::projectPoints(all_object_points, rvec, tvec, camera_matrix, dist_coeffs,
                    reprojected_points);

  double total_error = 0.0;
  for (size_t i = 0; i < all_image_points.size(); ++i) {
    // Calculate the Euclidean distance between the original detected point
    // and the reprojected point.
    total_error += cv::norm(all_image_points[i] - reprojected_points[i]);
  }
  double average_error = total_error / all_image_points.size();

  CameraPoseObservation final_observation;
  final_observation.tag_ids = all_tag_ids;
  final_observation.pose_0 = camera_in_field_pose;
  final_observation.error_0 = average_error;  // Populate the error field

  return final_observation;
}