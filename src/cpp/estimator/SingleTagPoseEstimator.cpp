#include "estimator/SingleTagPoseEstimator.h"

#include <frc/geometry/Pose3d.h>
#include <util/CameraIntrinsics.h>
#include <util/PoseUtils.h>

#include <opencv2/calib3d.hpp>
#include <vector>

#include "util/QueuedFiducialData.h"

std::optional<FiducialPoseObservation> SingleTagPoseEstimator::estimatePose(
    const FiducialImageObservation& observation,
    const CameraIntrinsics& intrinsics, double tag_size_m) {
  // 1. Define the 3D "object points" for a generic tag of the specified size.
  double half_tag_size = tag_size_m / 2.0;
  std::vector<cv::Point3d> object_points;
  object_points.push_back(cv::Point3d(-half_tag_size, -half_tag_size, 0));
  object_points.push_back(cv::Point3d(half_tag_size, -half_tag_size, 0));
  object_points.push_back(cv::Point3d(half_tag_size, half_tag_size, 0));
  object_points.push_back(cv::Point3d(-half_tag_size, half_tag_size, 0));

  // 2. Prepare the camera matrix and distortion coefficients for OpenCV.
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.cx, 0,
       intrinsics.fy, intrinsics.cy, 0, 0, 1);
  auto dist_coeffs = cv::Mat(intrinsics.dist_coeffs);

  // 3. Get the 2D "image points" from the observation.
  std::vector<cv::Point2d> image_points;
  for (size_t i = 0; i < observation.corners_pixels.size(); i += 2) {
    image_points.push_back(cv::Point2d(observation.corners_pixels[i],
                                       observation.corners_pixels[i + 1]));
  }

  if (image_points.size() != 4) {
    return std::nullopt;  // Must have 4 corners
  }

  // 4. Use solvePnPGeneric to get multiple possible solutions.
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> reprojection_errors;
  cv::solvePnPGeneric(object_points, image_points, camera_matrix, dist_coeffs,
                      rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE,
                      reprojection_errors);

  if (rvecs.empty()) {
    return std::nullopt;
  }

  // 5. Create the result struct.
  FiducialPoseObservation result;
  result.tag_id = observation.tag_id;

  // 6. Populate the result with the found poses.
  result.pose_0 = PoseUtils::rvec_tvec_to_pose3d(rvecs[0], tvecs[0]);
  result.error_0 = reprojection_errors[0];

  if (reprojection_errors.size() > 1) {
    result.pose_1 = PoseUtils::rvec_tvec_to_pose3d(rvecs[1], tvecs[1]);
    result.error_1 = reprojection_errors[1];
  }

  return result;
}
