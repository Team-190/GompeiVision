#include "estimator/SingleTagPoseEstimator.h"

#include <frc/geometry/Pose3d.h>
#include <util/PoseUtils.h>

#include <opencv2/calib3d.hpp>
#include <vector>

#include "util/QueuedFiducialData.h"

void SingleTagPoseEstimator::estimatePose(
    const FiducialImageObservation& observation, AprilTagResult& result,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const double tag_size_m) {
  const std::vector<cv::Point3d> standard_object_points = {
      {-tag_size_m / 2.0, tag_size_m / 2.0, 0.0f},  // Top-left
      {tag_size_m / 2.0, tag_size_m / 2.0, 0.0f},   // Top-right
      {tag_size_m / 2.0, -tag_size_m / 2.0, 0.0f},  // Bottom-right
      {-tag_size_m / 2.0, -tag_size_m / 2.0, 0.0f}  // Bottom-left
  };

  for (size_t i = 0; i < observation.tag_ids.size(); ++i) {
    // Convert the flat corner vector into a vector of cv::Point2f
    std::vector<cv::Point2f> image_points;
    image_points.reserve(4);
    for (size_t j = 0; j < observation.corners_pixels[i].size(); j += 2) {
      image_points.emplace_back(observation.corners_pixels[i][j],
                                observation.corners_pixels[i][j + 1]);
    }

    // Use IPPE to get the two ambiguous poses for the tag
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errors;
    cv::solvePnPGeneric(standard_object_points, image_points, cameraMatrix,
                        distCoeffs, rvecs, tvecs, false,
                        cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(),
                        errors);

    // A valid solution must have two poses and corresponding errors.
    if (rvecs.size() == 2 && tvecs.size() == 2 && errors.size() == 2) {
      FiducialPoseObservation pose;
      pose.tag_id = observation.tag_ids[i];
      pose.error_0 = errors[0];
      pose.pose_0 = PoseUtils::openCvPoseToWpilib(tvecs[0], rvecs[0]);
      pose.error_1 = errors[1];
      pose.pose_1 = PoseUtils::openCvPoseToWpilib(tvecs[1], rvecs[1]);

      units::meter_t distance;

      if (pose.error_1.has_value() && pose.pose_1.has_value() &&
          pose.error_1 < pose.error_0) {
        distance = pose.pose_1->Translation().Norm();
      } else {
        distance = pose.pose_0.Translation().Norm();
      }

      pose.distance = distance;

      result.single_tag_poses.push_back(pose);
    }
  }
}
