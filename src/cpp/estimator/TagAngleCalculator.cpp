#include "estimator/TagAngleCalculator.h"

#include <frc/geometry/Pose3d.h>

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <vector>

#include "util/PoseUtils.h"
#include "util/QueuedFiducialData.h"

void TagAngleCalculator::calculate(const FiducialImageObservation& observation,
                                   AprilTagResult& result,
                                   const cv::Mat& cameraMatrix,
                                   const cv::Mat& distCoeffs,
                                   double tag_size_m) {
  // Define the standard object points for a single tag.
  // This is used for pose estimation to find the distance.
  const std::vector<cv::Point3d> standard_object_points = {
      {-tag_size_m / 2.0, tag_size_m / 2.0, 0.0},  // Top-left
      {tag_size_m / 2.0, tag_size_m / 2.0, 0.0},   // Top-right
      {tag_size_m / 2.0, -tag_size_m / 2.0, 0.0},  // Bottom-right
      {-tag_size_m / 2.0, -tag_size_m / 2.0, 0.0}  // Bottom-left
  };

  const cv::Mat invCameraMatrix = cameraMatrix.inv();
  result.tag_angles.reserve(observation.tag_ids.size());

  for (size_t i = 0; i < observation.tag_ids.size(); ++i) {
    // 1. Get raw corner pixels for this tag
    std::vector<cv::Point2f> image_points;
    image_points.reserve(4);
    for (size_t j = 0; j < observation.corners_pixels[i].size(); j += 2) {
      image_points.emplace_back(observation.corners_pixels[i][j],
                                observation.corners_pixels[i][j + 1]);
    }

    // 2. Calculate distance via single-tag pose estimation (IPPE)
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errors;
    cv::solvePnPGeneric(standard_object_points, image_points, cameraMatrix,
                        distCoeffs, rvecs, tvecs, false,
                        cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(),
                        errors);

    double distance = 0.0;
    // A valid IPPE solution must have two poses and corresponding errors.
    if (rvecs.size() == 2 && tvecs.size() == 2 && errors.size() == 2) {
      // NOTE: Calling openCvPoseToWpilib with (rvec, tvec) as per the
      // function signature, not (tvec, rvec) as seen elsewhere.
      auto pose0 = PoseUtils::openCvPoseToWpilib(rvecs[0], tvecs[0]);
      auto pose1 = PoseUtils::openCvPoseToWpilib(rvecs[1], tvecs[1]);

      distance = (errors[0] <= errors[1]) ? pose0.Translation().Norm().value()
                                          : pose1.Translation().Norm().value();
    } else {
      // If pose estimation fails, we cannot calculate a distance. Skip tag.
      continue;
    }

    // 3. Undistort corner points for angle calculation
    std::vector<cv::Point2f> undistorted_points;
    cv::undistortPoints(image_points, undistorted_points, cameraMatrix,
                        distCoeffs, cv::noArray(), cameraMatrix);

    // 4. Calculate viewing angles from undistorted points
    std::vector<double> corner_angles;
    corner_angles.reserve(8);
    for (const auto& corner : undistorted_points) {
      cv::Mat p(3, 1, CV_64F);
      p.at<double>(0, 0) = corner.x;
      p.at<double>(1, 0) = corner.y;
      p.at<double>(2, 0) = 1.0;

      cv::Mat vec = invCameraMatrix * p;

      corner_angles.push_back(std::atan(vec.at<double>(0, 0)));  // Angle X
      corner_angles.push_back(std::atan(vec.at<double>(1, 0)));  // Angle Y
    }

    // 5. Store the final result for this tag
    const auto tagAngleObservation = TagAngleObservation{
        observation.tag_ids[i], std::move(corner_angles), distance};

    result.tag_angles.emplace_back(tagAngleObservation);
  }
}