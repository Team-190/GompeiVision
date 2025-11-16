#include "estimator/ObjectEstimator.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <sstream>
#include <vector>

void ObjectEstimator::calculate(ObjDetectObservation& observation,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distCoeffs) {
  if (observation.corner_pixels.size() != 8) {
    return;  // Must be a 4-corner bounding box
  }

  // 1. Get raw corner pixels for this object
  std::vector<cv::Point2f> image_points;
  image_points.reserve(4);
  for (size_t j = 0; j < observation.corner_pixels.size(); j += 2) {
    image_points.emplace_back(observation.corner_pixels[j],
                              observation.corner_pixels[j + 1]);
  }

  // 2. Undistort corner points for accurate angle calculation
  std::vector<cv::Point2f> undistorted_points;
  cv::undistortPoints(image_points, undistorted_points, cameraMatrix,
                      distCoeffs, cv::noArray(), cameraMatrix);

  // 3. Calculate viewing angles from undistorted points
  const cv::Mat invCameraMatrix = cameraMatrix.inv();

  for (const auto& corner : undistorted_points) {
    cv::Mat p(3, 1, CV_64F);
    p.at<double>(0, 0) = corner.x;
    p.at<double>(1, 0) = corner.y;
    p.at<double>(2, 0) = 1.0;

    cv::Mat vec = invCameraMatrix * p;
  }
  // TODO: Use yoavrozov.github.io/FRC-Game-Piece-Pos-Estimation

  // 8. Store the final pose and distance in the observation struct
  observation.pose = frc::Pose3d();
}