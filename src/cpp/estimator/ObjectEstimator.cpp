#include "estimator/ObjectEstimator.h"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <vector>

void ObjectEstimator::calculate(ObjDetectObservation& observation,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distCoeffs) {
  if (observation.corner_pixels.size() != 8) {
    return; // Must be a 4-corner bounding box
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
  observation.corner_angles.reserve(8);

  for (const auto& corner : undistorted_points) {
    cv::Mat p(3, 1, CV_64F);
    p.at<double>(0, 0) = corner.x;
    p.at<double>(1, 0) = corner.y;
    p.at<double>(2, 0) = 1.0;

    cv::Mat vec = invCameraMatrix * p;

    observation.corner_angles.push_back(std::atan(vec.at<double>(0, 0))); // Angle X
    observation.corner_angles.push_back(std::atan(vec.at<double>(1, 0))); // Angle Y
  }

  // NOTE: Distance estimation is more complex for generic objects than for
  // AprilTags. A simple method is to use the known width of the object and
  // the camera's focal length. This is less accurate than solvePnP but is a
  // common approach. You would add that logic here if needed.
}
