#include "estimator/ObjectEstimator.h"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>

void ObjectEstimator::calculate(ObjDetectObservation& observation,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distCoeffs) {
  std::cout << "[ObjectEstimator DEBUG] Starting calculation for observation" << std::endl;
  std::cout << "[ObjectEstimator DEBUG] Corner pixels size: " << observation.corner_pixels.size() << std::endl;
  
  if (observation.corner_pixels.size() != 8) {
    std::cout << "[ObjectEstimator DEBUG] Invalid corner pixels size, returning early" << std::endl;
    return; // Must be a 4-corner bounding box
  }

  // 1. Get raw corner pixels for this object
  std::cout << "[ObjectEstimator DEBUG] Converting corner pixels to cv::Point2f" << std::endl;
  std::vector<cv::Point2f> image_points;
  image_points.reserve(4);
  for (size_t j = 0; j < observation.corner_pixels.size(); j += 2) {
    cv::Point2f point(observation.corner_pixels[j], observation.corner_pixels[j + 1]);
    std::cout << "[ObjectEstimator DEBUG] Corner " << (j/2) << ": (" << point.x << ", " << point.y << ")" << std::endl;
    image_points.emplace_back(observation.corner_pixels[j],
                              observation.corner_pixels[j + 1]);
  }
  
  std::cout << "[ObjectEstimator DEBUG] Camera matrix size: " << cameraMatrix.size() << std::endl;
  std::cout << "[ObjectEstimator DEBUG] Distortion coeffs size: " << distCoeffs.size() << std::endl;
  
  // 2. Undistort corner points for accurate angle calculation
  std::cout << "[ObjectEstimator DEBUG] Starting undistortion process" << std::endl;
  std::vector<cv::Point2f> undistorted_points;
  cv::undistortPoints(image_points, undistorted_points, cameraMatrix,
                      distCoeffs, cv::noArray(), cameraMatrix);
  std::cout << "[ObjectEstimator DEBUG] Undistortion completed, got " << undistorted_points.size() << " points" << std::endl;

  // 3. Calculate viewing angles from undistorted points
  std::cout << "[ObjectEstimator DEBUG] Calculating camera matrix inverse" << std::endl;
  const cv::Mat invCameraMatrix = cameraMatrix.inv();
  observation.corner_angles.reserve(8);

  std::cout << "[ObjectEstimator DEBUG] Processing undistorted points for angle calculation" << std::endl;
  for (size_t i = 0; i < undistorted_points.size(); ++i) {
    const auto& corner = undistorted_points[i];
    std::cout << "[ObjectEstimator DEBUG] Processing corner " << i << ": (" << corner.x << ", " << corner.y << ")" << std::endl;
    
    cv::Mat p(3, 1, CV_64F);
    p.at<double>(0, 0) = corner.x;
    p.at<double>(1, 0) = corner.y;
    p.at<double>(2, 0) = 1.0;

    cv::Mat vec = invCameraMatrix * p;

    double angle_x = std::atan(vec.at<double>(0, 0)); // Angle X
    double angle_y = std::atan(vec.at<double>(1, 0)); // Angle Y
    
    std::cout << "[ObjectEstimator DEBUG] Corner " << i << " angles: X=" << angle_x << ", Y=" << angle_y << std::endl;
    
    observation.corner_angles.push_back(angle_x);
    observation.corner_angles.push_back(angle_y);
  }

  std::cout << "[ObjectEstimator DEBUG] Final corner_angles size: " << observation.corner_angles.size() << std::endl;
  std::cout << "[ObjectEstimator DEBUG] Calculation completed successfully" << std::endl;

  // NOTE: Distance estimation is more complex for generic objects than for
  // AprilTags. A simple method is to use the known width of the object and
  // the camera's focal length. This is less accurate than solvePnP but is a
  // common approach. You would add that logic here if needed.
}
