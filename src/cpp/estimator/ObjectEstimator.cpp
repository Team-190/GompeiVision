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
  observation.corner_angles.reserve(8);

  for (const auto& corner : undistorted_points) {
    cv::Mat p(3, 1, CV_64F);
    p.at<double>(0, 0) = corner.x;
    p.at<double>(1, 0) = corner.y;
    p.at<double>(2, 0) = 1.0;

    cv::Mat vec = invCameraMatrix * p;

    observation.corner_angles.push_back(
        std::atan(vec.at<double>(0, 0)));  // Angle X
    observation.corner_angles.push_back(
        std::atan(vec.at<double>(1, 0)));  // Angle Y
  }

  // 4. Look up the 3D model points for the detected object's class ID
  if (object_models.find(observation.obj_class) == object_models.end()) {
    return;  // No 3D model available for this object, can't calculate pose
  }
  const std::vector<cv::Point3f>& model_points_3d =
      object_models.at(observation.obj_class);

  // 5. Ensure we have a matching number of 2D and 3D points (at least 4)
  if (model_points_3d.size() != image_points.size() ||
      model_points_3d.size() < 4) {
    std::cerr << "ERROR: Mismatch between 3D model points and 2D image points "
                 "for class "
              << observation.obj_class << std::endl;
    return;
  }

  // 6. Use solvePnP to find the object's rotation and translation
  // We use the original 'image_points' as solvePnP takes distCoeffs directly.
  cv::Mat rvec, tvec;
  bool success = cv::solvePnP(model_points_3d, image_points, cameraMatrix,
                              distCoeffs, rvec, tvec);

  if (!success) {
    return;  // Pose estimation failed
  }

  // Convert rotation vector to rotation matrix
  cv::Matx33d rotation_matrix;
  cv::Rodrigues(rvec, rotation_matrix);

  // Convert rotation matrix to quaternion
  // Using the algorithm from
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
  double trace =
      rotation_matrix(0, 0) + rotation_matrix(1, 1) + rotation_matrix(2, 2);
  double qw, qx, qy, qz;

  if (trace > 0) {
    double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (rotation_matrix(2, 1) - rotation_matrix(1, 2)) * s;
    qy = (rotation_matrix(0, 2) - rotation_matrix(2, 0)) * s;
    qz = (rotation_matrix(1, 0) - rotation_matrix(0, 1)) * s;
  } else if (rotation_matrix(0, 0) > rotation_matrix(1, 1) &&
             rotation_matrix(0, 0) > rotation_matrix(2, 2)) {
    double s = 2.0 * std::sqrt(1.0 + rotation_matrix(0, 0) -
                               rotation_matrix(1, 1) - rotation_matrix(2, 2));
    qw = (rotation_matrix(2, 1) - rotation_matrix(1, 2)) / s;
    qx = 0.25 * s;
    qy = (rotation_matrix(0, 1) + rotation_matrix(1, 0)) / s;
    qz = (rotation_matrix(0, 2) + rotation_matrix(2, 0)) / s;
  } else if (rotation_matrix(1, 1) > rotation_matrix(2, 2)) {
    double s = 2.0 * std::sqrt(1.0 + rotation_matrix(1, 1) -
                               rotation_matrix(0, 0) - rotation_matrix(2, 2));
    qw = (rotation_matrix(0, 2) - rotation_matrix(2, 0)) / s;
    qx = (rotation_matrix(0, 1) + rotation_matrix(1, 0)) / s;
    qy = 0.25 * s;
    qz = (rotation_matrix(1, 2) + rotation_matrix(2, 1)) / s;
  } else {
    double s = 2.0 * std::sqrt(1.0 + rotation_matrix(2, 2) -
                               rotation_matrix(0, 0) - rotation_matrix(1, 1));
    qw = (rotation_matrix(1, 0) - rotation_matrix(0, 1)) / s;
    qx = (rotation_matrix(0, 2) + rotation_matrix(2, 0)) / s;
    qy = (rotation_matrix(1, 2) + rotation_matrix(2, 1)) / s;
    qz = 0.25 * s;
  }

  // Create Rotation3d from quaternion (w, x, y, z)
  frc::Rotation3d frc_rotation(frc::Quaternion(qw, qx, qy, qz));

  // Apply the coordinate system change from OpenCV to FRC
  frc::Translation3d frc_translation(
      units::meter_t{tvec.at<double>(2)},   // FRC X is OpenCV Z
      units::meter_t{-tvec.at<double>(0)},  // FRC Y is OpenCV -X
      units::meter_t{-tvec.at<double>(1)}   // FRC Z is OpenCV -Y
  );

  // 8. Store the final pose and distance in the observation struct
  observation.pose = frc::Pose3d(frc_translation, frc_rotation);
  observation.distance = frc_translation.Norm().value();
}