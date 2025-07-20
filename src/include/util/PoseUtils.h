#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>

#include <cmath>
#include <opencv2/opencv.hpp>

namespace PoseUtils {
/**
 * @brief Converts an OpenCV rotation vector and translation vector to an
 * frc::Pose3d, transforming from the OpenCV camera frame to the WPILib robot
 * frame.
 *
 * OpenCV Camera Frame: +Z forward, +X right, +Y down
 * WPILib Robot Frame:  +X forward, +Y left,  +Z up
 *
 * This results in the following mapping:
 *  - WPILib's X-axis becomes OpenCV's Z-axis.
 *  - WPILib's Y-axis becomes OpenCV's -X-axis.
 *  - WPILib's Z-axis becomes OpenCV's -Y-axis.
 *
 * @param rvec The rotation vector from solvePnP (in OpenCV's camera frame).
 * @param tvec The translation vector from solvePnP (in OpenCV's camera frame).
 * @return The calculated frc::Pose3d in WPILib's coordinate system.
 */
inline frc::Pose3d openCvPoseToWpilib(const cv::Mat& rvec,
                                      const cv::Mat& tvec) {
  // Ensure the input matrices are 3x1 and of type double (CV_64F).
  // This is the expected output format from cv::solvePnP.
  assert(rvec.rows == 3 && rvec.cols == 1 && rvec.type() == CV_64F);
  assert(tvec.rows == 3 && tvec.cols == 1 && tvec.type() == CV_64F);

  // 1. Create the Translation3d, converting from OpenCV to WPILib coords.
  //    WPILib (X, Y, Z) = OpenCV (Z, -X, -Y)
  //    Use .at<double>(row, 0) to access elements of the 3x1 cv::Mat.
  const frc::Translation3d frc_translation(
      units::meter_t{tvec.at<double>(2, 0)},
      units::meter_t{-tvec.at<double>(0, 0)},
      units::meter_t{-tvec.at<double>(1, 0)});

  // 2. Create the Rotation3d, converting from OpenCV to WPILib coords.
  //    WPILib

  cv::Mat rotationMatrix;
  cv::Rodrigues(rvec, rotationMatrix);

  // Fixed rotation matrix from OpenCV frame to WPILib frame
  // OpenCV: +X right, +Y down, +Z forward
  // WPILib: +X forward, +Y left, +Z up
  // Mapping:
  //   X_wpi = Z_cv
  //   Y_wpi = -X_cv
  //   Z_wpi = -Y_cv
  Eigen::Matrix3d cv_to_wpi;
  cv_to_wpi << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  // Convert OpenCV rotationMatrix (cv::Mat) to Eigen
  Eigen::Matrix3d cv_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      cv_rotation(row, col) = rotationMatrix.at<double>(row, col);
    }
  }

  // Apply coordinate frame rotation
  Eigen::Matrix3d wpi_rotation =
      cv_to_wpi * cv_rotation * cv_to_wpi.transpose();

  // Convert to frc::Rotation3d
  const frc::Rotation3d frc_rotation{
      units::radian_t{
          std::atan2(wpi_rotation(2, 1), wpi_rotation(2, 2))},  // roll
      units::radian_t{std::asin(-wpi_rotation(2, 0))},          // pitch
      units::radian_t{
          std::atan2(wpi_rotation(1, 0), wpi_rotation(0, 0))}  // yaw
  };

  return frc::Pose3d(frc_translation, frc_rotation);
}

inline cv::Point3f wpilibTranslationToOpenCV(
    const frc::Translation3d& translation) {
  return cv::Point3f(-translation.Y().value(), -translation.Z().value(),
                     translation.X().value());
}

inline bool isPoseZero(const frc::Pose3d& pose) {
  return pose.X() == 0_m && pose.Y() == 0_m && pose.Z() == 0_m;
}
}  // namespace PoseUtils