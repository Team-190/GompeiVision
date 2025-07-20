#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>

#include <opencv2/core/eigen.hpp>
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
  assert(rvec.rows == 3 && rvec.cols == 1 && rvec.type() == CV_64F);
  assert(tvec.rows == 3 && tvec.cols == 1 && tvec.type() == CV_64F);

  // 1. OpenCV Camera Frame translation to WPILib Field Frame
  frc::Translation3d frc_translation(
      units::meter_t{tvec.at<double>(2, 0)},    // Z → X
      units::meter_t{-tvec.at<double>(0, 0)},   // -X → Y
      units::meter_t{-tvec.at<double>(1, 0)});  // -Y → Z

  // 2. Convert rotation vector to rotation matrix
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);

  // 3. Apply camera-to-field rotation:
  cv::Mat cv_to_wpi = (cv::Mat_<double>(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0);
  cv::Mat R_wpi = cv_to_wpi * R_cv;

  // 4. Manually convert cv::Mat to Eigen::Matrix3d
  Eigen::Matrix3d eigen_R;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      eigen_R(row, col) = R_wpi.at<double>(row, col);
    }
  }

  frc::Rotation3d frc_rotation(eigen_R);

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