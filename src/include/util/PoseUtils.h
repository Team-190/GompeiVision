#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>

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

  // Convert OpenCV rvec to 3x3 rotation matrix
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);

  Eigen::Matrix3d eigen_R_cv;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) eigen_R_cv(i, j) = R_cv.at<double>(i, j);

  // OpenCV to WPILib coordinate frame conversion matrix
  Eigen::Matrix3d cv_to_wpi;
  cv_to_wpi << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  // Rotate into WPILib frame
  const Eigen::Matrix3d eigen_R_wpi =
      cv_to_wpi * eigen_R_cv * cv_to_wpi.transpose();

  // Translation conversion
  const frc::Translation3d frc_translation(
      units::meter_t{tvec.at<double>(2, 0)},
      units::meter_t{-tvec.at<double>(0, 0)},
      units::meter_t{-tvec.at<double>(1, 0)});

  const frc::Rotation3d frc_rotation(eigen_R_wpi);

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