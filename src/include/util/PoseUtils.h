#pragma once

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/units/angle.hpp>

#include <cassert>
#include <opencv2/opencv.hpp>

namespace PoseUtils {
/**
 * @brief Converts an OpenCV rotation vector and translation vector to an
 * wpi::math::Pose3d, transforming from the OpenCV camera frame to the WPILib robot
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
 * @return The calculated wpi::math::Pose3d in WPILib's coordinate system.
 */
inline wpi::math::Pose3d openCvPoseToWpilib(const cv::Mat& rvec,
                                      const cv::Mat& tvec) {
  assert(rvec.rows == 3 && rvec.cols == 1 && rvec.type() == CV_64F);
  assert(tvec.rows == 3 && tvec.cols == 1 && tvec.type() == CV_64F);

  // 1. Translation conversion (OpenCV to WPILib)
  const wpi::math::Translation3d frc_translation(
      wpi::units::meter_t{tvec.at<double>(2, 0)},    // Z -> X
      wpi::units::meter_t{-tvec.at<double>(0, 0)},   // -X -> Y
      wpi::units::meter_t{-tvec.at<double>(1, 0)});  // -Y -> Z

  // 2. Rotation conversion (Axis-angle)
  const Eigen::Vector3d axis(rvec.at<double>(2, 0),    // Z -> X
                             -rvec.at<double>(0, 0),   // -X -> Y
                             -rvec.at<double>(1, 0));  // -Y -> Z

  const double angle = axis.norm();

  const wpi::math::Rotation3d frc_rotation(axis, wpi::units::radian_t(angle));

  return wpi::math::Pose3d(frc_translation, frc_rotation);
}

inline cv::Point3f wpilibTranslationToOpenCV(
    const wpi::math::Translation3d& translation) {
  return cv::Point3f(-translation.Y().value(), -translation.Z().value(),
                     translation.X().value());
}

inline bool isPoseZero(const wpi::math::Pose3d& pose) {
  const wpi::units::meter_t zero{0};
  return pose.X() == zero && pose.Y() == zero && pose.Z() == zero;
}
}  // namespace PoseUtils