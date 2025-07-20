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

  const frc::Translation3d frc_translation(
      units::meter_t{tvec.at<double>(2, 0)},
      units::meter_t{-tvec.at<double>(0, 0)},
      units::meter_t{-tvec.at<double>(1, 0)});

  const double angle_rad = cv::norm(rvec);
  if (std::abs(angle_rad) < 1e-9) {
    return frc::Pose3d(frc_translation, frc::Rotation3d{});
  }

  const double axis_x_wpilib = rvec.at<double>(2, 0) / angle_rad;
  const double axis_y_wpilib = -rvec.at<double>(0, 0) / angle_rad;
  const double axis_z_wpilib = -rvec.at<double>(1, 0) / angle_rad;

  const double half_angle = angle_rad / 2.0;
  const double s = std::sin(half_angle);

  const frc::Quaternion frc_quat(std::cos(half_angle),  // w
                                 s * axis_x_wpilib,     // x
                                 s * axis_y_wpilib,     // y
                                 s * axis_z_wpilib      // z
  );

  const frc::Rotation3d frc_rotation(frc_quat);

  // Apply fixed camera -> WPILib transform (Pitch +90, Yaw 180)
  constexpr frc::Rotation3d cv_to_wpilib_rot{
      units::radian_t{0}, units::radian_t{M_PI / 2.0}, units::radian_t{M_PI}};

  const frc::Rotation3d corrected_rotation =
      frc_rotation.RotateBy(cv_to_wpilib_rot);

  return frc::Pose3d(frc_translation, corrected_rotation);
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