#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>

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

  // 2. The rvec is a rotation vector where the direction is the axis of
  //    rotation and the magnitude is the angle in radians. cv::norm works
  //    correctly on a cv::Mat vector.
  const double angle_rad = cv::norm(rvec);

  // If the angle is negligible, the rotation is identity. This also avoids
  // division by zero in the calculations below.
  if (std::abs(angle_rad) < 1e-9) {
    return frc::Pose3d(frc_translation, frc::Rotation3d{});
  }

  // 3. The axis of rotation in OpenCV's frame is rvec / |rvec|.
  //    We transform this unit axis to WPILib's frame using the same
  //    (Z, -X, -Y) mapping.
  const double axis_x_wpilib = rvec.at<double>(2, 0) / angle_rad;
  const double axis_y_wpilib = -rvec.at<double>(0, 0) / angle_rad;
  const double axis_z_wpilib = -rvec.at<double>(1, 0) / angle_rad;

  // 4. Create a Quaternion from the WPILib axis and angle.
  //    The formula for a quaternion from an axis-angle representation is:
  //    q = (cos(angle/2), sin(angle/2)*axis_x, sin(angle/2)*axis_y,
  //    sin(angle/2)*axis_z)
  const double half_angle = angle_rad / 2.0;
  const double s = std::sin(half_angle);

  const frc::Quaternion frc_quat(std::cos(half_angle),  // w
                                 s * axis_x_wpilib,     // x
                                 s * axis_y_wpilib,     // y
                                 s * axis_z_wpilib      // z
  );

  // 5. Create the Rotation3d from the Quaternion.
  const frc::Rotation3d frc_rotation(frc_quat);

  // 6. Combine the translation and rotation into the final frc::Pose3d.
  return frc::Pose3d(frc_translation, frc_rotation);
}

inline cv::Point3f wpilibTranslationToOpenCV(
    const frc::Translation3d& translation) {
  return cv::Point3f(translation.Y().value(), translation.Z().value(),
                     translation.X().value());
}
}  // namespace PoseUtils