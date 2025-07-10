#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>

#include <opencv2/calib3d.hpp>

namespace PoseUtils {

/**
 * @brief Converts an OpenCV rotation vector and translation vector to an
 * frc::Pose3d. This version has no dependency on the Eigen library.
 * @param rvec The rotation vector from solvePnP.
 * @param tvec The translation vector from solvePnP.
 * @return The calculated frc::Pose3d.
 */
inline frc::Pose3d rvec_tvec_to_pose3d(const cv::Mat& rvec,
                                       const cv::Mat& tvec) {
  // 1. Convert the Rodrigues rotation vector into a 3x3 OpenCV rotation matrix.
  cv::Mat rotation_matrix_cv;
  cv::Rodrigues(rvec, rotation_matrix_cv);

  // 2. Create an FRC Quaternion directly from the elements of the OpenCV
  // rotation matrix. This bypasses the need for Eigen. The FRC Quaternion
  // constructor logic is based on the standard conversion from a rotation
  // matrix.
  const double w = 0.5 * std::sqrt(1.0 + rotation_matrix_cv.at<double>(0, 0) +
                                   rotation_matrix_cv.at<double>(1, 1) +
                                   rotation_matrix_cv.at<double>(2, 2));
  const double x = (rotation_matrix_cv.at<double>(2, 1) -
                    rotation_matrix_cv.at<double>(1, 2)) /
                   (4.0 * w);
  const double y = (rotation_matrix_cv.at<double>(0, 2) -
                    rotation_matrix_cv.at<double>(2, 0)) /
                   (4.0 * w);
  const double z = (rotation_matrix_cv.at<double>(1, 0) -
                    rotation_matrix_cv.at<double>(0, 1)) /
                   (4.0 * w);

  const frc::Quaternion frc_quat(w, x, y, z);

  // 3. Create the FRC Rotation3d object from the FRC Quaternion.
  const frc::Rotation3d frc_rotation(frc_quat);

  // 4. Create the frc::Translation3d from the translation vector.
  const frc::Translation3d frc_translation(units::meter_t(tvec.at<double>(0)),
                                           units::meter_t(tvec.at<double>(1)),
                                           units::meter_t(tvec.at<double>(2)));

  // 5. Combine them into the final frc::Pose3d.
  return frc::Pose3d(frc_translation, frc_rotation);
}

}  // namespace PoseUtils
