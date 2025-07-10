#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace PoseUtils {

/**
 * @brief Converts an OpenCV rotation vector and translation vector to an
 * frc::Pose3d.
 * @param rvec The rotation vector from solvePnP.
 * @param tvec The translation vector from solvePnP.
 * @return The calculated frc::Pose3d.
 */
inline frc::Pose3d rvec_tvec_to_pose3d(const cv::Mat& rvec,
                                       const cv::Mat& tvec) {
  cv::Mat rotation_matrix_cv;
  cv::Rodrigues(rvec, rotation_matrix_cv);

  // Convert the OpenCV matrix to an Eigen matrix, which is what the
  // frc::Rotation3d constructor expects.
  Eigen::Matrix3d rotation_matrix_eigen;
  cv::cv2eigen(rotation_matrix_cv, rotation_matrix_eigen);

  // Create the FRC object from the Eigen matrix.
  const frc::Rotation3d frc_rotation(rotation_matrix_eigen);

  const frc::Translation3d frc_translation(units::meter_t(tvec.at<double>(0)),
                                           units::meter_t(tvec.at<double>(1)),
                                           units::meter_t(tvec.at<double>(2)));

  return frc::Pose3d(frc_translation, frc_rotation);
}

}  // namespace PoseUtils
