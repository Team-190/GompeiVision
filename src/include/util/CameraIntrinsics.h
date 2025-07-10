#pragma once

#include <vector>

/**
 * @struct CameraIntrinsics
 * @brief A data structure to hold the camera's calibration parameters.
 *
 * These values are read from a calibration file and are necessary for
 * accurate 3D pose estimation from a 2D image.
 */
struct CameraIntrinsics {
  // Camera Matrix Parameters
  double fx;  // Focal length in x
  double fy;  // Focal length in y
  double cx;  // Principal point x
  double cy;  // Principal point y

  // Distortion Coefficients
  // This vector holds the k1, k2, p1, p2, k3, etc. coefficients.
  std::vector<double> dist_coeffs;
};
