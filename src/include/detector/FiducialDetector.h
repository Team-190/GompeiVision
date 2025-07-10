#pragma once

#include <apriltag/apriltag.h>

#include <opencv2/core/mat.hpp>
#include <vector>

#include "util/QueuedFiducialData.h"

/**
 * @class FiducialDetector
 * @brief Finds the 2D locations of AprilTags in an image.
 *
 * This class is responsible for the first stage of the pipeline: running the
 * core AprilTag algorithm to detect tags and extract their corner points in
 * pixel coordinates. It does not perform any 3D calculations.
 */
class FiducialDetector {
 public:
  FiducialDetector();
  ~FiducialDetector();

  // Disable copy and move semantics
  FiducialDetector(const FiducialDetector&) = delete;
  FiducialDetector& operator=(const FiducialDetector&) = delete;

  /**
   * @brief Detects all AprilTags in a given frame.
   * @param frame The input image (will be converted to grayscale if necessary).
   * @return A vector of FiducialImageObservation structs, one for each tag
   * found.
   */
  std::vector<FiducialImageObservation> detect(const cv::Mat& frame) const;

 private:
  apriltag_detector_t* tagDetector = nullptr;
  apriltag_family_t* tagFamily = nullptr;
};
