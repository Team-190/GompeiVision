#pragma once

#include <opencv2/core/mat.hpp>

#include "GamePiecePosEstimator.h"
#include "util/QueuedObjectData.h"

/**
 * @class ObjectEstimator
 * @brief Calculates the viewing angles to the corners of a detected object
 * and its distance from the camera.
 *
 * This class is analogous to the TagAngleCalculator and provides the necessary
 * data for robot alignment and targeting.
 */
class ObjectEstimator {
 public:
  /**
   * @brief Calculates angles and distances for all detected objects.
   *
   * For each object in the observation, this function:
   * 1. Undistorts the 2D corner points of the bounding box.
   * 2. Calculates the horizontal and vertical viewing angle to each corner.
   * 3. Uses the known physical width of the object to estimate distance.
   * 4. Populates the `corner_angles` vector in the provided observation.
   *
   * @param observation A reference to the object observation to be populated.
   * @param cameraMatrix The camera's intrinsic matrix.
   * @param distCoeffs The camera's distortion coefficients.
   */
  void calculate(ObjDetectObservation& observation, const cv::Mat& cameraMatrix,
                 const cv::Mat& distCoeffs) const;

  static ObjectEstimator& GetInstance(float width, float height);

 private:
  ObjectEstimator(float width, float height);  // private

  static ObjectEstimator* instance;

  GamePiecePosEstimator posEstimator;
};
