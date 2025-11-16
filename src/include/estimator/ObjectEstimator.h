#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Quaternion.h>

#include <map>
#include <opencv2/core/mat.hpp>
#include <vector>

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
  static void calculate(ObjDetectObservation& observation,
                        const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

 private:
  // Predefined 3D models for known object classes (in meters)
  inline static const std::map<int, std::vector<cv::Point3f>> object_models = {
      {0,  // Algae (Box based on 0.406400 sphere diameter)
       {
           cv::Point3f(-0.2032f, 0.2032f,
                       0.0f),  // Top-left (X: -0.2032, Y: 0.2032)
           cv::Point3f(0.2032f, 0.2032f,
                       0.0f),  // Top-right (X: 0.2032, Y: 0.2032)
           cv::Point3f(0.2032f, -0.2032f,
                       0.0f),  // Bottom-right (X: 0.2032, Y: -0.2032)
           cv::Point3f(-0.2032f, -0.2032f,
                       0.0f)  // Bottom-left (X: -0.2032, Y: -0.2032)
       }},
      {1,  // Coral (Width: 0.301625, Height: 0.114300)
       {
           cv::Point3f(-0.1508125f, 0.05715f,
                       0.0f),  // Top-left (X: -0.1508125, Y: 0.05715)
           cv::Point3f(0.1508125f, 0.05715f,
                       0.0f),  // Top-right (X: 0.1508125, Y: 0.05715)
           cv::Point3f(0.1508125f, -0.05715f,
                       0.0f),  // Bottom-right (X: 0.1508125, Y: -0.05715)
           cv::Point3f(-0.1508125f, -0.05715f,
                       0.0f)  // Bottom-left (X: -0.1508125, Y: -0.05715)
       }},
  };
};
