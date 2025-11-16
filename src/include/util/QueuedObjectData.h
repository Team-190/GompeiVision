#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <frc/geometry/Pose3d.h>

/**
 * @struct ObjDetectObservation
 * @brief Represents a single detected object in a frame.
 */
struct ObjDetectObservation {
  int obj_class;
  float confidence;
  std::vector<double> corner_pixels;  // tl, tr, br, bl (x, y)
  frc::Pose3d pose;

  // Default constructor
  ObjDetectObservation() = default;

  // MODIFIED: Constructor for efficient emplace_back in the detector
  ObjDetectObservation(int cls, float conf, double tl_x, double tl_y,
                       double tr_x, double tr_y, double br_x, double br_y,
                       double bl_x, double bl_y, frc::Pose3d pose)
      : obj_class(cls), confidence(conf), pose(pose)
  {
      // The vector is now initialized inside the constructor body
      corner_pixels = {tl_x, tl_y, tr_x, tr_y, br_x, br_y, bl_x, bl_y};
  }
};

/**
 * @struct ObjectDetectResult
 * @brief A complete data packet containing all results from the object
 * detection pipeline for a single captured frame.
 */
struct ObjectDetectResult {
  std::string camera_role;  // To know which NT topic to publish to
  std::chrono::time_point<std::chrono::system_clock> timestamp;
  int fps;

  // A list of all objects found in the frame.
  std::vector<ObjDetectObservation> observations;
};
