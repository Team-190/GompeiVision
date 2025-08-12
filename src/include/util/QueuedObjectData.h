#pragma once

#include <chrono>
#include <string>
#include <vector>

/**
 * @struct ObjDetectObservation
 * @brief Represents a single detected object in a frame.
 * Corresponds to the Python 'ObjDetectObservation'.
 */
struct ObjDetectObservation {
  int obj_class;
  double confidence;
  std::vector<double> corner_angles;
  std::vector<double> corner_pixels;
};

// --- Main Result Data Packets ---
// These are the structs that will be placed into the ThreadSafeQueue
// for the NetworkTables thread to consume.

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
