#pragma once

#include <chrono>
#include <opencv2/core/mat.hpp>
#include <string>

/**
 * @struct QueuedFrame
 * @brief A simple data structure to bundle a captured video frame with its
 * metadata.
 *
 * This struct is used to pass all necessary information about a frame from the
 * capture thread to the processing threads in a single, neat package.
 */
struct QueuedFrame {
  cv::Mat frame;
  std::string cameraRole;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};