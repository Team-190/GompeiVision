#pragma once

#include <opencv2/core/types.hpp>
#include <vector>

#include "util/QueuedFrame.h"

/**
 * @struct AnnotationData
 * @brief Holds all the necessary data for the annotation thread to draw on a
 * frame.
 */
struct AnnotationData {
  QueuedFrame q_frame;
  std::vector<cv::Rect> boxes;
  std::vector<int> class_ids;
  std::vector<float> confidences;
};