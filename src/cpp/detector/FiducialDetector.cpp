#include "detector/FiducialDetector.h"

#include <apriltag/tag36h11.h>  // Using the standard 36h11 tag family

#include <iostream>
#include <opencv2/imgproc.hpp>  // For cvtColor

#include "util/QueuedFrame.h"

FiducialDetector::FiducialDetector() {
  std::cout << "[FiducialDetector] Initializing..." << std::endl;
  // Create the tag family (e.g., tag36h11)
  tagFamily = tag36h11_create();
  if (!tagFamily) {
    std::cerr << "[FiducialDetector] ERROR: Failed to create tag family."
              << std::endl;
    return;
  }

  // Create the AprilTag detector instance
  tagDetector = apriltag_detector_create();
  if (!tagDetector) {
    std::cerr << "[FiducialDetector] ERROR: Failed to create tag detector."
              << std::endl;
    tag36h11_destroy(tagFamily);
    tagFamily = nullptr;
    return;
  }

  // Add the tag family to the detector
  apriltag_detector_add_family(tagDetector, tagFamily);

  // Set detector parameters (can be tuned)
  tagDetector->quad_decimate = 1.0;
  tagDetector->nthreads = 1;
  tagDetector->debug = false;
  tagDetector->refine_edges = true;
}

FiducialDetector::~FiducialDetector() {
  std::cout << "[FiducialDetector] Shutting down..." << std::endl;
  // Clean up the detector and the tag family to prevent memory leaks.
  if (tagDetector) {
    apriltag_detector_destroy(tagDetector);
  }
  if (tagFamily) {
    tag36h11_destroy(tagFamily);
  }
}

void FiducialDetector::detect(const QueuedFrame& frame,
                              FiducialImageObservation& observation) const {
  if (!tagDetector || frame.frame.empty()) {
    return;
  }

  // 1. Convert the input OpenCV Mat to the grayscale image_u8_t format
  //    that the AprilTag library requires.
  cv::Mat gray_frame;
  if (frame.frame.channels() == 3) {
    cv::cvtColor(frame.frame, gray_frame, cv::COLOR_BGR2GRAY);
  } else {
    gray_frame = frame.frame;
  }

  image_u8_t img_header = {.width = gray_frame.cols,
                           .height = gray_frame.rows,
                           .stride = gray_frame.cols,
                           .buf = gray_frame.data};

  // 2. Run the detector and get a list of detections.
  zarray_t* detections = apriltag_detector_detect(tagDetector, &img_header);

  // 3. Loop through each detection and package it into our struct.
  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);

    observation.tag_ids.push_back(det->id);

    // Add the corner points to the flat vector
    std::vector<double> flat_corners;
    flat_corners.reserve(8);
    flat_corners.push_back(det->p[0][0]);
    flat_corners.push_back(det->p[0][1]);
    flat_corners.push_back(det->p[1][0]);
    flat_corners.push_back(det->p[1][1]);
    flat_corners.push_back(det->p[2][0]);
    flat_corners.push_back(det->p[2][1]);
    flat_corners.push_back(det->p[3][0]);
    flat_corners.push_back(det->p[3][1]);

    observation.corners_pixels.push_back(flat_corners);
  }

  // 4. Clean up the memory used by the detections array.
  apriltag_detections_destroy(detections);
}
