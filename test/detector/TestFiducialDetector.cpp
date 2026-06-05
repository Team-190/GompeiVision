#include <gtest/gtest.h>
#include "detector/FiducialDetector.h"
#include <opencv2/core.hpp>

TEST(FiducialDetectorTest, EmptyAndBlankFrames) {
  FiducialDetector detector;
  
  // Test empty frame
  QueuedFrame empty_frame;
  FiducialImageObservation obs1;
  detector.detect(empty_frame, obs1);
  EXPECT_TRUE(obs1.tag_ids.empty());

  // Test blank BGR frame (3 channels)
  QueuedFrame blank_bgr_frame;
  blank_bgr_frame.frame = cv::Mat::zeros(100, 100, CV_8UC3);
  blank_bgr_frame.timestamp = 1000;
  FiducialImageObservation obs2;
  detector.detect(blank_bgr_frame, obs2);
  EXPECT_TRUE(obs2.tag_ids.empty());

  // Test blank grayscale frame (1 channel)
  QueuedFrame blank_gray_frame;
  blank_gray_frame.frame = cv::Mat::zeros(100, 100, CV_8UC1);
  blank_gray_frame.timestamp = 2000;
  FiducialImageObservation obs3;
  detector.detect(blank_gray_frame, obs3);
  EXPECT_TRUE(obs3.tag_ids.empty());
}
