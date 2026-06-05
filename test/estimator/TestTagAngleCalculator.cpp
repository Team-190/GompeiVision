#include <gtest/gtest.h>
#include "estimator/TagAngleCalculator.h"
#include "util/QueuedFiducialData.h"
#include <opencv2/core.hpp>
#include <cmath>

static cv::Mat getTestCameraMatrix() {
  return (cv::Mat_<double>(3, 3) << 500.0, 0.0, 320.0,
                                    0.0, 500.0, 240.0,
                                    0.0, 0.0, 1.0);
}

static cv::Mat getTestDistCoeffs() {
  return cv::Mat::zeros(5, 1, CV_64F);
}

TEST(TagAngleCalculatorTest, BasicCalculation) {
  FiducialImageObservation observation;
  observation.tag_ids = {7};
  // Centered tag, 1.0m away. Tag size = 0.1651m.
  observation.corners_pixels = {
    {361.275, 198.725, 278.725, 198.725, 278.725, 281.275, 361.275, 281.275}
  };
  observation.timestamp = 123456;

  AprilTagResult result;
  cv::Mat cameraMatrix = getTestCameraMatrix();
  cv::Mat distCoeffs = getTestDistCoeffs();
  double tag_size_m = 0.1651;

  TagAngleCalculator::calculate(observation, result, cameraMatrix, distCoeffs, tag_size_m);

  ASSERT_EQ(result.tag_angles.size(), 1u);
  EXPECT_EQ(result.tag_angles[0].tag_id, 7);
  // Distance should be close to 1.0m
  EXPECT_NEAR(result.tag_angles[0].distance, 1.0, 0.05);

  // Corner angles check: X angle for corner 0 (top-left) should be around atan((361.275-320)/500) = atan(0.08255)
  double expected_angle_x = std::atan(0.08255);
  double expected_angle_y = std::atan(-0.08255);
  EXPECT_NEAR(result.tag_angles[0].corners_angles[0], expected_angle_x, 1e-4);
  EXPECT_NEAR(result.tag_angles[0].corners_angles[1], expected_angle_y, 1e-4);
}

TEST(TagAngleCalculatorTest, FailureBranch) {
  // If fewer than 2 solutions or solver fails, should skip.
  FiducialImageObservation observation;
  observation.tag_ids = {99};
  observation.corners_pixels = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };

  AprilTagResult result;
  cv::Mat cameraMatrix = getTestCameraMatrix();
  cv::Mat distCoeffs = getTestDistCoeffs();
  TagAngleCalculator::calculate(observation, result, cameraMatrix, distCoeffs, 0.1651);

  EXPECT_EQ(result.tag_angles.size(), 0u);
}
