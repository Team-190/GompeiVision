#include <gtest/gtest.h>
#include "estimator/SingleTagPoseEstimator.h"
#include "util/QueuedFiducialData.h"
#include "util/PoseUtils.h"
#include <opencv2/core.hpp>

static cv::Mat getTestCameraMatrix() {
  return (cv::Mat_<double>(3, 3) << 500.0, 0.0, 320.0,
                                    0.0, 500.0, 240.0,
                                    0.0, 0.0, 1.0);
}

static cv::Mat getTestDistCoeffs() {
  return cv::Mat::zeros(5, 1, CV_64F);
}

TEST(SingleTagPoseEstimatorTest, BasicPoseEstimation) {
  FiducialImageObservation observation;
  observation.tag_ids = {7};
  observation.corners_pixels = {
    {361.275, 198.725, 278.725, 198.725, 278.725, 281.275, 361.275, 281.275}
  };

  AprilTagResult result;
  cv::Mat cameraMatrix = getTestCameraMatrix();
  cv::Mat distCoeffs = getTestDistCoeffs();
  double tag_size_m = 0.1651;

  SingleTagPoseEstimator::estimatePose(observation, result, cameraMatrix, distCoeffs, tag_size_m);

  ASSERT_EQ(result.single_tag_poses.size(), 1u);
  EXPECT_EQ(result.single_tag_poses[0].tag_id, 7);
  EXPECT_NEAR(result.single_tag_poses[0].distance.value(), 1.0, 0.05);
  
  // Z translation in OpenCV camera frame maps to X in WPILib robot frame.
  EXPECT_NEAR(result.single_tag_poses[0].pose_0.X().value(), 1.0, 0.05);
}

TEST(SingleTagPoseEstimatorTest, FailureBranch) {
  FiducialImageObservation observation;
  observation.tag_ids = {99};
  observation.corners_pixels = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };

  AprilTagResult result;
  cv::Mat cameraMatrix = getTestCameraMatrix();
  cv::Mat distCoeffs = getTestDistCoeffs();
  SingleTagPoseEstimator::estimatePose(observation, result, cameraMatrix, distCoeffs, 0.1651);

  EXPECT_EQ(result.single_tag_poses.size(), 0u);
}
