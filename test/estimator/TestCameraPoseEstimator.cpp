#include <gtest/gtest.h>
#include "estimator/CameraPoseEstimator.h"
#include "util/QueuedFiducialData.h"
#include "util/PoseUtils.h"
#include <opencv2/core.hpp>
#include <map>

static cv::Mat getTestCameraMatrix() {
  return (cv::Mat_<double>(3, 3) << 500.0, 0.0, 320.0,
                                    0.0, 500.0, 240.0,
                                    0.0, 0.0, 1.0);
}

static cv::Mat getTestDistCoeffs() {
  return cv::Mat::zeros(5, 1, CV_64F);
}

TEST(CameraPoseEstimatorTest, EmptyFieldLayout) {
  FiducialImageObservation observation;
  observation.tag_ids = {7};
  observation.corners_pixels = {
    {361.275, 198.725, 278.725, 198.725, 278.725, 281.275, 361.275, 281.275}
  };

  AprilTagResult result;
  std::map<int, frc::Pose3d> empty_layout;
  CameraPoseEstimator::estimatePose(observation, result, getTestCameraMatrix(), getTestDistCoeffs(), 0.1651, empty_layout);

  EXPECT_EQ(result.multi_tag_pose.tag_ids.size(), 0u);
}

TEST(CameraPoseEstimatorTest, SingleTag) {
  FiducialImageObservation observation;
  observation.tag_ids = {7};
  observation.corners_pixels = {
    {361.275, 198.725, 278.725, 198.725, 278.725, 281.275, 361.275, 281.275}
  };

  std::map<int, frc::Pose3d> field_layout;
  field_layout[7] = frc::Pose3d(1_m, 0_m, 0_m, frc::Rotation3d());

  AprilTagResult result;
  CameraPoseEstimator::estimatePose(observation, result, getTestCameraMatrix(), getTestDistCoeffs(), 0.1651, field_layout);

  ASSERT_EQ(result.multi_tag_pose.tag_ids.size(), 1u);
  EXPECT_EQ(result.multi_tag_pose.tag_ids[0], 7);
  
  EXPECT_NEAR(result.multi_tag_pose.pose_0.X().value(), 0.0, 0.05);
  EXPECT_NEAR(result.multi_tag_pose.pose_0.Y().value(), 0.0, 0.05);
  EXPECT_NEAR(result.multi_tag_pose.pose_0.Z().value(), 0.0, 0.05);
}

TEST(CameraPoseEstimatorTest, MultiTag) {
  FiducialImageObservation observation;
  observation.tag_ids = {7, 8};
  observation.corners_pixels = {
    {611.275, 198.725, 528.725, 198.725, 528.725, 281.275, 611.275, 281.275}, // Tag 7
    {111.275, 198.725, 28.725, 198.725, 28.725, 281.275, 111.275, 281.275}    // Tag 8
  };

  std::map<int, frc::Pose3d> field_layout;
  field_layout[7] = frc::Pose3d(1_m, 0_m, 0_m, frc::Rotation3d());
  field_layout[8] = frc::Pose3d(1_m, 1.0_m, 0_m, frc::Rotation3d());

  AprilTagResult result;
  CameraPoseEstimator::estimatePose(observation, result, getTestCameraMatrix(), getTestDistCoeffs(), 0.1651, field_layout);

  ASSERT_EQ(result.multi_tag_pose.tag_ids.size(), 2u);
  EXPECT_EQ(result.multi_tag_pose.tag_ids[0], 7);
  EXPECT_EQ(result.multi_tag_pose.tag_ids[1], 8);

  EXPECT_NEAR(result.multi_tag_pose.pose_0.X().value(), 0.0, 0.05);
  EXPECT_NEAR(result.multi_tag_pose.pose_0.Y().value(), 0.5, 0.05);
  EXPECT_NEAR(result.multi_tag_pose.pose_0.Z().value(), 0.0, 0.05);
}
