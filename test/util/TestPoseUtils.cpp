#include <gtest/gtest.h>
#include "util/PoseUtils.h"

TEST(PoseUtilsTest, IsPoseZero) {
  // Test with zero pose
  frc::Pose3d zero_pose;
  EXPECT_TRUE(PoseUtils::isPoseZero(zero_pose));

  // Test with non-zero pose
  frc::Pose3d non_zero_pose(1_m, 0_m, 0_m, frc::Rotation3d());
  EXPECT_FALSE(PoseUtils::isPoseZero(non_zero_pose));

  frc::Pose3d non_zero_pose_rotation(0_m, 0_m, 0_m, frc::Rotation3d(1_rad, 0_rad, 0_rad));
  // Note: isPoseZero only checks translation (X == 0_m && Y == 0_m && Z == 0_m)
  EXPECT_TRUE(PoseUtils::isPoseZero(non_zero_pose_rotation));
}

TEST(PoseUtilsTest, WpilibTranslationToOpenCV) {
  frc::Translation3d translation(1.5_m, -2.5_m, 3.0_m);
  cv::Point3f opencv_pt = PoseUtils::wpilibTranslationToOpenCV(translation);

  // OpenCV X = -WPILib Y = -(-2.5) = 2.5
  EXPECT_NEAR(opencv_pt.x, 2.5f, 1e-5f);
  // OpenCV Y = -WPILib Z = -3.0
  EXPECT_NEAR(opencv_pt.y, -3.0f, 1e-5f);
  // OpenCV Z = WPILib X = 1.5
  EXPECT_NEAR(opencv_pt.z, 1.5f, 1e-5f);
}

TEST(PoseUtilsTest, OpenCvPoseToWpilib) {
  // Translate 1.0 in X, 2.0 in Y, 3.0 in Z (OpenCV coordinates)
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 1.0, 2.0, 3.0);
  
  // Rotate 90 degrees (pi/2) around the OpenCV X axis.
  // OpenCV X axis in WPILib coordinates is -Y.
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << M_PI / 2.0, 0.0, 0.0);

  frc::Pose3d pose = PoseUtils::openCvPoseToWpilib(rvec, tvec);

  // Translation conversion:
  // WPILib X = OpenCV Z = 3.0
  // WPILib Y = -OpenCV X = -1.0
  // WPILib Z = -OpenCV Y = -2.0
  EXPECT_NEAR(pose.X().value(), 3.0, 1e-5);
  EXPECT_NEAR(pose.Y().value(), -1.0, 1e-5);
  EXPECT_NEAR(pose.Z().value(), -2.0, 1e-5);

  // Rotation:
  // OpenCV X-axis points to WPILib -Y axis.
  // So a 90 degree rotation about OpenCV X is a -90 degree rotation about WPILib Y.
  auto rotation = pose.Rotation();
  EXPECT_NEAR(rotation.Y().value(), -M_PI / 2.0, 1e-5);
  EXPECT_NEAR(rotation.X().value(), 0.0, 1e-5);
  EXPECT_NEAR(rotation.Z().value(), 0.0, 1e-5);
}
