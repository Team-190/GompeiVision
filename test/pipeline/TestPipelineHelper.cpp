#include <gtest/gtest.h>
#include "pipeline/PipelineHelper.h"
#include "io/ConfigInterface.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/DoubleArrayTopic.h>

TEST(PipelineHelperTest, LoadCameraIntrinsicsSuccess) {
  auto nt_inst = nt::NetworkTableInstance::Create();
  auto configTable = nt_inst.GetTable("/cameras/test_cam/config");

  auto rolePub = configTable->GetStringTopic("role").Publish();
  auto cameraMatrixPub = configTable->GetDoubleArrayTopic("camera_matrix").Publish();
  auto distCoeffsPub = configTable->GetDoubleArrayTopic("distortion_coefficients").Publish();
  
  rolePub.Set("test_role");
  cameraMatrixPub.Set(std::vector<double>{1.0, 0.0, 320.0, 0.0, 1.0, 240.0, 0.0, 0.0, 1.0});
  distCoeffsPub.Set(std::vector<double>{0.1, -0.2, 0.0, 0.0, 0.0});

  ConfigInterface config("test_cam", nt_inst);
  config.waitForInitialization();

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  bool success = PipelineHelper::load_camera_intrinsics(config, cameraMatrix, distCoeffs);
  EXPECT_TRUE(success);
  EXPECT_EQ(cameraMatrix.rows, 3);
  EXPECT_EQ(cameraMatrix.cols, 3);
  EXPECT_EQ(distCoeffs.cols, 5);

  // Exercise instantiate/destruct
  PipelineHelper helper;
}

TEST(PipelineHelperTest, LoadCameraIntrinsicsEmpty) {
  auto nt_inst = nt::NetworkTableInstance::Create();
  auto configTable = nt_inst.GetTable("/cameras/test_cam_empty/config");

  auto rolePub = configTable->GetStringTopic("role").Publish();
  auto cameraMatrixPub = configTable->GetDoubleArrayTopic("camera_matrix").Publish();
  
  rolePub.Set("test_role_empty");
  cameraMatrixPub.Set(std::vector<double>{1.0, 0.0, 320.0, 0.0, 1.0, 240.0, 0.0, 0.0, 1.0});

  ConfigInterface config("test_cam_empty", nt_inst);
  config.waitForInitialization();

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  bool success = PipelineHelper::load_camera_intrinsics(config, cameraMatrix, distCoeffs);
  EXPECT_FALSE(success); // Fails because distortion coeffs are empty
}
