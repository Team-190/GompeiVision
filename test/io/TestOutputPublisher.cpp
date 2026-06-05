#include <gtest/gtest.h>
#include "io/OutputPublisher.h"
#include "util/QueuedFiducialData.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>
#include <thread>
#include <chrono>

TEST(OutputPublisherTest, NTOutputPublisherBasic) {
  auto nt_inst = nt::NetworkTableInstance::Create();
  NTOutputPublisher publisher("test_cam", nt_inst);

  auto outputTable = nt_inst.GetTable("/cameras/test_cam/output");
  auto connSub = outputTable->GetBooleanTopic("connected").Subscribe(false);
  auto capFpsSub = outputTable->GetIntegerTopic("capture_fps").Subscribe(0);
  auto procFpsSub = outputTable->GetIntegerTopic("processing_fps").Subscribe(0);
  auto obsSub = outputTable->GetDoubleArrayTopic("observations").Subscribe({});

  publisher.SendConnectionStatus(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_TRUE(connSub.Get());

  publisher.SendCaptureFPS(60);
  publisher.SendProcessingFPS(45);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(capFpsSub.Get(), 60);
  EXPECT_EQ(procFpsSub.Get(), 45);

  AprilTagResult result;
  result.timestamp = 987654321;
  
  result.multi_tag_pose.tag_ids = {7};
  result.multi_tag_pose.pose_0 = frc::Pose3d(1_m, 2_m, 3_m, frc::Rotation3d(frc::Quaternion(1.0, 0.0, 0.0, 0.0)));
  result.multi_tag_pose.error_0 = 0.05;
  result.multi_tag_pose.pose_1 = frc::Pose3d(1.1_m, 2.1_m, 3.1_m, frc::Rotation3d(frc::Quaternion(1.0, 0.0, 0.0, 0.0)));
  result.multi_tag_pose.error_1 = 0.15;

  TagAngleObservation angle_obs;
  angle_obs.tag_id = 7;
  angle_obs.corners_angles = {0.1, -0.1, 0.2, -0.2};
  angle_obs.distance = 1.23;
  result.tag_angles.push_back(angle_obs);

  publisher.SendAprilTagResult(result);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  std::vector<double> obs = obsSub.Get();
  
  ASSERT_EQ(obs.size(), 23u);
  EXPECT_DOUBLE_EQ(obs[0], 2.0);
  EXPECT_DOUBLE_EQ(obs[1], 0.05);
  EXPECT_DOUBLE_EQ(obs[2], 1.0);
  EXPECT_DOUBLE_EQ(obs[17], 7.0);
  EXPECT_DOUBLE_EQ(obs[18], 0.1);
  EXPECT_DOUBLE_EQ(obs[22], 1.23);
}
