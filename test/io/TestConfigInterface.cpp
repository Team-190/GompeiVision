#include <gtest/gtest.h>
#include "io/ConfigInterface.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
#include <thread>
#include <chrono>

TEST(ConfigInterfaceTest, BasicConfigLoad) {
  auto nt_inst = nt::NetworkTableInstance::Create();
  auto configTable = nt_inst.GetTable("/cameras/test_cam/config");

  // Create publishers
  auto rolePub = configTable->GetStringTopic("role").Publish();
  auto cameraMatrixPub = configTable->GetDoubleArrayTopic("camera_matrix").Publish();
  auto distCoeffsPub = configTable->GetDoubleArrayTopic("distortion_coefficients").Publish();
  auto exposurePub = configTable->GetIntegerTopic("exposure").Publish();
  auto gainPub = configTable->GetIntegerTopic("gain").Publish();
  auto widthPub = configTable->GetIntegerTopic("width").Publish();
  auto heightPub = configTable->GetIntegerTopic("height").Publish();
  auto compressedPub = configTable->GetBooleanTopic("compressed").Publish();
  auto setupModePub = configTable->GetBooleanTopic("setup_mode").Publish();

  // Set initial values
  rolePub.Set("test_role");
  cameraMatrixPub.Set(std::vector<double>{1.0, 0.0, 320.0, 0.0, 1.0, 240.0, 0.0, 0.0, 1.0});
  distCoeffsPub.Set(std::vector<double>{0.1, -0.2, 0.0, 0.0, 0.0});
  exposurePub.Set(10);
  gainPub.Set(5);
  widthPub.Set(640);
  heightPub.Set(480);
  compressedPub.Set(true);
  setupModePub.Set(false);

  // Now create the ConfigInterface
  ConfigInterface config("test_cam", nt_inst);
  
  // Wait for initialization
  config.waitForInitialization();

  EXPECT_EQ(config.getRole(), "test_role");
  EXPECT_EQ(config.getExposure(), 10);
  EXPECT_EQ(config.getGain(), 5);
  EXPECT_EQ(config.getWidth(), 640);
  EXPECT_EQ(config.getHeight(), 480);
  EXPECT_TRUE(config.getCompressed());
  EXPECT_FALSE(config.isSetupMode());

  // Test setup mode setter/getter
  config.setSetupMode(true);
  EXPECT_TRUE(config.isSetupMode());

  // Test update with changed values
  exposurePub.Set(20);
  setupModePub.Set(true);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  config.update();

  EXPECT_EQ(config.getExposure(), 20);
  EXPECT_TRUE(config.isSetupMode());
}
