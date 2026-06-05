#include <gtest/gtest.h>
#include "pipeline/Pipeline.h"
#include "io/FieldInterface.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
#include <chrono>
#include <thread>

TEST(PipelineTest, BasicLifecycle) {
  auto nt_inst = nt::NetworkTableInstance::Create();

  // 1. Initialize FieldInterface and publish field layout (22 tags)
  FieldInterface::initialize(nt_inst);
  auto fieldTable = nt_inst.GetTable("field");
  std::vector<nt::DoubleArrayPublisher> field_pubs;
  for (int i = 1; i <= 22; i++) {
    auto pub = fieldTable->GetDoubleArrayTopic("tag_" + std::to_string(i)).Publish();
    pub.Set(std::vector<double>{static_cast<double>(i), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
    field_pubs.push_back(std::move(pub));
  }

  // Force NT data synchronization and ensure FieldInterface is ready
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  bool field_ready = FieldInterface::update();
  EXPECT_TRUE(field_ready);

  // 2. Publish camera configuration to satisfy ConfigInterface
  auto configTable = nt_inst.GetTable("/cameras/test_pipeline_cam/config");
  auto rolePub = configTable->GetStringTopic("role").Publish();
  auto cameraMatrixPub = configTable->GetDoubleArrayTopic("camera_matrix").Publish();
  auto distCoeffsPub = configTable->GetDoubleArrayTopic("distortion_coefficients").Publish();
  auto exposurePub = configTable->GetIntegerTopic("exposure").Publish();
  auto gainPub = configTable->GetIntegerTopic("gain").Publish();
  auto widthPub = configTable->GetIntegerTopic("width").Publish();
  auto heightPub = configTable->GetIntegerTopic("height").Publish();
  auto compressedPub = configTable->GetBooleanTopic("compressed").Publish();
  auto setupModePub = configTable->GetBooleanTopic("setup_mode").Publish();

  rolePub.Set("test_pipeline_role");
  cameraMatrixPub.Set(std::vector<double>{500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0});
  distCoeffsPub.Set(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
  exposurePub.Set(10);
  gainPub.Set(5);
  widthPub.Set(640);
  heightPub.Set(480);
  compressedPub.Set(false);
  setupModePub.Set(false);

  // 3. Construct the Pipeline
  Pipeline pipeline("/dev/non_existent_device_456", "test_pipeline_cam", 8888, nt_inst);

  EXPECT_FALSE(pipeline.isRunning());

  // 4. Start the pipeline threads
  pipeline.start();
  EXPECT_TRUE(pipeline.isRunning());

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Run the periodic "during" update
  pipeline.during();

  // Test setup mode toggle on the pipeline
  setupModePub.Set(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  pipeline.during(); // Should enable setup mode MJPEG server

  setupModePub.Set(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  pipeline.during(); // Should disable setup mode MJPEG server

  // 5. Stop the pipeline
  pipeline.stop();
  EXPECT_FALSE(pipeline.isRunning());
}
