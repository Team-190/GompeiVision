#include <gtest/gtest.h>
#include "capture/Camera.h"
#include <opencv2/core.hpp>

TEST(CameraTest, DisconnectedBehavior) {
  Camera camera("/dev/non_existent_video_device_123", "test_cam_hw", 640, 480);

  EXPECT_FALSE(camera.isConnected());
  EXPECT_DOUBLE_EQ(camera.getSpeed(), 480.0);
  EXPECT_EQ(camera.getFormat(), "");

  cv::Mat frame;
  int64_t timestamp = 0;
  EXPECT_FALSE(camera.getFrame(frame, timestamp));

  EXPECT_FALSE(camera.setExposure(50));
  EXPECT_FALSE(camera.setBrightness(100));

  EXPECT_FALSE(camera.attemptReconnect());
}
