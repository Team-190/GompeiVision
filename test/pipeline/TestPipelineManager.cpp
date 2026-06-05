#include <gtest/gtest.h>
#include "pipeline/PipelineManager.h"
#include <chrono>
#include <thread>

TEST(PipelineManagerTest, SingletonAndLifecycle) {
  PipelineManager& manager = PipelineManager::getInstance();
  
  manager.startAll();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  manager.stopAll();
}
