#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "capture/Camera.h"  // For Camera::CameraInfo
#include "pipeline/Pipeline.h"

/**
 * @class PipelineManager
 * @brief A singleton class to manage the lifecycle of all vision pipelines.
 *
 * This class is responsible for detecting all connected cameras, creating a
 * dedicated Pipeline instance for each one, and managing their start and stop
 * operations. It runs a background thread to monitor the health of the
 * pipelines.
 */
class PipelineManager {
 public:
  /**
   * @brief Gets the singleton instance of the PipelineManager.
   * @return Reference to the singleton instance.
   */
  static PipelineManager& getInstance();

  // Delete copy constructor and assignment operator to enforce singleton
  // pattern
  PipelineManager(const PipelineManager&) = delete;
  void operator=(const PipelineManager&) = delete;

  /**
   * @brief Detects cameras, initializes, and starts a pipeline for each.
   */
  void startAll();

  /**
   * @brief Stops all running pipelines and cleans up resources.
   */
  void stopAll();

 private:
  PipelineManager();
  ~PipelineManager();

  void heartbeat_loop();

  std::map<std::string, std::unique_ptr<Pipeline>> m_pipelines;
  std::mutex m_pipelines_mutex;

  std::thread m_heartbeat_thread;
  std::atomic<bool> m_is_running{false};
};