// src/include/pipeline/PipelineManager.h

#pragma once

#include <sys/types.h>  // Required for pid_t

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>

/**
 * @class PipelineManager
 * @brief A singleton class to manage the lifecycle of all vision pipeline
 * processes.
 *
 * This class is responsible for continuously polling for connected cameras
 * (/dev/camera_*), launching a dedicated worker process for each one, and
 * managing their lifecycle. It runs a background management thread to handle
 * camera discovery, worker launch, and cleanup of disconnected or crashed
 * processes.
 */
class PipelineManager {
 public:
  /**
   * @brief Get the singleton instance of the PipelineManager.
   * @return Reference to the singleton instance.
   */
  static PipelineManager& getInstance();

  // Delete copy constructor and assignment operator to enforce singleton
  // pattern.
  PipelineManager(const PipelineManager&) = delete;
  void operator=(const PipelineManager&) = delete;

  /**
   * @brief Starts the camera polling and process management loop.
   */
  void startAll();

  /**
   * @brief Stops all running pipeline processes and shuts down the management
   * loop.
   */
  void stopAll();

 private:
  /**
   * @brief Private constructor to enforce singleton pattern.
   */
  PipelineManager();

  /**
   * @brief Destructor that ensures all child processes are stopped.
   */
  ~PipelineManager();

  /**
   * @brief The main management loop that runs in a separate thread.
   *
   * This function periodically scans for camera devices, launches new worker
   * processes for new cameras, and cleans up workers for cameras that have
   * been disconnected or have crashed.
   */
  void management_loop();

  /**
   * @brief Launches a new worker process for a specific camera device.
   * @param device_path The full path to the camera device (e.g.,
   * "/dev/camera_0").
   */
  void launch_worker_for_camera(const std::string& device_path);

  // Stores the Process ID (pid_t) for each running pipeline, keyed by camera
  // ID.
  std::map<std::string, pid_t> m_child_pids;
  std::mutex m_pipelines_mutex;

  // The thread that runs the main management_loop.
  std::thread m_management_thread;
  std::atomic<bool> m_is_running{false};
};
