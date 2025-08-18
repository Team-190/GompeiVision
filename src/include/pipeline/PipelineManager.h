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
 * This class is responsible for detecting all connected cameras, launching a
 * dedicated worker process for each one, and managing their start and stop
 * operations. It runs a background thread to monitor the health of the child
 * processes.
 */
class PipelineManager {
public:
  static PipelineManager& getInstance();

  PipelineManager(const PipelineManager&) = delete;
  void operator=(const PipelineManager&) = delete;

  void startAll(bool testMode);
  void stopAll();

private:
  PipelineManager();
  ~PipelineManager();

  void heartbeat_loop();

  // Stores the Process ID (pid_t) for each running pipeline, keyed by hardware
  // ID.
  std::map<std::string, pid_t> m_child_pids;
  std::mutex m_pipelines_mutex;

  std::thread m_heartbeat_thread;
  std::atomic<bool> m_is_running{false};
};