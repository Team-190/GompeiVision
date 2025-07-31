// src/cpp/pipeline/PipelineManager.cpp

#include "pipeline/PipelineManager.h"

#include <chrono>
#include <cstring>     // For strerror
#include <filesystem>  // For robust path manipulation (C++17)
#include <iostream>
#include <string>
#include <vector>

// For process management on POSIX systems
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "openpnp-capture.h"
#include "util/Platform.h"  // Our helper for finding the executable path

struct CapContextDeleter {
  void operator()(const CapContext ctx) const {
    if (ctx) {
      Cap_releaseContext(ctx);
      std::cout << "[Manager] OpenPnP Capture context released." << std::endl;
    }
  }
};
using CapContextPtr = std::unique_ptr<void, CapContextDeleter>;

PipelineManager& PipelineManager::getInstance() {
  static PipelineManager instance;
  return instance;
}

PipelineManager::PipelineManager() {
  std::cout << "[Manager] Initializing Pipeline Manager..." << std::endl;
}

PipelineManager::~PipelineManager() {
  std::cout << "[Manager] Shutting down Pipeline Manager..." << std::endl;
  stopAll();
}

void PipelineManager::startAll() {
  if (m_is_running) {
    std::cout << "[Manager] Pipelines are already running." << std::endl;
    return;
  }

  // --- 1. Robustly determine the worker executable path ---
  const std::string manager_path_str = platform::get_self_executable_path();
  if (manager_path_str.empty()) {
    std::cerr << "[Manager] FATAL: Could not determine own executable path. "
                 "Cannot launch workers."
              << std::endl;
    return;
  }

  // Use std::filesystem to construct the path to the worker, assuming it's
  // in the same directory as the manager executable.
  const std::filesystem::path worker_path =
      std::filesystem::path(manager_path_str).parent_path() /
      "gompei_pipeline_worker";
  const std::string worker_executable_str = worker_path.string();

  std::cout << "[Manager] Resolved worker executable path to: "
            << worker_executable_str << std::endl;

  // --- 2. Detect Cameras ---
  std::cout << "[Manager] Detecting cameras and preparing to launch pipeline "
               "processes..."
            << std::endl;

  const CapContextPtr ctx(Cap_createContext());
  if (!ctx) {
    std::cerr << "[Manager] ERROR: Failed to create openpnp-capture context."
              << std::endl;
    return;
  }

  const uint32_t device_count = Cap_getDeviceCount(ctx.get());
  if (device_count == 0) {
    std::cerr
        << "[Manager] ERROR: No cameras found. GompeiVision will not start."
        << std::endl;
    return;
  }

  std::cout << "[Manager] Found " << device_count << " camera(s)." << std::endl;

  // --- 3. Launch a process for each camera ---
  std::lock_guard<std::mutex> lock(m_pipelines_mutex);

  for (uint32_t i = 0; i < device_count; ++i) {
    constexpr int control_port_base = 1182;
    constexpr int stream_port_base = 5800;
    const char* name_cstr = Cap_getDeviceName(ctx.get(), i);
    const char* id_cstr = Cap_getDeviceUniqueID(ctx.get(), i);

    if (!id_cstr || std::string(id_cstr).empty()) {
      std::cerr << "[Manager] WARNING: Skipping camera "
                << (name_cstr ? name_cstr : "at index " + std::to_string(i))
                << " due to missing unique/hardware ID." << std::endl;
      continue;
    }

    const std::string hardware_id = id_cstr;
    const std::string name = name_cstr ? name_cstr : "Unknown Camera";
    const int device_index = static_cast<int>(i);

    std::cout << "[Manager] Found camera: " << name << " (ID: " << hardware_id
              << ", Index: " << device_index << ")" << std::endl;

    const int stream_port = stream_port_base + (device_index * 2);
    const int control_port = control_port_base + (device_index * 2);
    const int default_width = 1280;
    const int default_height = 720;

    pid_t pid = fork();

    if (pid == -1) {
      std::cerr << "[Manager] ERROR: Failed to fork process for camera "
                << hardware_id << std::endl;
      continue;
    }

    if (pid == 0) {
      // --- This is the child process ---
      std::cout << "[Manager] Starting worker for " << hardware_id << "..."
                << std::endl;

      const std::string dev_idx_str = std::to_string(device_index);
      const std::string width_str = std::to_string(default_width);
      const std::string height_str = std::to_string(default_height);
      const std::string stream_port_str = std::to_string(stream_port);
      const std::string control_port_str = std::to_string(control_port);

      // By convention, argv[0] is the program name itself.
      const std::string worker_name = "GompeiVisionProcess";

      // Prepare arguments for execv. The list must be null-terminated.
      const char* args[] = {worker_name.c_str(),      dev_idx_str.c_str(),
                            hardware_id.c_str(),      width_str.c_str(),
                            height_str.c_str(),       stream_port_str.c_str(),
                            control_port_str.c_str(), nullptr};

      // Replace the child process image with the worker executable.
      execv(worker_executable_str.c_str(), const_cast<char* const*>(args));

      // If execv returns, an error occurred. This is a fatal error for the
      // child.
      std::cerr << "[Worker] FATAL: execv failed for " << worker_executable_str
                << ". Error: " << strerror(errno) << std::endl;
      _exit(1);  // Use _exit in child after fork to avoid calling destructors.
    } else {
      // --- This is the parent process ---
      std::cout << "[Manager] Launched worker for " << hardware_id
                << " with PID " << pid << std::endl;
      m_child_pids[hardware_id] = pid;
    }
  }

  if (!m_child_pids.empty()) {
    m_is_running = true;
    m_heartbeat_thread = std::thread(&PipelineManager::heartbeat_loop, this);
    std::cout << "[Manager] Launched " << m_child_pids.size()
              << " pipeline processes." << std::endl;
  } else {
    std::cerr << "[Manager] ERROR: Failed to launch any pipeline processes."
              << std::endl;
  }
}

void PipelineManager::stopAll() {
  if (!m_is_running.exchange(false)) {
    return;  // Already stopped or stopping
  }

  std::cout << "[Manager] Stopping all pipeline processes..." << std::endl;

  if (m_heartbeat_thread.joinable()) {
    m_heartbeat_thread.join();
  }

  std::lock_guard<std::mutex> lock(m_pipelines_mutex);
  for (const auto& [id, pid] : m_child_pids) {
    std::cout << "[Manager] Sending SIGTERM to process " << pid
              << " for camera " << id << std::endl;
    kill(pid, SIGTERM);
  }

  // Wait for all child processes to terminate.
  int status;
  pid_t wpid;
  while ((wpid = waitpid(-1, &status, 0)) > 0) {
    std::cout << "[Manager] Worker process " << wpid << " terminated."
              << std::endl;
  }

  m_child_pids.clear();
  std::cout << "[Manager] All pipeline processes stopped and cleared."
            << std::endl;
}

void PipelineManager::heartbeat_loop() {
  while (m_is_running) {
    {
      std::lock_guard<std::mutex> lock(m_pipelines_mutex);
      std::cout << "[Manager] Heartbeat: " << m_child_pids.size()
                << " pipeline processes are being managed." << std::endl;

      // Check if processes are still alive and clean up if not.
      std::vector<std::string> dead_pipelines;
      for (const auto& [id, pid] : m_child_pids) {
        // kill with signal 0 is a standard POSIX way to check for existence.
        if (kill(pid, 0) == -1 && errno == ESRCH) {
          std::cerr << "[Manager] Heartbeat: Detected that worker for " << id
                    << " (PID " << pid << ") has died unexpectedly."
                    << std::endl;
          dead_pipelines.push_back(id);
        }
      }

      // Clean up any dead pipelines from our map.
      // In a more advanced system, you might try to restart them here.
      for (const auto& id : dead_pipelines) {
        m_child_pids.erase(id);
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
  std::cout << "[Manager] Heartbeat loop stopped." << std::endl;
}