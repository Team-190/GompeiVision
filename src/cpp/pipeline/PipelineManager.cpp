// src/cpp/pipeline/PipelineManager.cpp

#include "pipeline/PipelineManager.h"

#include <chrono>
#include <cstdlib>
#include <cstring>     // For strerror
#include <filesystem>  // For robust path manipulation (C++17)
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

// For process management on POSIX systems
#include <signal.h>
#include <sys/select.h>  // For select()
#include <sys/wait.h>
#include <unistd.h>

#include <ranges>

#include "io/FieldInterface.h"
#include "openpnp-capture.h"
#include "util/Platform.h"  // Our helper for finding the executable path

// (CapContextDeleter and CapContextPtr remain unchanged)

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
  if (m_is_running.exchange(true)) {
    std::cout << "[Manager] Pipelines are already running or starting." << std::endl;
    return;
  }

  // The main logic is now in the management loop.
  // This thread will handle discovering, launching, and cleaning up workers.
  m_management_thread = std::thread(&PipelineManager::management_loop, this);
  std::cout << "[Manager] Management loop started. Now polling for cameras." << std::endl;
}

void PipelineManager::stopAll() {
  if (!m_is_running.exchange(false)) {
    return;  // Already stopped or stopping
  }

  std::cout << "[Manager] Stopping all pipeline processes..." << std::endl;

  if (m_management_thread.joinable()) {
    m_management_thread.join();
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

void PipelineManager::launch_worker_for_camera(const std::string& device_path) {
    // This function encapsulates the logic for forking and executing a new worker.
    // It's called by the management loop whenever a new camera is detected.

    const std::string manager_path_str = platform::get_self_executable_path();
    if (manager_path_str.empty()) {
        std::cerr << "[Manager] FATAL: Could not determine own executable path. "
                    "Cannot launch workers."
                << std::endl;
        return;
    }

    const std::filesystem::path worker_path =
        std::filesystem::path(manager_path_str).parent_path() /
        "GompeiVisionProcess";
    const std::string worker_executable_str = worker_path.string();

    const std::string camera_id = std::filesystem::path(device_path).filename().string();

    // Use a simple counter for port assignment to avoid conflicts
    static int device_counter = 0;
    constexpr int stream_port_base = 5800;
    const int stream_port = stream_port_base + (device_counter++ * 2);

    std::cout << "[Manager] Preparing to launch for camera: " << camera_id
              << " (Device: " << device_path << ")" << std::endl;

    int pipe_fds[2];
    if (pipe(pipe_fds) == -1) {
      std::cerr << "[Manager] ERROR: Failed to create pipe for " << camera_id
                << ". Error: " << strerror(errno) << std::endl;
      return;
    }

    pid_t pid = fork();

    if (pid == -1) {
      std::cerr << "[Manager] ERROR: Failed to fork process for camera "
                << camera_id << std::endl;
      close(pipe_fds[0]);
      close(pipe_fds[1]);
      return;
    }

    if (pid == 0) { // Child process
      close(pipe_fds[0]);
      const std::string stream_port_str = std::to_string(stream_port);
      const std::string pipe_fd_str = std::to_string(pipe_fds[1]);
      const std::string worker_name = "GompeiVisionProcess";

      const char* args[] = {worker_name.c_str(), device_path.c_str(),
                            camera_id.c_str(),   stream_port_str.c_str(),
                            pipe_fd_str.c_str(), nullptr};

      execv(worker_executable_str.c_str(), const_cast<char* const*>(args));
      std::cerr << "[Worker] FATAL: execv failed for " << worker_executable_str
                << ". Error: " << strerror(errno) << std::endl;
      _exit(1);
    } else { // Parent process
      close(pipe_fds[1]);
      std::cout << "[Manager] Launched worker for " << camera_id << " with PID "
                << pid << "." << std::endl;
      std::cout << "[Manager] Waiting for worker " << camera_id << " (PID " << pid << ") to become ready..." << std::endl;

      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(pipe_fds[0], &read_fds);

      struct timeval timeout;
      timeout.tv_sec = 10;
      timeout.tv_usec = 0;

      int retval = select(pipe_fds[0] + 1, &read_fds, nullptr, nullptr, &timeout);

      bool is_ready = false;
      if (retval > 0 && FD_ISSET(pipe_fds[0], &read_fds)) {
          char buf;
          if (read(pipe_fds[0], &buf, 1) > 0 && buf == 'R') {
              std::cout << "[Manager] Worker for " << camera_id << " (PID " << pid << ") is ready." << std::endl;
              is_ready = true;
          }
      }

      close(pipe_fds[0]);

      if (is_ready) {
          // The lock is already held by the calling function (management_loop)
          m_child_pids[camera_id] = pid;
      } else {
          std::cerr << "[Manager] ERROR: Worker " << camera_id << " (PID " << pid << ") failed to become ready. Killing process." << std::endl;
          kill(pid, SIGKILL);
          waitpid(pid, nullptr, 0);
      }
    }
}

void PipelineManager::management_loop() {
  while (m_is_running) {
    // 1. Discover all currently available cameras
    std::set<std::string> available_cameras;
    const std::string base = "/dev";
    try {
        for (const auto& entry : std::filesystem::directory_iterator(base)) {
            if (entry.is_symlink()) {
                const std::string name = entry.path().filename().string();
                if (name.rfind("camera_", 0) == 0) {
                    available_cameras.insert(name);
                }
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[Manager] Error scanning /dev: " << e.what() << std::endl;
    }


    { // Lock scope for modifying the child process map
      std::lock_guard<std::mutex> lock(m_pipelines_mutex);

      // 2. Launch workers for new cameras
      for (const auto& camera_id : available_cameras) {
        if (m_child_pids.find(camera_id) == m_child_pids.end()) {
          std::cout << "[Manager] Discovered new camera: " << camera_id << std::endl;
          std::string full_path = base + "/" + camera_id;
          launch_worker_for_camera(full_path);
        }
      }

      // 3. Clean up workers for disconnected or crashed cameras
      std::vector<std::string> dead_pipelines;
      for (const auto& [id, pid] : m_child_pids) {
        bool is_disconnected = available_cameras.find(id) == available_cameras.end();
        bool is_crashed = (kill(pid, 0) == -1 && errno == ESRCH);

        if (is_disconnected) {
            std::cerr << "[Manager] Detected that camera " << id
                      << " (PID " << pid << ") was disconnected." << std::endl;
            kill(pid, SIGTERM); // Gracefully terminate the worker
            dead_pipelines.push_back(id);
        } else if (is_crashed) {
            std::cerr << "[Manager] Detected that worker for " << id
                      << " (PID " << pid << ") has died unexpectedly."
                      << std::endl;
            dead_pipelines.push_back(id);
        }
      }

      for (const auto& id : dead_pipelines) {
        m_child_pids.erase(id);
      }
    } // End lock scope

    // Poll every 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  std::cout << "[Manager] Management loop stopped." << std::endl;
}
