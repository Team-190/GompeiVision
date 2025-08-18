// src/cpp/pipeline/PipelineManager.cpp

#include "pipeline/PipelineManager.h"

#include <chrono>
#include <cstdlib>
#include <cstring>     // For strerror
#include <filesystem>  // For robust path manipulation (C++17)
#include <iostream>
#include <map>
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

void PipelineManager::startAll(bool testMode) {
  if (m_is_running) {
    std::cout << "[Manager] Pipelines are already running." << std::endl;
    return;
  }

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

  std::cout << "[Manager] Resolved worker executable path to: "
            << worker_executable_str << std::endl;

  std::vector<std::string> camera_symlinks;
  const std::string base = "/dev";
  for (const auto& entry : std::filesystem::directory_iterator(base)) {
    if (entry.is_symlink()) {
      const std::string name = entry.path().filename().string();
      if (name.rfind("camera_", 0) == 0) {
        camera_symlinks.push_back(entry.path().string());
      }
    }
  }

  if (camera_symlinks.empty()) {
    std::cerr << "[Manager] ERROR: No /dev/camera_* devices found."
              << std::endl;
    return;
  }

  std::cout << "[Manager] Found " << camera_symlinks.size() << " camera(s)."
            << std::endl;

  std::lock_guard<std::mutex> lock(m_pipelines_mutex);
  constexpr int stream_port_base = 5800;

  for (size_t i = 0; i < camera_symlinks.size(); ++i) {
    const std::string& device_path = camera_symlinks[i];
    const std::string camera_id = std::filesystem::path(device_path).filename();
    const int device_index = static_cast<int>(i);

    std::cout << "[Manager] Preparing to launch for camera: " << camera_id
              << " (Device: " << device_path << ")" << std::endl;

    int pipe_fds[2];
    if (pipe(pipe_fds) == -1) {
      std::cerr << "[Manager] ERROR: Failed to create pipe for " << camera_id
                << ". Error: " << strerror(errno) << std::endl;
      continue;
    }

    const int stream_port = stream_port_base + (device_index * 2);
    pid_t pid = fork();

    if (pid == -1) {
      std::cerr << "[Manager] ERROR: Failed to fork process for camera "
                << camera_id << std::endl;
      close(pipe_fds[0]);
      close(pipe_fds[1]);
      continue;
    }

    if (pid == 0) {
      close(pipe_fds[0]);
      const std::string dev_path_str = device_path;
      const std::string stream_port_str = std::to_string(stream_port);
      const std::string pipe_fd_str = std::to_string(pipe_fds[1]);
      const std::string worker_name = "GompeiVisionProcess";
      const std::string test_mode = std::to_string(testMode);

      const char* args[] = {worker_name.c_str(), dev_path_str.c_str(),
                            camera_id.c_str(),   stream_port_str.c_str(),
                            pipe_fd_str.c_str(), test_mode.c_str(), nullptr};

      execv(worker_executable_str.c_str(), const_cast<char* const*>(args));
      std::cerr << "[Worker] FATAL: execv failed for " << worker_executable_str
                << ". Error: " << strerror(errno) << std::endl;
      _exit(1);
    } else {
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
      if (retval == -1) {
          std::cerr << "[Manager] ERROR: select() failed while waiting for " << camera_id << ". Error: " << strerror(errno) << std::endl;
      } else if (retval == 0) {
          std::cerr << "[Manager] ERROR: Timed out waiting for worker " << camera_id << "." << std::endl;
      } else {
          if (FD_ISSET(pipe_fds[0], &read_fds)) {
              char buf;
              if (read(pipe_fds[0], &buf, 1) > 0 && buf == 'R') {
                  std::cout << "[Manager] Worker for " << camera_id << " (PID " << pid << ") is ready." << std::endl;
                  is_ready = true;
              } else {
                  std::cerr << "[Manager] ERROR: Failed to get ready signal from worker " << camera_id << "." << std::endl;
              }
          }
      }

      close(pipe_fds[0]);

      if (is_ready) {
          m_child_pids[camera_id] = pid;
      } else {
          std::cerr << "[Manager] ERROR: Worker " << camera_id << " (PID " << pid << ") failed to become ready. Killing process." << std::endl;
          kill(pid, SIGKILL);
          waitpid(pid, nullptr, 0);
      }
    }
  }

  if (testMode && m_child_pids.size() == camera_symlinks.size() && !camera_symlinks.empty()) {
    std::cout << "[Manager] All pipelines successfully started. Rebooting system." << std::endl;
    system("reboot now");
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
      if (!m_child_pids.empty()) {
        std::cout << "[Manager] Heartbeat: " << m_child_pids.size()
                  << " pipeline processes are being managed." << std::endl;
      }

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

      for (const auto& id : dead_pipelines) {
        m_child_pids.erase(id);
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
  std::cout << "[Manager] Heartbeat loop stopped." << std::endl;
}
