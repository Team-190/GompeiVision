// src/cpp/pipeline/PipelineManager.cpp

#include "pipeline/PipelineManager.h"

#include <chrono>
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
  std::map<pid_t, int> worker_pipes;
  std::map<pid_t, std::string> pid_to_id;
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

      const char* args[] = {worker_name.c_str(), dev_path_str.c_str(),
                            camera_id.c_str(),   stream_port_str.c_str(),
                            pipe_fd_str.c_str(), nullptr};

      execv(worker_executable_str.c_str(), const_cast<char* const*>(args));
      std::cerr << "[Worker] FATAL: execv failed for " << worker_executable_str
                << ". Error: " << strerror(errno) << std::endl;
      _exit(1);
    } else {
      close(pipe_fds[1]);
      std::cout << "[Manager] Launched worker for " << camera_id << " with PID "
                << pid << "." << std::endl;
      m_child_pids[camera_id] = pid;
      worker_pipes[pid] = pipe_fds[0];
      pid_to_id[pid] = camera_id;
    }
  }

  if (!m_child_pids.empty()) {
    std::cout << "[Manager] Waiting for " << m_child_pids.size()
              << " worker(s) to become ready..." << std::endl;

    fd_set read_fds;
    int max_fd = 0;
    std::vector<pid_t> ready_pids;
    bool timed_out = false;

    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;

    while (ready_pids.size() < m_child_pids.size() && !timed_out) {
      FD_ZERO(&read_fds);
      max_fd = 0;
      for (const auto& [pid, fd] : worker_pipes) {
        FD_SET(fd, &read_fds);
        if (fd > max_fd) {
          max_fd = fd;
        }
      }

      int retval = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);

      if (retval == -1) {
        std::cerr << "[Manager] ERROR: select() failed while waiting. Error: "
                  << strerror(errno) << std::endl;
        timed_out = true;
      } else if (retval == 0) {
        std::cerr << "[Manager] ERROR: Timed out waiting for worker(s)."
                  << std::endl;
        timed_out = true;
      } else {
        for (const auto& [pid, fd] : worker_pipes) {
          if (FD_ISSET(fd, &read_fds)) {
            char buf;
            if (read(fd, &buf, 1) > 0 && buf == 'R') {
              std::cout << "[Manager] Worker for " << pid_to_id[pid] << " (PID "
                        << pid << ") is ready." << std::endl;
              ready_pids.push_back(pid);
            }
          }
        }
      }
    }

    std::vector<std::string> dead_pipelines;
    for (auto it = worker_pipes.begin(); it != worker_pipes.end();) {
      bool is_ready = false;
      for (pid_t ready_pid : ready_pids) {
        if (it->first == ready_pid) {
          is_ready = true;
          break;
        }
      }
      if (!is_ready) {
        const auto& id = pid_to_id[it->first];
        std::cerr << "[Manager] ERROR: Timed out waiting for worker " << id
                  << " (PID " << it->first << "). Killing process."
                  << std::endl;
        kill(it->first, SIGKILL);
        dead_pipelines.push_back(id);
      }
      close(it->second);
      it = worker_pipes.erase(it);
    }

    for (const auto& id : dead_pipelines) {
      m_child_pids.erase(id);
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
