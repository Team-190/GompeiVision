#include "pipeline/PipelineProcess.h"

#include <unistd.h>  // For write() and close()

#include <chrono>
#include <csignal>
#include <cstring>  // For strerror
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "networktables/NetworkTableInstance.h"
#include "pipeline/Pipeline.h"

// Global pointer to the pipeline to be accessible by the signal handler.
std::unique_ptr<Pipeline> g_pipeline;

void signal_handler(const int signum) {
  std::cout << "[Worker] Signal (" << signum
            << ") received. Shutting down pipeline..." << std::endl;
  if (g_pipeline) {
    g_pipeline->stop();
  }
}

int main(const int argc, char* argv[]) {
  if (argc < 6) {
    std::cerr << "Usage: " << argv[0]
              << " <device_path> <hardware_id> <stream_port> <pipe_write_fd> <test_mode>"
              << std::endl;
    return 1;
  }

  int pipe_write_fd = -1;
  try {
    const std::string device_path = argv[1];
    const std::string hardware_id = argv[2];
    const int stream_port = std::stoi(argv[3]);
    pipe_write_fd = std::stoi(argv[4]);
    const bool testMode = std::stoi(argv[5]);


    // Each worker process must initialize its own NT client.
    nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();
    nt_inst.StartClient4("GompeiVision-" + hardware_id);
    nt_inst.SetServer("10.1.90.2");

    // Register signal handlers for graceful shutdown.
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    std::cout << "[Worker] Creating pipeline for device " << hardware_id
              << " (path " << device_path << ")" << std::endl;

    g_pipeline =
        std::make_unique<Pipeline>(device_path, hardware_id, stream_port, nt_inst, testMode);
    g_pipeline->start();

    // --- Signal readiness to manager by writing to the pipe ---
    std::cout << "[Worker] Signaling readiness to manager via pipe fd "
              << pipe_write_fd << std::endl;
    constexpr char ready_signal = 'R';
    if (write(pipe_write_fd, &ready_signal, 1) != 1) {
      std::cerr
          << "[Worker] FATAL: Failed to write readiness signal to pipe. Error: "
          << strerror(errno) << std::endl;
      close(pipe_write_fd);
      return 1;
    }

    close(pipe_write_fd);
    std::cout << "[Worker] Readiness signal sent and pipe closed." << std::endl;

    std::cout << "[Worker] Pipeline started. Running until signal..."
              << std::endl;

    // The pipeline runs in its own threads. The main thread waits until the
    // pipeline is stopped (e.g., by the signal handler).
    while (g_pipeline->isRunning()) {
      g_pipeline->during();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[Worker] Pipeline for " << hardware_id << " has shut down."
              << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "[Worker] ERROR: An exception occurred: " << e.what()
              << std::endl;
    if (pipe_write_fd != -1) {
      // In case the exception happened after the fd was assigned but before it
      // was closed.
      close(pipe_write_fd);
    }
    return 1;
  }

  return 0;
}
