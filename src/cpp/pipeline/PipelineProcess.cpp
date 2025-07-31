#include "pipeline/PipelineProcess.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

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
  if (argc < 7) {
    std::cerr << "Usage: " << argv[0]
              << " <device_index> <hardware_id> <width> <height> <stream_port> "
                 "<control_port>"
              << std::endl;
    return 1;
  }

  try {
    const int device_index = std::stoi(argv[1]);
    const std::string hardware_id = argv[2];
    const int stream_port = std::stoi(argv[3]);
    const int control_port = std::stoi(argv[4]);

    // Register signal handlers for graceful shutdown.
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    std::cout << "[Worker] Creating pipeline for device " << hardware_id
              << " (index " << device_index << ")" << std::endl;

    g_pipeline = std::make_unique<Pipeline>(device_index, hardware_id,
                                            stream_port, control_port);
    g_pipeline->start();

    std::cout << "[Worker] Pipeline started. Running until signal..."
              << std::endl;

    // The pipeline runs in its own threads. The main thread waits until the
    // pipeline is stopped (e.g., by the signal handler).
    while (g_pipeline->isRunning()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "[Worker] Pipeline for " << hardware_id << " has shut down."
              << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "[Worker] ERROR: An exception occurred: " << e.what()
              << std::endl;
    return 1;
  }

  return 0;
}