// src/cpp/main.cpp

#include <networktables/NetworkTableInstance.h>

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>

#include "pipeline/PipelineManager.h"

// A global atomic flag is the safest way for a signal handler to communicate
// with the main thread.
std::atomic<bool> g_terminate = false;

// A condition variable allows the main thread to sleep until it's notified,
// which is far more efficient than a busy-wait loop.
std::mutex g_terminate_mutex;
std::condition_variable g_terminate_cv;

/**
 * @brief Signal handler for SIGINT (Ctrl+C) and SIGTERM.
 *
 * This function sets the global termination flag and notifies the main thread
 * to wake up and begin a graceful shutdown.
 * @param signum The signal number that was caught.
 */
void signal_handler(const int signum) {
  std::cout << "\n[Main] Signal " << signum
            << " received, initiating shutdown..." << std::endl;
  g_terminate = true;
  g_terminate_cv.notify_one();  // Wake up the main thread's wait.
}

int main() {
  std::cout << "[Main] GompeiVision Manager starting up." << std::endl;

  // Register our signal_handler to be called on SIGINT (Ctrl+C) or SIGTERM.
  // This is crucial for ensuring stopAll() is called to clean up child
  // processes.
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Get the singleton instance of the PipelineManager.
  auto& manager = PipelineManager::getInstance();
  manager.startAll();

  std::cout << "[Main] Pipelines started. Manager is running." << std::endl;
  std::cout << "[Main] Press Ctrl+C to shut down." << std::endl;

  // The main thread will now block efficiently, waiting for the signal handler
  // to set g_terminate to true.
  std::unique_lock<std::mutex> lock(g_terminate_mutex);
  g_terminate_cv.wait(lock, [] { return g_terminate.load(); });

  // Once woken up, perform a graceful shutdown.
  manager.stopAll();

  std::cout << "[Main] GompeiVision Manager has shut down gracefully."
            << std::endl;
  return 0;
}