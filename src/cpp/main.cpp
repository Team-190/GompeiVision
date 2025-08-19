// src/cpp/main.cpp

#include <networktables/NetworkTableInstance.h>

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <filesystem>

#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <libusb>

#include "pipeline/PipelineManager.h"

// A global atomic flag is the safest way for a signal handler to communicate
// with the main thread.
std::atomic<bool> g_terminate = false;

// A condition variable allows the main thread to sleep until it's notified,
// which is far more efficient than a busy-wait loop.
std::mutex g_terminate_mutex;
std::condition_variable g_terminate_cv;

bool isTestModeUSBConnected() {
  if (const std::string symlink_path = "/dev/gompei_testkey";
      std::filesystem::exists(symlink_path)) {
    std::cout << "[USB TestMode] Key detected via symlink." << std::endl;
    return true;
  }
  return false;
}

// Reset USB devices that are enumerated below USB 3
void resetUSBIfNotSuperSpeed() {
  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    std::cerr << "[USB] Failed to initialize libusb." << std::endl;
    return;
  }

  libusb_device** list = nullptr;
  ssize_t cnt = libusb_get_device_list(ctx, &list);
  if (cnt < 0) {
    std::cerr << "[USB] Failed to get device list." << std::endl;
    libusb_exit(ctx);
    return;
  }

  for (ssize_t i = 0; i < cnt; ++i) {
    libusb_device* dev = list[i];
    int speed = libusb_get_device_speed(dev);
    if (speed < LIBUSB_SPEED_SUPER) { // LIBUSB_SPEED_SUPER = USB 3
      std::cout << "[USB Reset] Device " << i
                << " enumerated at speed " << speed
                << ", attempting reset..." << std::endl;

      libusb_device_handle* handle = nullptr;
      if (libusb_open(dev, &handle) == 0) {
        if (libusb_reset_device(handle) == 0) {
          std::cout << "[USB Reset] Device " << i << " reset successfully." << std::endl;
        } else {
          std::cerr << "[USB Reset] Failed to reset device " << i << std::endl;
        }
        libusb_close(handle);
      } else {
        std::cerr << "[USB Reset] Cannot open device " << i << std::endl;
      }
    }
  }

  libusb_free_device_list(list, 1);
  libusb_exit(ctx);

  // Wait a moment for devices to re-enumerate
  std::this_thread::sleep_for(std::chrono::seconds(2));
}

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
  const bool testMode = isTestModeUSBConnected();
  std::cout << "[Main] GompeiVision Manager starting up." << std::endl;

  // Register our signal_handler to be called on SIGINT (Ctrl+C) or SIGTERM.
  // This is crucial for ensuring stopAll() is called to clean up child
  // processes.
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Get the singleton instance of the PipelineManager.
  auto& manager = PipelineManager::getInstance();
  manager.startAll(testMode);

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