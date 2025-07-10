#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <thread>

#include "capture/Camera.h"
#include "util/QueuedFrame.h"
#include "util/ThreadSafeQueue.h"

// --- Global variables for this test ---
ThreadSafeQueue<QueuedFrame> frame_queue;
std::atomic<bool> g_is_running(true);
constexpr size_t MAX_QUEUE_SIZE = 99;  // New: Set a max size for the queue

/**
 * @brief The producer thread function. Captures frames and pushes them to the
 * queue.
 */
void producer_thread(Camera* camera) {
  std::cout << "[Producer] Thread started." << std::endl;

  cv::Mat frame;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;

  while (g_is_running) {
    if (camera->getFrame(frame, timestamp)) {
      // --- KEY CHANGE: Check queue size before pushing ---
      if (frame_queue.size() < MAX_QUEUE_SIZE) {
        QueuedFrame q_frame;
        q_frame.frame = frame.clone();
        q_frame.timestamp = timestamp;
        q_frame.cameraRole = camera->getRole();
        frame_queue.push(q_frame);
      } else {
        // If the queue is full, we drop the frame to prevent lag.
        // std::cout << "[Producer] Queue full, dropping frame." << std::endl;
      }
    } else {
      std::cout << "[Producer] Dropped a frame." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  std::cout << "[Producer] Thread finished." << std::endl;
}

int main() {
  std::cout << "--- ThreadSafeQueue Test Program ---" << std::endl;

  // --- Camera Setup with Resilient Format Finding ---
  CapContext ctx = Cap_createContext();
  if (!ctx || Cap_getDeviceCount(ctx) == 0) {
    std::cerr << "ERROR: No camera found or context failed." << std::endl;
    return -1;
  }

  CapDeviceID device_index = 0;
  const char* id_ptr = Cap_getDeviceUniqueID(ctx, 0);
  std::string camera_id(id_ptr ? id_ptr : "");

  std::unique_ptr<Camera> camera = nullptr;
  int num_formats = Cap_getNumFormats(ctx, device_index);
  cv::Mat test_frame;
  std::chrono::time_point<std::chrono::steady_clock> test_timestamp;

  std::cout << "Found " << num_formats
            << " available formats. Verifying each one..." << std::endl;

  for (int i = 0; i < num_formats; ++i) {
    auto temp_camera =
        std::make_unique<Camera>(ctx, device_index, i, camera_id);
    if (temp_camera && temp_camera->isConnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (temp_camera->getFrame(test_frame, test_timestamp)) {
        std::cout << "SUCCESS: Found working format ID: " << i << std::endl;
        camera = std::move(temp_camera);
        break;
      }
    }
  }

  if (!camera) {
    std::cerr << "ERROR: Could not find any working format for the camera."
              << std::endl;
    Cap_releaseContext(ctx);
    return -1;
  }
  camera->setRole("test_cam");

  // --- Thread Launch ---
  std::cout << "\nStarting producer thread..." << std::endl;
  std::thread producer(producer_thread, camera.get());

  // --- Main Thread acts as the Consumer ---
  std::cout << "Consumer (main thread) started." << std::endl;
  const std::string window_name = "Queue Test";
  cv::namedWindow(window_name);

  while (g_is_running) {
    QueuedFrame q_frame;
    if (frame_queue.waitAndPop(q_frame)) {
      if (!q_frame.frame.empty()) {
        std::string text = "Queue Size: " + std::to_string(frame_queue.size());
        cv::putText(q_frame.frame, text, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::imshow(window_name, q_frame.frame);
      }
    }

    if (int key = cv::waitKey(1); key == 'q') {
      g_is_running = false;  // Signal producer thread to stop
    }
  }

  // --- KEY CHANGE: Correct Shutdown Order ---
  std::cout
      << "Shutdown signal received. Waiting for producer thread to finish..."
      << std::endl;

  // 1. Unblock any waiting threads (like the producer if it were waiting on a
  // full queue)
  frame_queue.shutdown();

  // 2. Wait for the producer thread to completely finish its work.
  if (producer.joinable()) {
    producer.join();
  }
  std::cout << "Producer thread joined." << std::endl;

  // 3. NOW it is safe to clean up the resources the thread was using.
  cv::destroyAllWindows();
  // The camera unique_ptr destructor will be called here, safely.
  camera.reset();
  Cap_releaseContext(ctx);

  std::cout << "--- Test Program Finished ---" << std::endl;
  return 0;
}
