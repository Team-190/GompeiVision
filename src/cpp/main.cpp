#include <apriltag/apriltag.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <openpnp-capture.h>

#include <chrono>  // Required for std::chrono
#include <iostream>
#include <opencv2/highgui.hpp>  // For the windowing system
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>  // Required for std::this_thread
#include <vector>

#include "capture/Camera.h"
#include "openpnp-capture.h"  // The C library header

void testDeps() {
  // Testing objects
  nt::NetworkTableInstance ntinst = nt::NetworkTableInstance::GetDefault();
  apriltag_detection_t* detection = nullptr;
  apriltag_detection_destroy(detection);
  auto image = cv::Mat();
  CapResult result = CapResult();
}

// Helper function to convert a FourCC integer code to a readable string
std::string fourcc_to_string(const uint32_t fourcc) {
  std::string s = "    ";
  s[0] = fourcc & 0xFF;
  s[1] = (fourcc >> 8) & 0xFF;
  s[2] = (fourcc >> 16) & 0xFF;
  s[3] = (fourcc >> 24) & 0xFF;
  return s;
}

[[noreturn]] int camTest() {
  std::cout << "--- Headless Camera Latency Test ---" << std::endl;

  CapContext ctx = Cap_createContext();
  if (!ctx) {
    std::cerr << "ERROR: Could not create openpnp-capture context."
              << std::endl;
    return -1;
  }

  if (Cap_getDeviceCount(ctx) == 0) {
    std::cout << "No cameras found. Skipping hardware test." << std::endl;
    Cap_releaseContext(ctx);
    return 0;
  }

  CapDeviceID device_index = 0;
  const char* id_ptr = Cap_getDeviceUniqueID(ctx, device_index);
  std::string camera_id(id_ptr ? id_ptr : "");

  std::unique_ptr<Camera> camera = nullptr;
  int num_formats = Cap_getNumFormats(ctx, device_index);
  cv::Mat test_frame;
  std::chrono::time_point<std::chrono::steady_clock> test_timestamp;

  std::cout << "Found " << num_formats
            << " available formats. Verifying each one..." << std::endl;

  for (int i = 0; i < num_formats; ++i) {
    CapFormatInfo info;
    if (Cap_getFormatInfo(ctx, device_index, i, &info) == CAPRESULT_OK) {
      std::cout << "--> Trying Format ID " << i << ": " << info.width << "x"
                << info.height << " @ " << info.fps << " FPS"
                << " (" << fourcc_to_string(info.fourcc) << ")" << std::endl;
    } else {
      continue;
    }

    auto temp_camera =
        std::make_unique<Camera>(ctx, device_index, i, camera_id);

    if (temp_camera && temp_camera->isConnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      if (temp_camera->getFrame(test_frame, test_timestamp)) {
        std::cout << "    SUCCESS: Successfully connected and received a frame "
                     "using format ID: "
                  << i << std::endl;
        camera = std::move(temp_camera);
        break;
      }
    }

    std::cout
        << "    FAILURE: Could not initialize or get frame with this format."
        << std::endl;
  }

  if (!camera) {
    std::cerr << "ERROR: Failed to connect to camera with any available format."
              << std::endl;
    Cap_releaseContext(ctx);
    return -1;
  }

  cv::Mat frame;

  std::cout << "\n--- Starting Capture Loop (Press Ctrl+C to quit) ---\n"
            << std::endl;

  while (true) {
    // Start the timer right before we ask for a frame.
    auto start_time = std::chrono::steady_clock::now();

    std::chrono::time_point<std::chrono::steady_clock> capture_timestamp;

    if (camera->getFrame(frame, capture_timestamp)) {
      // Stop the timer the moment get_frame() returns successfully.
      auto end_time = std::chrono::steady_clock::now();

      // Calculate the time spent waiting inside get_frame().
      std::chrono::duration<double, std::milli> wait_time =
          end_time - start_time;

      // Print the result to the console.
      std::cout << "Frame Wait Time: " << std::fixed << std::setprecision(2)
                << wait_time.count() << " ms" << std::endl;
    }
  }

  // Clean up resources.
  Cap_releaseContext(ctx);

  std::cout << "--- Test Program Finished ---" << std::endl;
  return 0;
}

int main() { camTest(); }