#pragma once

#include <httplib.h>
#include <libudev.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "../io/OutputPublisher.h"
#include "capture/Camera.h"
#include "cscore_cv.h"
#include "detector/FiducialDetector.h"
#include "io/ConfigInterface.h"
#include "io/FieldInterface.h"
#include "util/QueuedFiducialData.h"
#include "util/ThreadSafeQueue.h"

struct UdevContextDeleter {
  void operator()(struct udev* ctx) const {
    if (ctx) udev_unref(ctx);
  }
};

class Pipeline {
 public:
  Pipeline(const std::string& device_path, const std::string& hardware_id,
           const int stream_port);
  ~Pipeline();

  void start();
  void during();
  void stop();

  bool isRunning() const;

 private:
  void processing_loop();
  void networktables_loop();

  std::atomic<bool> m_is_running{false};

  FieldInterface m_field;

  std::string m_hardware_id;
  std::string m_role;
  int m_stream_port;
  std::mutex m_role_mutex;

  // Cached settings to prevent unnecessary reloads
  int m_active_width = 0;
  int m_active_height = 0;
  int m_active_exposure = 0;
  int m_active_gain = 0;
  cv::Mat m_camera_matrix;
  cv::Mat m_dist_coeffs;
  bool m_intrinsics_loaded = false;

  std::unique_ptr<Camera> m_camera;

  std::unique_ptr<cs::MjpegServer> m_mjpeg_server;
  std::unique_ptr<cs::CvSource> m_cv_source;

  // --- Detectors ---
  FiducialDetector m_AprilTagDetector;

  // --- Pipeline Data Flow ---
  ThreadSafeQueue<AprilTagResult> m_estimated_poses;

  // Threads & Control
  std::thread m_processing_thread;
  std::thread m_networktables_thread;

  // --- NetworkTables Interface ---
  std::unique_ptr<ConfigInterface> m_config_interface;
  std::unique_ptr<OutputPublisher> m_output_publisher;
};
