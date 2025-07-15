#pragma once

#include <httplib.h>
#include <libudev.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "calibrator/CalibrationSession.h"
#include "capture/Camera.h"
#include "cscore_cv.h"
#include "detector/FiducialDetector.h"
#include "util/QueuedFiducialData.h"
#include "util/QueuedFrame.h"
#include "util/ThreadSafeQueue.h"

struct CapContextDeleter {
  void operator()(CapContext ctx) const {
    if (ctx) Cap_releaseContext(ctx);
  }
};

struct UdevContextDeleter {
  void operator()(struct udev* ctx) const {
    if (ctx) udev_unref(ctx);
  }
};

static CapFormatID find_optimal_format(const CapContext ctx,
                                       const CapDeviceID dev_index) {
  const int num_formats = Cap_getNumFormats(ctx, dev_index);
  if (num_formats <= 0) {
    std::cerr << "[ERROR] No camera formats found for device " << dev_index
              << std::endl;
    return 0;
  }

  CapFormatID best_format_id = 0;
  uint32_t max_data = 0;
  uint32_t max_bpp = 0;

  for (int i = 0; i < num_formats; ++i) {
    CapFormatInfo info;
    if (Cap_getFormatInfo(ctx, dev_index, i, &info) == CAPRESULT_OK) {
      if (const uint32_t current_data = info.width * info.height * info.fps;
          current_data > max_data) {
        max_data = current_data;
        best_format_id = i;
      } else if (current_data == max_data) {
        if (info.bpp > max_bpp) {
          max_bpp = info.bpp;
          best_format_id = i;
        }
      }
    }
  }
  return best_format_id;
}

class Pipeline {
 public:
  Pipeline(const std::string& hardware_id, const std::string& role,
           int stream_port, int control_port);
  ~Pipeline();

  void start();
  void stop();

 private:
  void capture_loop();
  void apriltag_detector_loop();
  void pose_estimator_loop();
  void networktables_loop();
  void server_loop();

  std::map<int, frc::Pose3d> m_field;

  std::string m_hardware_id;
  std::string m_role;
  int m_control_port;
  int m_stream_port;
  std::mutex m_role_mutex;

  int m_stream_width = 0;
  int m_stream_height = 0;

  std::unique_ptr<void, CapContextDeleter> m_cap_ctx;
  std::unique_ptr<struct udev, UdevContextDeleter> m_udev_ctx;
  std::unique_ptr<Camera> m_camera;

  httplib::Server m_server;
  cs::MjpegServer m_mjpeg_server;
  std::unique_ptr<cs::CvSource> m_cv_source;

  // Calibration state
  std::unique_ptr<CalibrationSession> m_calibration_session;
  std::atomic<bool> m_capture_next_frame_for_calib{false};
  std::mutex m_calibration_mutex;

  // --- calibration worker ---
  std::thread m_calibration_worker_thread;
  std::atomic<bool> m_is_calibrating{false};
  std::string m_calibration_status_message;
  std::mutex m_calibration_status_mutex;

  // --- Detectors ---
  FiducialDetector m_AprilTagDetector;

  // --- Pipeline Data Flow ---
  ThreadSafeQueue<QueuedFrame> m_frame_queue;
  ThreadSafeQueue<FiducialImageObservation> m_apriltag_detector_result_queue;
  ThreadSafeQueue<AprilTagResult> m_estimated_poses;

  // Threads & Control
  std::thread m_capture_thread;
  std::thread m_apriltag_detector_thread;
  std::thread m_pose_estimator_thread;
  std::thread m_networktables_thread;
  std::thread m_server_thread;
  std::atomic<bool> m_is_running{false};
};