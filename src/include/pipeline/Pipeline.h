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

class Pipeline {
 public:
  Pipeline(const std::string& hardware_id, const std::string& role,
           int stream_port, int control_port);
  ~Pipeline();

  void start();
  void stop();

 private:
  void capture_loop();
  void server_loop();

  void start_calibration_session(int squares_x, int squares_y,
                                 float square_length_m, float marker_length_m);
  void add_calibration_frame();

  // --- NEW: Worker function for background processing ---
  void async_finish_calibration(const std::string& output_file);

  // Member Variables
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

  // --- NEW: State for the asynchronous calibration worker ---
  std::thread m_calibration_worker_thread;
  std::atomic<bool> m_is_calibrating{false};
  std::string m_calibration_status_message;
  std::mutex m_calibration_status_mutex;

  // Threads & Control
  std::thread m_capture_thread;
  std::thread m_server_thread;
  std::atomic<bool> m_is_running{false};
};