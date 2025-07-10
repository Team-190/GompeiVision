#pragma once

#include <httplib.h>  // The simple C++ HTTP server library

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "capture/Camera.h"
#include "cscore_cv.h"

class Pipeline {
 public:
  /**
   * @brief Constructs the Pipeline worker process.
   * @param hardware_id The unique hardware ID of the camera to manage.
   * @param role The functional role of this camera.
   * @param web_port The unique network port for this worker's web server.
   */
  Pipeline(const std::string& hardware_id, const std::string& role,
           int web_port);
  ~Pipeline();

  /**
   * @brief Starts the worker's threads (capture loop and web server).
   */
  void start();

  /**
   * @brief Signals all threads to stop and joins them.
   */
  void stop();

 private:
  // --- Thread Entry Points ---
  void capture_loop() const;
  void server_loop();
  // ... other thread placeholders ...

  // --- Member Variables ---

  // Identifiers
  std::string m_hardware_id;
  std::string m_role;
  int m_web_port;

  // Main Camera Object
  std::unique_ptr<Camera> m_camera;

  // Library Contexts
  CapContext m_cap_ctx = nullptr;
  struct udev* m_udev_ctx = nullptr;

  // Web Server and Streaming
  httplib::Server m_server;
  cs::MjpegServer m_mjpeg_server;
  CS_Source m_video_source_handle = 0;  // Use the C-style handle

  // Threads
  std::thread m_capture_thread;
  std::thread m_server_thread;
  // ... other threads for processing ...

  // Control Flag
  std::atomic<bool> m_is_running = false;
};
