#include "pipeline/Pipeline.h"

#include <libudev.h>

#include <chrono>
#include <iostream>
#include <opencv2/imgproc.hpp>

#include "capture/Camera.h"

// Helper function to find the best format (can be expanded later)
static CapFormatID find_optimal_format(const CapContext ctx,
                                       const CapDeviceID dev_index) {
  const int num_formats = Cap_getNumFormats(ctx, dev_index);
  for (int i = 0; i < num_formats; ++i) {
    if (const CapStream stream = Cap_openStream(ctx, dev_index, i);
        stream >= 0) {
      Cap_closeStream(ctx, stream);
      return i;
    }
  }
  return 0;  // Fallback to default
}

Pipeline::Pipeline(const std::string& hardware_id, const std::string& role,
                   const int web_port)
    : m_hardware_id(hardware_id),
      m_role(role),
      m_web_port(web_port),
      m_mjpeg_server(role + "_stream", web_port) {
  std::cout << "[" << m_role << "] Initializing pipeline..." << std::endl;

  // --- cscore Initialization (using C++ functions) ---
  CS_Status status = 0;

  // 1. Create a VideoMode object with the desired properties.
  const cs::VideoMode video_mode(cs::VideoMode::kMJPEG, 640, 480, 30);

  // 2. Create the video source using the C++ function, which returns a C-style
  // handle.
  m_video_source_handle =
      cs::CreateRawSource("cv_" + role, true, video_mode, &status);
  cs::SetSinkSource(m_mjpeg_server.GetHandle(), m_video_source_handle, &status);
  std::cout << "[" << m_role << "] MJPEG stream available at port "
            << m_web_port << std::endl;

  // 3. Set the MjpegServer's source using the C-style function.
  // This bypasses the protected constructor issue by using the raw handles
  // directly.
  cs::SetSinkSource(m_mjpeg_server.GetHandle(), m_video_source_handle, &status);

  std::cout << "[" << m_role << "] MJPEG stream available at port "
            << m_web_port << std::endl;

  // --- Camera Initialization ---
  m_cap_ctx = Cap_createContext();
  m_udev_ctx = udev_new();

  if (!m_cap_ctx || !m_udev_ctx) {
    std::cerr << "[" << m_role << "] ERROR: Failed to create library contexts."
              << std::endl;
    return;
  }

  int device_index = -1;
  const int device_count = Cap_getDeviceCount(m_cap_ctx);
  for (int i = 0; i < device_count; ++i) {
    if (const char* id_ptr = Cap_getDeviceUniqueID(m_cap_ctx, i);
        id_ptr && m_hardware_id == id_ptr) {
      device_index = i;
      break;
    }
  }

  if (device_index == -1) {
    std::cerr << "[" << m_role
              << "] ERROR: Could not find our assigned device ID: "
              << m_hardware_id << std::endl;
    return;
  }

  CapFormatID format_id = find_optimal_format(m_cap_ctx, device_index);
  m_camera = std::make_unique<Camera>(m_cap_ctx, device_index, format_id,
                                      m_hardware_id);

  if (!m_camera || !m_camera->isConnected()) {
    std::cerr << "[" << m_role
              << "] ERROR: Failed to create or connect Camera object."
              << std::endl;
  }
}

Pipeline::~Pipeline() {
  std::cout << "[" << m_role << "] Shutting down pipeline..." << std::endl;
  stop();

  if (m_cap_ctx) Cap_releaseContext(m_cap_ctx);
  if (m_udev_ctx) udev_unref(m_udev_ctx);
}

void Pipeline::start() {
  if (m_is_running) return;
  std::cout << "[" << m_role << "] Starting all threads..." << std::endl;
  m_is_running = true;

  m_capture_thread = std::thread(&Pipeline::capture_loop, this);
  m_server_thread = std::thread(&Pipeline::server_loop, this);
}

void Pipeline::stop() {
  if (!m_is_running) return;
  m_is_running = false;

  m_server.stop();  // Stop the httplib server

  if (m_capture_thread.joinable()) m_capture_thread.join();
  if (m_server_thread.joinable()) m_server_thread.join();

  std::cout << "[" << m_role << "] All threads stopped." << std::endl;
}

void Pipeline::capture_loop() const {
  std::cout << "[" << m_role << "] Capture thread started." << std::endl;
  cv::Mat frame;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;

  while (m_is_running) {
    if (m_camera && m_camera->getFrame(frame, timestamp)) {
      WPI_RawFrame raw_frame;
      raw_frame.width = frame.cols;
      raw_frame.height = frame.rows;
      raw_frame.pixelFormat = WPI_PIXFMT_BGR;
      raw_frame.size = frame.total() * frame.elemSize();
      raw_frame.data = frame.data;

      CS_Status status = 0;
      cs::PutSourceFrame(m_video_source_handle, raw_frame, &status);
    } else {
      // Wait a bit if the camera is disconnected or fails to get a frame
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

void Pipeline::server_loop() {
  std::cout << "[" << m_role << "] Web server thread started." << std::endl;

  // Define a simple root endpoint
  m_server.Get("/", [this](const httplib::Request&, httplib::Response& res) {
    const std::string html_content = "<h1>Pipeline Worker: " + this->m_role +
                                     "</h1>"
                                     "<p>Streaming on port " +
                                     std::to_string(this->m_web_port) + "</p>";
    res.set_content(html_content, "text/html");
  });

  // The MJPEG stream is handled by the cscore MjpegServer automatically.
  // This server just needs to run to handle other future API calls.
  m_server.listen("0.0.0.0", m_web_port);
  std::cout << "[" << m_role << "] Web server stopped." << std::endl;
}

#include "openpnp-capture.h"