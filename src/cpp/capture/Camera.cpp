#include "capture/Camera.h"

#include <fstream>
#include <iostream>
#include <thread>

// Constructor: Initializes and opens the camera using OpenCV
Camera::Camera(const std::string& device_path, const std::string& hardwareID,
               const int width, const int height)
    : m_hardwareID(hardwareID),
      m_device_path(device_path),
      m_req_width(width),
      m_req_height(height) {
  logInfo("Initializing with OpenCV backend...");
  // Initial camera connection and configuration
  attemptReconnect();
}

// Destructor: Releases the camera resource
Camera::~Camera() {
  logInfo("Releasing camera...");
  if (m_capture.isOpened()) {
    m_capture.release();  // This is crucial to free the camera for other
                          // applications.
  }
}

// Captures a new frame from the camera
bool Camera::getFrame(
    cv::Mat& frame,
    std::chrono::time_point<std::chrono::system_clock>& timestamp) {
  if (!isConnected()) {
    m_is_connected = false;
    return false;
  }

  // Atomically grab and retrieve the frame.
  if (m_capture.read(frame)) {
    timestamp = std::chrono::system_clock::now();
    m_is_connected = true;
    return true;  // Frame captured successfully.
  }

  logError("Failed to capture frame. The camera may have been disconnected.");
  m_is_connected = false;
  return false;
}

double Camera::getSpeed() const {
  int usb_speed = -1;
  if (!m_device_path.empty()) {
    try {
      auto node = std::filesystem::path(m_device_path).filename().string();
      auto dev = std::filesystem::canonical("/sys/class/video4linux/" + node +
                                            "/device");

      while (dev.has_parent_path()) {
        auto sp = dev.string() + "/speed";
        if (std::filesystem::exists(sp)) {
          std::ifstream f(sp);
          int s = -1;
          if (f >> s) usb_speed = s;
          break;
        }
        dev = dev.parent_path();
      }
    } catch (std::exception& e) {
      std::cerr << "Failed to read USB speed: " << e.what() << std::endl;
    }
  }
  return usb_speed;
}

// Sets the camera's exposure property
bool Camera::setExposure(const int value) {
  if (!isConnected()) {
    m_last_exposure =
        value;  // Store even if not connected, will apply on reconnect
    return false;
  }
  logInfo("Setting exposure to " + std::to_string(value));
  if (!m_capture.set(cv::CAP_PROP_EXPOSURE, value)) {
    logError("Failed to set exposure.");
    return false;
  }
  m_last_exposure = value;
  return true;
}

// Sets the camera's brightness property
bool Camera::setBrightness(const int value) {
  if (!isConnected()) {
    m_last_brightness =
        value;  // Store even if not connected, will apply on reconnect
    return false;
  }
  logInfo("Setting brightness to " + std::to_string(value));
  if (!m_capture.set(cv::CAP_PROP_BRIGHTNESS, value)) {
    logError("Failed to set brightness.");
    return false;
  }
  m_last_brightness = value;
  return true;
}

// Checks if the camera is open and ready for use
bool Camera::isConnected() const { return m_capture.isOpened(); }

std::string Camera::getFormat() {
  if (!isConnected()) {
    return "";
  }
  int fourcc = static_cast<int>(m_capture.get(cv::CAP_PROP_FOURCC));
  if (fourcc == 0) {
    return "";
  }
  std::string formatStr;
  formatStr += (fourcc & 0XFF);
  formatStr += ((fourcc >> 8) & 0XFF);
  formatStr += ((fourcc >> 16) & 0XFF);
  formatStr += ((fourcc >> 24) & 0XFF);
  return formatStr;
}

// Helper for logging informational messages
void Camera::logInfo(const std::string& message) const {
  std::cout << "[INFO] Camera (" << m_hardwareID << "): " << message
            << std::endl;
}

// Helper for logging error messages
void Camera::logError(const std::string& message) const {
  std::cerr << "[ERROR] Camera (" << m_hardwareID << "): " << message
            << std::endl;
}

void Camera::configureCamera() {
  if (!m_capture.isOpened()) {
    logError("Cannot configure camera: stream not open.");
    return;
  }

  logInfo("Configuring camera properties...");

  // Set the desired frame dimensions.
  if (m_capture.set(cv::CAP_PROP_FRAME_WIDTH, m_req_width)) {
    logInfo("Set frame width to " + std::to_string(m_req_width));
  } else {
    logError("Warning: Failed to set frame width.");
  }

  if (m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, m_req_height)) {
    logInfo("Set frame height to " + std::to_string(m_req_height));
  } else {
    logError("Warning: Failed to set frame height.");
  }

  // --- Disable Auto Properties ---
  // It's crucial to disable auto-exposure and auto-white-balance for consistent
  // imaging. Note: For V4L2, a value of '1' often means 'manual mode' (auto
  // off).
  if (m_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1)) {
    logInfo("Disabled auto-exposure (set to manual mode).");
  } else {
    logError("Warning: Could not disable auto-exposure.");
  }

  // Apply last known exposure and brightness settings
  if (m_last_exposure != -1) {
    setExposure(m_last_exposure);
  }
  if (m_last_brightness != -1) {
    setBrightness(m_last_brightness);
  }

  // Log the actual format the camera has settled on.
  if (const int fourcc = static_cast<int>(m_capture.get(cv::CAP_PROP_FOURCC));
      fourcc != 0) {
    // Decode the FourCC integer into a human-readable string.
    std::string formatStr;
    formatStr += (fourcc & 0XFF);
    formatStr += ((fourcc >> 8) & 0XFF);
    formatStr += ((fourcc >> 16) & 0XFF);
    formatStr += ((fourcc >> 24) & 0XFF);
    logInfo("Actual camera format in use: " + formatStr);
  } else {
    logError("Could not retrieve camera format.");
  }

  // Retrieve the actual frame dimensions, as the camera might not support the
  // exact requested size.
  m_width = static_cast<int>(m_capture.get(cv::CAP_PROP_FRAME_WIDTH));
  m_height = static_cast<int>(m_capture.get(cv::CAP_PROP_FRAME_HEIGHT));

  logInfo("Actual stream resolution: " + std::to_string(m_width) + "x" +
          std::to_string(m_height));
}

bool Camera::attemptReconnect() {
  if (m_capture.isOpened()) {
    m_capture.release();
    logInfo("Released existing camera connection.");
  }

  logInfo("Attempting to open camera stream at path " + m_device_path);
  // Open the camera stream using the specified device path and the Video4Linux
  // backend. Using V4L2 is often more reliable on Linux for setting properties.
  m_capture.open(m_device_path, cv::CAP_V4L2);

  if (!m_capture.isOpened()) {
    logError("Failed to open camera stream at path " + m_device_path +
             ". Retrying in 1 second...");
    std::this_thread::sleep_for(
        std::chrono::seconds(1));  // Wait before retrying
    return false;
  }

  logInfo("Camera stream opened successfully.");
  configureCamera();
  return true;
}
