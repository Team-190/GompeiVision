#include "capture/Camera.h"  // Corresponds to the new OpenCV-based header

#include <iostream>

// Constructor: Initializes and opens the camera using OpenCV
Camera::Camera(const int deviceIndex, const std::string& hardwareID,
               const int width, const int height, const bool useMJPG)
    : m_hardwareID(hardwareID), m_deviceIndex(deviceIndex) {
  logInfo("Initializing with OpenCV backend...");

  // Open the camera stream using the specified device index and the Video4Linux
  // backend. Using V4L2 is often more reliable on Linux for setting properties.
  m_capture.open(m_deviceIndex, cv::CAP_V4L2);

  if (!m_capture.isOpened()) {
    logError("Failed to open camera stream at index " +
             std::to_string(m_deviceIndex));
    return;
  }

  logInfo("Camera stream opened successfully.");

  // --- Configure Camera Properties ---

  // Set the desired codec (FourCC)
  if (useMJPG) {
    // Request MJPG format.
    if (m_capture.set(cv::CAP_PROP_FOURCC,
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'))) {
      logInfo("Successfully set format to MJPG.");
    } else {
      logError("Warning: Failed to set format to MJPG.");
    }
  }

  // Set the desired frame dimensions.
  if (m_capture.set(cv::CAP_PROP_FRAME_WIDTH, width)) {
    logInfo("Set frame width to " + std::to_string(width));
  } else {
    logError("Warning: Failed to set frame width.");
  }

  if (m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height)) {
    logInfo("Set frame height to " + std::to_string(height));
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

  // Log the actual format the camera has settled on.
  int fourcc = static_cast<int>(m_capture.get(cv::CAP_PROP_FOURCC));
  if (fourcc != 0) {
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
    std::chrono::time_point<std::chrono::steady_clock>& timestamp) {
  if (!isConnected()) {
    return false;
  }

  // Atomically grab and retrieve the frame.
  if (m_capture.read(frame)) {
    timestamp = std::chrono::steady_clock::now();
    return true;  // Frame captured successfully.
  }

  logError("Failed to capture frame. The camera may have been disconnected.");
  return false;
}

// Sets the camera's exposure property
bool Camera::setExposure(int value) {
  if (!isConnected()) return false;
  logInfo("Setting exposure to " + std::to_string(value));
  // cv::VideoCapture::set returns true on success
  if (!m_capture.set(cv::CAP_PROP_EXPOSURE, static_cast<double>(value))) {
    logError("Failed to set exposure.");
    return false;
  }
  return true;
}

// Sets the camera's brightness property
bool Camera::setBrightness(int value) {
  if (!isConnected()) return false;
  logInfo("Setting brightness to " + std::to_string(value));
  if (!m_capture.set(cv::CAP_PROP_BRIGHTNESS, static_cast<double>(value))) {
    logError("Failed to set brightness.");
    return false;
  }
  return true;
}

// Gets the camera's current exposure value
bool Camera::getExposure(int& value) const {
  if (!isConnected()) return false;
  value = static_cast<int>(m_capture.get(cv::CAP_PROP_EXPOSURE));
  return true;  // .get() doesn't have a failure return, so we assume success if
                // connected.
}

// Gets the camera's current brightness value
bool Camera::getBrightness(int& value) const {
  if (!isConnected()) return false;
  value = static_cast<int>(m_capture.get(cv::CAP_PROP_BRIGHTNESS));
  return true;
}

// Checks if the camera is open and ready for use
bool Camera::isConnected() const { return m_capture.isOpened(); }

// Gets the hardware ID of the camera
const std::string& Camera::getHardwareID() const { return m_hardwareID; }

// Gets the frame width of the stream
int Camera::getWidth() const { return m_width; }

// Gets the frame height of the stream
int Camera::getHeight() const { return m_height; }

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
