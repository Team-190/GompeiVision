#include "capture/Camera.h"

#include <openpnp-capture.h>

#include <iostream>

Camera::Camera(const CapContext context, const CapDeviceID deviceIndex,
               const CapFormatID deviceFormat, const std::string& hardwareID)
    : hardwareID(hardwareID),
      deviceIndex(deviceIndex),
      deviceFormat(deviceFormat),
      context(context) {
  logInfo("Initializing...");
  if (!context) {
    logError("Context is null. Cannot initialize capture.");
    return;
  }
  openStream();
}

Camera::~Camera() {
  logInfo("Releasing...");
  closeStream();
}

bool Camera::getFrame(
    cv::Mat& frame,
    std::chrono::time_point<std::chrono::steady_clock>& timestamp) {
  if (!isConnected()) {
    return false;
  }

  // --- KEY CHANGE ---
  // We now directly call Cap_captureFrame, as we've proven that
  // Cap_hasNewFrame is unreliable for this camera driver.
  // Cap_captureFrame will block until a frame is ready.

  // Ensure the output Mat has the correct size and type.
  if (frame.rows != height || frame.cols != width || frame.type() != CV_8UC3) {
    frame.create(height, width, CV_8UC3);
  }

  timestamp = std::chrono::steady_clock::now();

  const CapResult result = Cap_captureFrame(context, stream, frame.data,
                                            frame.total() * frame.elemSize());

  if (result != CAPRESULT_OK) {
    logError("Failed to capture frame. Assuming disconnection.");
    closeStream();
    return false;
  }

  return true;
}

bool Camera::setExposure(const int value) const {
  if (!isConnected()) return false;
  logInfo("Setting exposure to " + std::to_string(value));
  const CapResult result =
      Cap_setProperty(context, stream, CAPPROPID_EXPOSURE, value);
  if (result != CAPRESULT_OK) {
    logError("Failed to set exposure.");
    return false;
  }
  return true;
}

bool Camera::setBrightness(const int value) const {
  if (!isConnected()) return false;
  logInfo("Setting brightness to " + std::to_string(value));
  const CapResult result =
      Cap_setProperty(context, stream, CAPPROPID_BRIGHTNESS, value);
  if (result != CAPRESULT_OK) {
    logError("Failed to set brightness.");
    return false;
  }
  return true;
}

bool Camera::getExposure(int& value) const {
  if (!isConnected()) return false;
  int32_t out_value = 0;
  const CapResult result =
      Cap_getProperty(context, stream, CAPPROPID_EXPOSURE, &out_value);
  if (result == CAPRESULT_OK) {
    value = out_value;
    return true;
  }
  return false;
}

bool Camera::getBrightness(int& value) const {
  if (!isConnected()) return false;
  int32_t out_value = 0;
  const CapResult result =
      Cap_getProperty(context, stream, CAPPROPID_BRIGHTNESS, &out_value);
  if (result == CAPRESULT_OK) {
    value = out_value;
    return true;
  }
  return false;
}

bool Camera::isConnected() const {
  // A more robust check might involve checking the stream status with the
  // library
  return connected && stream >= 0;
}

void Camera::setRole(const std::string& role) { deviceRole = role; }

const std::string& Camera::getRole() const { return deviceRole; }

const std::string& Camera::getHardwareID() const { return hardwareID; }

void Camera::openStream() {
  logInfo("Attempting to open device stream...");

  // For simplicity, we'll open the default stream format (formatID = 0).
  // A more advanced implementation might iterate through available formats.
  stream = Cap_openStream(context, deviceIndex, deviceFormat);
  if (stream < 0) {
    logError("Failed to open stream.");
    return;
  }

  // After opening the stream, get the format information to know the frame
  // size.
  CapFormatInfo format_info;
  if (Cap_getFormatInfo(context, deviceIndex, 0, &format_info) ==
      CAPRESULT_OK) {
    width = format_info.width;
    height = format_info.height;
  } else {
    logError("Failed to get format info after opening stream.");
    closeStream();  // Close the stream if we can't get its info
    return;
  }

  // --- NEW: Disable Auto-Exposure ---
  logInfo("Disabling auto-exposure...");
  // We call Cap_setAutoProperty with the exposure property ID and a value of 0
  // to turn it off.
  CapResult result =
      Cap_setAutoProperty(context, stream, CAPPROPID_EXPOSURE, 0);
  if (result != CAPRESULT_OK) {
    // This is not a fatal error. Some cameras may not support this control.
    logError(
        "Warning: Could not disable auto-exposure. The camera may not support "
        "this setting.");
  }
  // --- End of new logic ---

  connected = true;
  logInfo("Successfully opened stream.");
}

void Camera::closeStream() {
  if (stream >= 0) {
    Cap_closeStream(context, stream);
    stream = -1;
  }
  connected = false;
  // We DO NOT release the context here, as it is owned by the CameraManager.
}

void Camera::logInfo(const std::string& message) const {
  std::cout << "[INFO] Camera (" << hardwareID << "): " << message << std::endl;
}

void Camera::logError(const std::string& message) const {
  std::cerr << "[ERROR] Camera (" << hardwareID << "): " << message
            << std::endl;
}