#pragma once

#include <chrono>
#include <opencv2/core/mat.hpp>
#include <string>

#include "openpnp-capture.h"  // Include the header to get the type definitions

/**
 * @class Camera
 * @brief Represents and manages a single physical USB capture.
 *
 * This class provides a high-level interface for an already-discovered capture.
 * It is given the necessary resources by a CameraManager to open a stream,
 * capture frames, and manage settings.
 */
class Camera {
 public:
  /**
   * @brief Constructs a Camera object using resources provided by a manager.
   * @param context A shared pointer to the openpnp-capture context.
   * @param deviceIndex The specific index of the device to open.
   * @param deviceFormat The specific format of the device to open
   * @param hardwareID The unique, persistent ID for logging and
   * identification.
   */
  Camera(CapContext context, CapDeviceID deviceIndex,
         const CapFormatID deviceFormat, const std::string& hardwareID);

  /**
   * @brief Destructor that ensures the capture stream is properly released.
   */
  ~Camera();

  // Disable copy and move semantics to prevent issues with resource ownership.
  Camera(const Camera&) = delete;
  Camera& operator=(const Camera&) = delete;
  Camera(Camera&&) = delete;
  Camera& operator=(Camera&&) = delete;

  /**
   * @brief Captures the latest frame from the capture if one is available.
   * @param frame An OpenCV Mat object to store the captured frame.
   * @param timestamp
   * @return True if a NEW frame was successfully captured, false otherwise.
   */
  bool getFrame(cv::Mat& frame,
                std::chrono::time_point<std::chrono::steady_clock>& timestamp);

  /**
   * @brief Sets the exposure value for the capture.
   * @param value The desired exposure value.
   * @return True on success, false on failure.
   */
  bool setExposure(int value) const;

  /**
   * @brief Sets the brightness value for the capture.
   * @param value The desired brightness value.
   * @return True on success, false on failure.
   */
  bool setBrightness(int value) const;

  /**
   * @brief Gets the current exposure value from the capture.
   * @param[out] value A reference to store the retrieved value.
   * @return True on success, false on failure.
   */
  bool getExposure(int& value) const;

  /**
   * @brief Gets the current brightness value from the capture.
   * @param[out] value A reference to store the retrieved value.
   * @return True on success, false on failure.
   */
  bool getBrightness(int& value) const;

  /**
   * @brief Checks if the capture is currently connected and streaming.
   * @return True if the capture is connected, false otherwise.
   */
  bool isConnected() const;

  /**
   * @brief Sets the functional role of this capture.
   * @param role The role name (e.g., "front_cam").
   */
  void setRole(const std::string& role);

  /**
   * @brief Gets the functional role of this capture.
   * @return The role name.
   */
  const std::string& getRole() const;

  /**
   * @brief Gets the hardware ID this capture object was initialized with.
   * @return A constant reference to the hardware ID string.
   */
  const std::string& getHardwareID() const;

 private:
  /**
   * @brief Helper function to open the device stream using the stored index.
   */
  void openStream();

  /**
   * @brief Helper function to properly close and release the capture stream.
   */
  void closeStream();

  /**
   * @brief Private helper to log informational messages.
   * @param message The message to log.
   */
  void logInfo(const std::string& message) const;

  /**
   * @brief Private helper to log error messages.
   * @param message The message to log.
   */
  void logError(const std::string& message) const;

  // Member Variables
  std::string hardwareID;
  std::string deviceRole = "unassigned";
  CapDeviceID deviceIndex;
  CapFormatID deviceFormat;
  bool connected = false;

  // Frame dimensions, retrieved once when the stream is opened.
  int width = 0;
  int height = 0;

  // The CameraManager owns the context, this class just uses it.
  CapContext context = nullptr;
  CapStream stream = -1;
};
