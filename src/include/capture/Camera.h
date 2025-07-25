#pragma once

#include <chrono>
#include <opencv2/videoio.hpp>  // OpenCV's header for video I/O operations
#include <string>

/**
 * @class Camera
 * @brief Represents and manages a single physical USB camera using OpenCV.
 *
 * This class provides a high-level interface for a camera device.
 * It uses OpenCV's VideoCapture to open a stream, capture frames, and
 * manage camera settings like exposure and brightness.
 */
class Camera {
 public:
  /**
   * @brief Constructs a Camera object and attempts to open the device.
   * @param deviceIndex The numerical index of the camera (e.g., 0 for
   * /dev/video0).
   * @param hardwareID A unique, persistent ID for logging and identification.
   * @param width The desired frame width for the camera stream.
   * @param height The desired frame height for the camera stream.
   * @param useMJPG Set to true to request MJPG format, false for default/YUYV.
   */
  Camera(int deviceIndex, const std::string& hardwareID, int width, int height,
         bool useMJPG);

  /**
   * @brief Destructor that ensures the camera stream is properly released.
   */
  ~Camera();

  // Disable copy and move semantics to prevent issues with resource ownership.
  Camera(const Camera&) = delete;
  Camera& operator=(const Camera&) = delete;
  Camera(Camera&&) = delete;
  Camera& operator=(Camera&&) = delete;

  /**
   * @brief Captures the latest frame from the camera if one is available.
   * @param[out] frame An OpenCV Mat object to store the captured frame.
   * @param[out] timestamp A time_point to store the capture timestamp.
   * @return True if a new frame was successfully captured, false otherwise.
   */
  bool getFrame(cv::Mat& frame,
                std::chrono::time_point<std::chrono::steady_clock>& timestamp);

  /**
   * @brief Sets the exposure value for the camera.
   * @param value The desired exposure value (OpenCV uses its own scale).
   * @return True on success, false on failure.
   */
  bool setExposure(int value);

  /**
   * @brief Sets the brightness value for the camera.
   * @param value The desired brightness value (range typically 0-255).
   * @return True on success, false on failure.
   */
  bool setBrightness(int value);

  /**
   * @brief Gets the current exposure value from the camera.
   * @param[out] value A reference to store the retrieved value.
   * @return True on success, false on failure.
   */
  bool getExposure(int& value) const;

  /**
   * @brief Gets the current brightness value from the camera.
   * @param[out] value A reference to store the retrieved value.
   * @return True on success, false on failure.
   */
  bool getBrightness(int& value) const;

  /**
   * @brief Checks if the camera is currently connected and streaming.
   * @return True if the camera stream is open, false otherwise.
   */
  bool isConnected() const;

  /**
   * @brief Gets the hardware ID this camera object was initialized with.
   * @return A constant reference to the hardware ID string.
   */
  const std::string& getHardwareID() const;

  /**
   * @brief Gets the width of the camera stream.
   * @return The width of the camera stream in pixels.
   */
  int getWidth() const;

  /**
   * @brief Gets the height of the camera stream.
   * @return The height of the camera stream in pixels.
   */
  int getHeight() const;

 private:
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
  std::string m_hardwareID;
  int m_deviceIndex;

  // Frame dimensions, retrieved from the camera after opening it.
  int m_width = 0;
  int m_height = 0;

  // The core OpenCV camera object.
  // This object handles all interaction with the physical camera device.
  cv::VideoCapture m_capture;
};
