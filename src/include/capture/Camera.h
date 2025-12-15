#pragma once

#include <chrono>
#include <filesystem>
#include <opencv2/videoio.hpp>  // OpenCV's header for video I/O operations
#include <string>

/**
 * @class Camera
 * @brief Represents and manages a single physical USB camera using OpenCV.
 *
 * This class provides a high-level interface for a camera device.
 * It uses OpenCV's VideoCapture to open a stream, capture frames, and
 * manage camera settings like exposure and brightness. It also includes
 * logic to automatically reconnect if the camera stream is lost.
 */
class Camera {
 public:
  /**
   * @brief Constructs a Camera object and attempts to open the device.
   * @param device_path The system path to the camera (e.g., /dev/video0).
   * @param hardwareID A unique, persistent ID for logging and identification.
   * @param width The desired frame width for the camera stream.
   * @param height The desired frame height for the camera stream.
   */
  Camera(const std::string& device_path, const std::string& hardwareID,
         int width, int height);

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
   * @brief Captures the latest frame from the camera. If disconnected, it
   *        will attempt to reconnect.
   * @param[out] frame An OpenCV Mat object to store the captured frame.
   * @param[out] timestamp A time_point to store the capture timestamp.
   * @return True if a new frame was successfully captured, false otherwise.
   */
  bool getFrame(cv::Mat& frame,
                std::chrono::time_point<std::chrono::system_clock>& timestamp);

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
   * @brief Checks if the camera is currently connected and streaming.
   * @return True if the camera stream is open, false otherwise.
   */
  bool isConnected() const;

  /**
   * @brief Gets the speed of the camera.
   * @return The USB transfer speed of the camera (mbps)
   */
  double getSpeed() const;

  /**
   * @brief Attempts to release and re-open the camera stream.
   * @return True if the camera was successfully reconnected, false otherwise.
   */
  bool attemptReconnect();

  /**
   * @brief Gets the pixel format of the camera stream.
   * @return A string representing the fourcc code of the pixel format.
   */
  std::string getFormat();

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

  /**
   * @brief Applies all stored camera configurations (resolution, exposure,
   * etc). Called on initial setup and after a reconnect.
   */
  void configureCamera();

  // Member Variables
  std::string m_hardwareID;
  std::string m_device_path;

  // Frame dimensions, retrieved from the camera after opening it.
  int m_width = 0;
  int m_height = 0;

  // The core OpenCV camera object.
  cv::VideoCapture m_capture;
  bool m_is_connected = true;

  // Store requested settings to re-apply on reconnect
  int m_req_width;
  int m_req_height;
  int m_last_exposure = -1;    // Using -1 to indicate not set
  int m_last_brightness = -1;  // Using -1 to indicate not set
};
