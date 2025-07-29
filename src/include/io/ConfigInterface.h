#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <memory>
#include <opencv2/core/mat.hpp>
#include <string>

/**
 * @class ConfigInterface
 * @brief Manages loading and providing configuration parameters from
 * NetworkTables.
 *
 * This class subscribes to a specific NetworkTables table (e.g.,
 * "GompeiVision") to dynamically load and update camera and vision processing
 * settings. It provides a centralized, in-memory cache of these settings for
 * other parts of the application to use.
 *
 * When the "setup_mode" topic is true, this class can also be used to publish
 * new values for the settings, allowing for live tuning.
 */
class ConfigInterface {
  struct LocalConfig {
    std::optional<cv::Mat> cameraMatrix;
    std::optional<cv::Mat> distortionCoeffs;
    std::optional<int> exposure;
    std::optional<int> gain;
    std::optional<int> width;
    std::optional<int> height;
  };

 public:
  /**
   * @brief Constructs the ConfigInterface and initializes NetworkTables
   * subscribers and publishers.
   *
   * On construction, it establishes connections to the relevant NetworkTables
   * topics and performs an initial load of all configuration values.
   */
  explicit ConfigInterface(const std::string&);
  ~ConfigInterface();

  /**
   * @brief Fetches the latest values from all NetworkTables topics.
   *
   * This method should be called periodically (e.g., once per main loop
   * iteration) to ensure the configuration remains up-to-date with any changes
   * published by a client, such as a dashboard or the robot code.
   */
  void update();

  // --- State Getters ---

  /**
   * @brief Checks if the system is currently in setup mode.
   * @return True if setup mode is active, false otherwise.
   */
  bool isSetupMode() const;

  // --- State Setters ---

  /**
   * @brief sets the setup mode
   * @param setupMode The state of the setup mode
   */
  void setSetupMode(bool setupMode);

  // --- Configuration Getters ---

  cv::Mat getCameraMatrix() const;
  cv::Mat getDistortionCoeffs() const;
  int getExposure() const;
  int getGain() const;
  int getWidth() const;
  int getHeight() const;

  // --- Configuration Setters (only publish if in setup mode) ---

  void setConfig(const LocalConfig& config);

 private:
  /**
   * @brief Publishes a new camera matrix to NetworkTables if in setup mode.
   * @param matrix The 3x3 cv::Mat (CV_64F) to publish.
   */
  void setCameraMatrix(const cv::Mat& matrix);

  /**
   * @brief Publishes new distortion coefficients to NetworkTables if in setup
   * mode.
   * @param coeffs The 1xN cv::Mat (CV_64F) of coefficients to publish.
   */
  void setDistortionCoeffs(const cv::Mat& coeffs);

  /**
   * @brief Publishes a new exposure value to NetworkTables if in setup mode.
   * @param exposure The new integer exposure value.
   */
  void setExposure(int exposure);

  /**
   * @brief Publishes a new gain value to NetworkTables if in setup mode.
   * @param gain The new integer gain value.
   */
  void setGain(int gain);

  /**
   * @brief Publishes a new frame width to NetworkTables if in setup mode.
   * @param width The new integer width.
   */
  void setWidth(int width);

  /**
   * @brief Publishes a new frame height to NetworkTables if in setup mode.
   * @param height The new integer height.
   */
  void setHeight(int height);

  // --- NetworkTables Handles ---
  nt::NetworkTableInstance m_ntInst;
  std::shared_ptr<nt::NetworkTable> m_table;

  // Subscribers for each configuration parameter
  nt::BooleanSubscriber m_setupModeSub;
  nt::DoubleArraySubscriber m_cameraMatrixSub;
  nt::DoubleArraySubscriber m_distCoeffsSub;
  nt::IntegerSubscriber m_exposureSub;
  nt::IntegerSubscriber m_gainSub;
  nt::IntegerSubscriber m_widthSub;
  nt::IntegerSubscriber m_heightSub;

  // Publishers for each configuration parameter
  nt::DoubleArrayPublisher m_cameraMatrixPub;
  nt::DoubleArrayPublisher m_distCoeffsPub;
  nt::IntegerPublisher m_exposurePub;
  nt::IntegerPublisher m_gainPub;
  nt::IntegerPublisher m_widthPub;
  nt::IntegerPublisher m_heightPub;

  // --- In-Memory Configuration Cache ---
  // These members hold the last valid values received from NetworkTables.
  bool m_setupMode;
  cv::Mat m_cameraMatrix;
  cv::Mat m_distortionCoeffs;
  int m_exposure;
  int m_gain;
  int m_width;
  int m_height;

  // --- Logging Helpers ---
  static void logInfo(const std::string& message);
  static void logError(const std::string& message);
};