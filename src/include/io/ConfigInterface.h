#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/StringTopic.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <string>
#include <thread>

class ConfigInterface {
 public:
  explicit ConfigInterface(const std::string&,
                           const nt::NetworkTableInstance& nt_inst);
  ~ConfigInterface();

  void waitForInitialization();
  void update();

  // --- State Getters ---
  bool isSetupMode() const;

  // --- State Setters ---
  void setSetupMode(bool setupMode);

  // --- Configuration Getters ---
  std::string getRole() const;
  cv::Mat getCameraMatrix() const;
  cv::Mat getDistortionCoeffs() const;
  int getExposure() const;
  int getGain() const;
  int getWidth() const;
  int getHeight() const;
  bool getCompressed() const;
  int getModelIndex() const;

 private:
  /**
   * @brief The target function for the initialization thread. Polls for
   * initial values.
   */
  void initializationThreadFunc();

  // --- NetworkTables Handles ---
  std::shared_ptr<nt::NetworkTable> m_table;
  std::shared_ptr<nt::NetworkTable> m_configTable;
  std::shared_ptr<nt::NetworkTable> m_outputTable;

  // Subscribers for each configuration parameter
  nt::BooleanSubscriber m_setupModeSub;
  nt::StringSubscriber m_roleSub;
  nt::DoubleArraySubscriber m_cameraMatrixSub;
  nt::DoubleArraySubscriber m_distCoeffsSub;
  nt::IntegerSubscriber m_exposureSub;
  nt::IntegerSubscriber m_gainSub;
  nt::IntegerSubscriber m_widthSub;
  nt::IntegerSubscriber m_heightSub;
  nt::BooleanSubscriber m_compressedSub;
  nt::IntegerSubscriber m_modelIndexSub;

  // --- In-Memory Configuration Cache ---
  // These members hold the last valid values received from NetworkTables.
  bool m_setupMode;
  std::string m_role;
  cv::Mat m_cameraMatrix;
  cv::Mat m_distortionCoeffs;
  int m_exposure;
  int m_gain;
  int m_width;
  int m_height;
  bool m_compressed;
  int m_modelIndex;

  // --- Threading and Synchronization for Initialization ---
  std::thread m_initThread;
  std::mutex m_initMutex;
  std::condition_variable m_initCv;
  bool m_initialized = false;

  // --- Logging Helpers ---
  static void logInfo(const std::string& message);
  static void logError(const std::string& message);
};
