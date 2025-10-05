#include "io/ConfigInterface.h"

#include <chrono>
#include <iostream>
#include <vector>

// --- Constants for NetworkTables Keys ---
// These keys must match the ones used by the client (e.g., Robot Code,
// Dashboard)
namespace nt_keys {
constexpr auto kSetupMode = "setup_mode";
constexpr auto kRole = "role";
constexpr auto kHardwareID = "hardware_id";
constexpr auto kCameraMatrix = "camera_matrix";
constexpr auto kDistCoeffs = "distortion_coefficients";
constexpr auto kExposure = "exposure";
constexpr auto kGain = "gain";
constexpr auto kWidth = "width";
constexpr auto kHeight = "height";
constexpr auto kCompressed = "compressed";
}  // namespace nt_keys

ConfigInterface::ConfigInterface(const std::string& hardwareID,
                                 const nt::NetworkTableInstance& nt_inst) {
  m_table = nt_inst.GetTable("/cameras/" + hardwareID);
  m_configTable = nt_inst.GetTable("/cameras/" + hardwareID + "/config");

  if (m_table) {
    std::cout << "Initializing subscribers" << std::endl;
    // --- Initialize Subscribers ---
    // Subscribe to each topic with a default value. This ensures
    // that if the topic doesn't exist on the server yet, we have
    // a sensible default.
    nt::PubSubOptions options;
    options.periodic = 0.01;
    options.keepDuplicates = true;

    m_setupModeSub = m_configTable->GetBooleanTopic(nt_keys::kSetupMode)
                         .Subscribe(false, options);
    m_roleSub =
        m_configTable->GetStringTopic(nt_keys::kRole).Subscribe("", options);
    m_cameraMatrixSub =
        m_configTable->GetDoubleArrayTopic(nt_keys::kCameraMatrix)
            .Subscribe({}, options);
    m_distCoeffsSub = m_configTable->GetDoubleArrayTopic(nt_keys::kDistCoeffs)
                          .Subscribe({}, options);
    m_exposureSub = m_configTable->GetIntegerTopic(nt_keys::kExposure)
                        .Subscribe(0, options);
    m_gainSub =
        m_configTable->GetIntegerTopic(nt_keys::kGain).Subscribe(0, options);
    m_widthSub =
        m_configTable->GetIntegerTopic(nt_keys::kWidth).Subscribe(0, options);
    m_heightSub =
        m_configTable->GetIntegerTopic(nt_keys::kHeight).Subscribe(0, options);
    m_compressedSub = m_configTable->GetBooleanTopic(nt_keys::kCompressed)
                          .Subscribe(false, options);

    // Start the initialization thread
    m_initThread =
        std::thread(&ConfigInterface::initializationThreadFunc, this);

    logInfo("ConfigInterface initialized for table: " + hardwareID);
  } else {
    logError("ConfigInterface failed to initialize table: " + hardwareID +
             " Network table not found");
  }
}

ConfigInterface::~ConfigInterface() {
  if (m_initThread.joinable()) {
    m_initThread.join();
  }
}

void ConfigInterface::initializationThreadFunc() {
  while (true) {
    update();

    // Check if we have received non-default values.
    // The role and camera matrix are good indicators of initialization.
    if (!m_role.empty() && !m_cameraMatrix.empty()) {
      std::lock_guard<std::mutex> lock(m_initMutex);
      m_initialized = true;
      logInfo("Initial configuration received from NetworkTables.");
      m_initCv.notify_all();  // Notify waiting threads
      return;                 // Exit the thread
    }

    // Wait a bit before polling again to avoid busy-waiting.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void ConfigInterface::waitForInitialization() {
  std::unique_lock<std::mutex> lock(m_initMutex);
  m_initCv.wait(lock, [this] { return m_initialized; });
}

void ConfigInterface::update() {
  m_setupMode = m_setupModeSub.Get();
  m_role = m_roleSub.Get();

  // Update camera matrix from NetworkTables
  std::vector<double> cameraMatrixData = m_cameraMatrixSub.Get();
  if (cameraMatrixData.size() == 9) {
    // Create a Mat header for the data and then clone it to own the data
    m_cameraMatrix = cv::Mat(3, 3, CV_64F, cameraMatrixData.data()).clone();
  }

  // Update distortion coefficients from NetworkTables
  std::vector<double> distCoeffsData = m_distCoeffsSub.Get();
  if (!distCoeffsData.empty()) {
    m_distortionCoeffs =
        cv::Mat(1, distCoeffsData.size(), CV_64F, distCoeffsData.data())
            .clone();
  }

  m_exposure = m_exposureSub.Get();
  m_gain = m_gainSub.Get();
  m_width = m_widthSub.Get();
  m_height = m_heightSub.Get();
  m_compressed = m_compressedSub.Get();
}

// --- State Getters ---

bool ConfigInterface::isSetupMode() const { return m_setupMode; }

// --- State Setters ---

void ConfigInterface::setSetupMode(const bool setupMode) {
  m_setupMode = setupMode;
}

// --- Configuration Getters ---

std::string ConfigInterface::getRole() const { return m_role; }

cv::Mat ConfigInterface::getCameraMatrix() const { return m_cameraMatrix; }

cv::Mat ConfigInterface::getDistortionCoeffs() const {
  return m_distortionCoeffs;
}

int ConfigInterface::getExposure() const { return m_exposure; }

int ConfigInterface::getGain() const { return m_gain; }

int ConfigInterface::getWidth() const { return m_width; }

int ConfigInterface::getHeight() const { return m_height; }

bool ConfigInterface::getCompressed() const { return m_compressed; }

// --- Logging Helpers ---

void ConfigInterface::logInfo(const std::string& message) {
  std::cout << "[ConfigInterface INFO] " << message << std::endl;
}

void ConfigInterface::logError(const std::string& message) {
  std::cerr << "[ConfigInterface ERROR] " << message << std::endl;
}
