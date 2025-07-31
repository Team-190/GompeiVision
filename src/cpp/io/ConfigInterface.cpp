#include "io/ConfigInterface.h"

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
}  // namespace nt_keys

ConfigInterface::ConfigInterface(const std::string& hardwareID) {
  for (const auto subtable : nt::NetworkTableInstance::GetDefault()
                                 .GetTable("configs")
                                 ->GetSubTables()) {
    const auto table =
        nt::NetworkTableInstance::GetDefault().GetTable("configs")->GetSubTable(
            subtable);
    if (table->GetEntry(nt_keys::kHardwareID).GetString("invalid") ==
        hardwareID) {
      m_table = table;
    }
  }

  if (m_table) {
    // --- Initialize Subscribers ---
    // Subscribe to each topic with a default value. This ensures
    // that if the topic doesn't exist on the server yet, we have
    // a sensible default.
    m_setupModeSub =
        m_table->GetBooleanTopic(nt_keys::kSetupMode).Subscribe(false);
    m_roleSub = m_table->GetStringTopic(nt_keys::kRole).Subscribe("");
    m_cameraMatrixSub =
        m_table->GetDoubleArrayTopic(nt_keys::kCameraMatrix).Subscribe({});
    m_distCoeffsSub =
        m_table->GetDoubleArrayTopic(nt_keys::kDistCoeffs).Subscribe({});
    m_exposureSub = m_table->GetIntegerTopic(nt_keys::kExposure).Subscribe(0);
    m_gainSub = m_table->GetIntegerTopic(nt_keys::kGain).Subscribe(0);
    m_widthSub = m_table->GetIntegerTopic(nt_keys::kWidth).Subscribe(0);
    m_heightSub = m_table->GetIntegerTopic(nt_keys::kHeight).Subscribe(0);

    // --- Initialize Publishers ---
    // Get publishers for each topic. These will be used to send data back
    // when in setup mode.
    m_cameraMatrixPub =
        m_table->GetDoubleArrayTopic(nt_keys::kCameraMatrix).Publish();
    m_distCoeffsPub =
        m_table->GetDoubleArrayTopic(nt_keys::kDistCoeffs).Publish();
    m_exposurePub = m_table->GetIntegerTopic(nt_keys::kExposure).Publish();
    m_gainPub = m_table->GetIntegerTopic(nt_keys::kGain).Publish();
    m_widthPub = m_table->GetIntegerTopic(nt_keys::kWidth).Publish();
    m_heightPub = m_table->GetIntegerTopic(nt_keys::kHeight).Publish();

    // Perform an initial update to populate the cache with values from
    // NetworkTables if they exist.
    update();

    logInfo("ConfigInterface initialized for table: " + hardwareID);
  } else {
    logError("ConfigInterface failed to initialize table: " + hardwareID +
             " Network table not found");
  }
}

ConfigInterface::~ConfigInterface() {}

void ConfigInterface::update() {
  // Process the queue for each subscribed topic. This pulls all changes since
  // the last call to update().

  for (const auto& value : m_setupModeSub.ReadQueue()) {
    setSetupMode(value.value);
  }

  for (const auto& value : m_exposureSub.ReadQueue()) {
    m_exposure = value.value;
  }

  for (const auto& value : m_gainSub.ReadQueue()) {
    m_gain = value.value;
  }

  for (const auto& value : m_widthSub.ReadQueue()) {
    m_width = value.value;
  }

  for (const auto& value : m_heightSub.ReadQueue()) {
    m_height = value.value;
  }

  for (const auto& value : m_cameraMatrixSub.ReadQueue()) {
    if (value.value.size() == 9) {
      // The incoming data is a std::vector<double>. We create a 3x3 CV_64F
      // Mat from it. .clone() is crucial to copy the data, as the `value`
      // object is temporary.
      m_cameraMatrix =
          cv::Mat(3, 3, CV_64F, const_cast<double*>(value.value.data()))
              .clone();
    } else if (!value.value.empty()) {
      logError(
          "Received camera_matrix with incorrect size. Expected 9 elements, "
          "got " +
          std::to_string(value.value.size()));
    }
  }

  for (const auto& value : m_distCoeffsSub.ReadQueue()) {
    if (!value.value.empty()) {
      // Create a 1xN CV_64F Mat from the incoming vector.
      m_distortionCoeffs = cv::Mat(1, value.value.size(), CV_64F,
                                   const_cast<double*>(value.value.data()))
                               .clone();
    }
  }
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

// --- Configuration Setters ---

void ConfigInterface::setConfig(const LocalConfig& config) {
  if (!isSetupMode()) {
    logError("Attempted to set config while not in setup mode.");
    return;
  }
  if (config.cameraMatrix) setCameraMatrix(*config.cameraMatrix);
  if (config.distortionCoeffs) setDistortionCoeffs(*config.distortionCoeffs);
  if (config.exposure) setExposure(*config.exposure);
  if (config.gain) setGain(*config.gain);
  if (config.width) setWidth(*config.width);
  if (config.height) setHeight(*config.height);
}

void ConfigInterface::setCameraMatrix(const cv::Mat& matrix) {
  if (matrix.rows != 3 || matrix.cols != 3 || matrix.type() != CV_64F) {
    logError("Invalid format for setCameraMatrix. Must be a 3x3 CV_64F Mat.");
    return;
  }
  // Convert the cv::Mat to a std::vector<double> for publishing.
  std::vector<double> data(matrix.begin<double>(), matrix.end<double>());
  m_cameraMatrixPub.Set(data);
}

void ConfigInterface::setDistortionCoeffs(const cv::Mat& coeffs) {
  if (coeffs.rows != 1 || coeffs.type() != CV_64F) {
    logError(
        "Invalid format for setDistortionCoeffs. Must be a 1xN CV_64F Mat.");
    return;
  }
  std::vector<double> data(coeffs.begin<double>(), coeffs.end<double>());
  m_distCoeffsPub.Set(data);
}

void ConfigInterface::setExposure(const int exposure) {
  m_exposurePub.Set(exposure);
}

void ConfigInterface::setGain(const int gain) { m_gainPub.Set(gain); }

void ConfigInterface::setWidth(const int width) { m_widthPub.Set(width); }

void ConfigInterface::setHeight(const int height) { m_heightPub.Set(height); }

// --- Logging Helpers ---

void ConfigInterface::logInfo(const std::string& message) {
  std::cout << "[ConfigInterface INFO] " << message << std::endl;
}

void ConfigInterface::logError(const std::string& message) {
  std::cerr << "[ConfigInterface ERROR] " << message << std::endl;
}