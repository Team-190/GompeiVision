#include "pipeline/Pipeline.h"

#include <cscore/cscore_cv.h>
#include <httplib.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <thread>

#include "capture/Camera.h"
#include "estimator/CameraPoseEstimator.h"
#include "estimator/SingleTagPoseEstimator.h"
#include "estimator/TagAngleCalculator.h"
#include "io/OutputPublisher.h"
#include "pipeline/PipelineHelper.h"

Pipeline::Pipeline(const std::string& device_path,
                   const std::string& hardware_id, const int stream_port,
                   nt::NetworkTableInstance& nt_inst)
    : m_hardware_id(hardware_id), m_stream_port(stream_port) {
  m_config_interface = std::make_unique<ConfigInterface>(hardware_id, nt_inst);

  m_output_publisher =
      std::make_unique<NTOutputPublisher>(m_hardware_id, nt_inst);

  m_config_interface->waitForInitialization();

  m_config_interface->update();

  m_role = m_config_interface->getRole();

  // Initialize active settings from config
  m_active_width = m_config_interface->getWidth();
  m_active_height = m_config_interface->getHeight();
  m_active_exposure = m_config_interface->getExposure();
  m_active_gain = m_config_interface->getGain();

  std::cout << "[" << m_role << "] Initializing pipeline..." << std::endl;

  m_camera = std::make_unique<Camera>(device_path, hardware_id, m_active_width,
                                      m_active_height);

  // Set initial camera connection status

  if (!m_camera->isConnected()) {
    std::cerr << "[" << m_role
              << "] WARNING: Camera failed to connect on startup." << std::endl;
  }

  m_camera->setExposure(m_active_exposure);
  m_camera->setBrightness(m_active_gain);

  if (!FieldInterface::isInitialized()) {
    FieldInterface::initialize(nt_inst);
  }

  while (!FieldInterface::update())

    m_intrinsics_loaded = PipelineHelper::load_camera_intrinsics(
        *m_config_interface, m_camera_matrix, m_dist_coeffs);

  if (m_intrinsics_loaded) {
    std::cout << "[" << m_role << "] Initial Camera Matrix: "
              << cv::format(m_camera_matrix, cv::Formatter::FMT_DEFAULT)
              << std::endl;
    std::cout << "[" << m_role << "] Initial Distortion Coefficients: "
              << cv::format(m_dist_coeffs, cv::Formatter::FMT_DEFAULT)
              << std::endl;
  } else {
    std::cerr << "[" << m_role
              << "] WARNING: Cannot run Pose Estimator without valid "
                 "calibration. Pose estimation will be disabled."
              << std::endl;
  }

  std::cout << "[" << m_role
            << "] Initialized pipeline with ID: " << hardware_id << std::endl;
}

Pipeline::~Pipeline() {
  std::cout << "[" << m_role << "] Shutting down pipeline..." << std::endl;
  stop();
}

void Pipeline::start() {
  if (m_is_running) return;
  m_is_running = true;
  m_capture_thread = std::thread(&Pipeline::capture_loop, this);
  m_processing_thread = std::thread(&Pipeline::processing_loop, this);
  m_networktables_thread = std::thread(&Pipeline::networktables_loop, this);
}

void Pipeline::during() {
  if (!m_is_running) return;

  m_config_interface->update();

  cv::Mat new_camera_matrix;
  cv::Mat new_dist_coeffs;
  const bool new_intrinsics_loaded = PipelineHelper::load_camera_intrinsics(
      *m_config_interface, new_camera_matrix, new_dist_coeffs);

  bool changed = false;
  if (m_intrinsics_loaded != new_intrinsics_loaded) {
    changed = true;
  } else if (new_intrinsics_loaded) {
    if (cv::norm(m_camera_matrix, new_camera_matrix, cv::NORM_L1) != 0 ||
        cv::norm(m_dist_coeffs, new_dist_coeffs, cv::NORM_L1) != 0) {
      changed = true;
    }
  }

  if (changed) {
    m_camera_matrix = new_camera_matrix.clone();
    m_dist_coeffs = new_dist_coeffs.clone();
    m_intrinsics_loaded = new_intrinsics_loaded;

    if (m_intrinsics_loaded) {
      std::cout << "[" << m_role << "] Camera Matrix updated: "
                << cv::format(m_camera_matrix, cv::Formatter::FMT_DEFAULT)
                << std::endl;
      std::cout << "[" << m_role << "] Distortion Coefficients updated: "
                << cv::format(m_dist_coeffs, cv::Formatter::FMT_DEFAULT)
                << std::endl;
    } else {
      std::cerr << "[" << m_role
                << "] WARNING: Camera intrinsics have been invalidated."
                << std::endl;
    }
  }

  if (m_config_interface->isSetupMode()) {
    // --- We are in Setup Mode ---

    const int new_width = m_config_interface->getWidth();
    const int new_height = m_config_interface->getHeight();
    const int new_exposure = m_config_interface->getExposure();
    const int new_gain = m_config_interface->getGain();

    const bool resolution_changed =
        (new_width != m_active_width || new_height != m_active_height);
    const bool exposure_changed = (new_exposure != m_active_exposure);
    const bool gain_changed = (new_gain != m_active_gain);

    // If resolution changed, we must restart the server.
    if (resolution_changed) {
      std::cout << "[" << m_role << "] Resolution changed to " << new_width
                << "x" << new_height << ". Restarting stream." << std::endl;

      // 1. Stop the server if it's running
      if (m_config_interface->isSetupMode()) {
        m_mjpeg_server.reset();
        m_cv_source.reset();
      }

      // 2. Update active settings
      m_active_width = new_width;
      m_active_height = new_height;

      // 3. Apply to camera
      // if (m_camera) {
      //   m_camera->setResolution(m_active_width, m_active_height);
      // }
      // The server will be started (or restarted) by the logic below.
    }

    // Apply other settings if they have changed.
    if (exposure_changed) {
      m_active_exposure = new_exposure;
      if (m_camera) {
        m_camera->setExposure(m_active_exposure);
      }
      std::cout << "[" << m_role << "] Exposure updated to "
                << m_active_exposure << std::endl;
    }
    if (gain_changed) {
      m_active_gain = new_gain;
      if (m_camera) {
        m_camera->setBrightness(m_active_gain);
      }
      std::cout << "[" << m_role << "] Gain updated to " << m_active_gain
                << std::endl;
    }

    // Ensure the server is running if we are in setup mode.
    if (m_config_interface->isSetupMode() && !m_mjpeg_server) {
      std::cout << "[" << m_role
                << "] Setup mode enabled. Initializing servers." << std::endl;

      m_mjpeg_server =
          std::make_unique<cs::MjpegServer>(m_role + "_stream", m_stream_port);
      m_cv_source = std::make_unique<cs::CvSource>(
          m_role + "_source", cs::VideoMode::PixelFormat::kBGR, m_active_width,
          m_active_height, 60);

      CS_Status status = 0;
      cs::SetSinkSource(m_mjpeg_server->GetHandle(),
                        m_cv_source.get()->GetHandle(), &status);

      std::cout << "[" << m_role << "] MJPEG stream available at port "
                << m_stream_port << std::endl;
    }

  } else {
    // --- We are NOT in Setup Mode ---
    if (!m_config_interface->isSetupMode() && m_mjpeg_server)
      std::cout << "[" << m_role << "] Setup mode disabled. Stopping servers."
                << std::endl;
    m_mjpeg_server.reset();
    m_cv_source.reset();
  }
}

void Pipeline::stop() {
  if (!m_is_running) return;
  m_is_running = false;

  m_estimated_poses.shutdown();

  if (m_capture_thread.joinable()) m_capture_thread.join();
  if (m_processing_thread.joinable()) m_processing_thread.join();
  if (m_networktables_thread.joinable()) m_networktables_thread.join();

  std::cout << "[" << m_role << "] All threads stopped." << std::endl;
}

bool Pipeline::isRunning() const { return m_is_running; }

void Pipeline::capture_loop() {
  uint8_t frame_count = 0;
  auto start_time = std::chrono::high_resolution_clock::now();

  while (m_is_running) {
    QueuedFrame frame;
    bool frame_was_captured = false;

    if (m_camera) {
      if (m_camera->getFrame(frame.frame, frame.timestamp)) {
        frame_was_captured = true;
        m_captured_frames.push(frame);
        frame_count++;  // Increment captured frames
      } else {
        m_camera->attemptReconnect();
      }
    }

    if (frame_was_captured) {
      if (m_config_interface->isSetupMode() && m_cv_source) {
        m_cv_source->PutFrame(frame.frame);
      }
    }

    // Check if one second has passed
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time).count();
    if (elapsed >= 1.0) {
      std::cout << "[" << m_role << "] Capture FPS: " << frame_count
                << std::endl;
      if (m_output_publisher) {
        m_output_publisher->SendCaptureFPS(frame_count);
      }
      frame_count = 0;   // Reset counter
      start_time = now;  // Reset timer
    }
  }
}

void Pipeline::processing_loop() {
  uint8_t frame_count = 0;
  auto start_time = std::chrono::high_resolution_clock::now();

  while (m_is_running) {
    if (QueuedFrame frame; m_captured_frames.waitAndPopWithTimeout(
            frame, std::chrono::milliseconds(1))) {
      FiducialImageObservation frame_observation;
      m_AprilTagDetector.detect(frame, frame_observation);
      frame_observation.timestamp = frame.timestamp;

      if (!frame_observation.tag_ids.empty() && m_intrinsics_loaded) {
        AprilTagResult result;
        result.timestamp = frame_observation.timestamp;
        result.camera_role = m_role;
        constexpr double tag_size_m = 0.1651;

        cv::Mat cameraMatrix = m_camera_matrix.clone();
        cv::Mat distCoeffs = m_dist_coeffs.clone();

        CameraPoseEstimator::estimatePose(frame_observation, result,
                                          cameraMatrix, distCoeffs, tag_size_m,
                                          FieldInterface::getMap());

        TagAngleCalculator::calculate(frame_observation, result, cameraMatrix,
                                      distCoeffs, tag_size_m);

        m_estimated_poses.push(result);
      }

      frame_count++;  // Increment processed frames

      // Check if one second has passed
      auto now = std::chrono::high_resolution_clock::now();
      double elapsed = std::chrono::duration<double>(now - start_time).count();
      if (elapsed >= 1.0) {
        if (m_output_publisher) {
          m_output_publisher->SendProcessingFPS(frame_count);
        }
        std::cout << "[" << m_role << "] Processing FPS: " << frame_count
                  << std::endl;
        frame_count = 0;   // Reset counter
        start_time = now;  // Reset timer
      }
    }
  }
}

void Pipeline::networktables_loop() {
  while (m_is_running) {
    // Always publish the latest known connection status.
    if (m_output_publisher) {
      m_output_publisher->SendConnectionStatus(m_camera->isConnected());

      AprilTagResult result;
      if (!m_estimated_poses.waitAndPopWithTimeout(
              result, std::chrono::milliseconds(1))) {
        continue;
      }

      m_output_publisher->SendAprilTagResult(result);
    }
  }
}
