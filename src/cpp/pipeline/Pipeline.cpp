#include "pipeline/Pipeline.h"

#include <chrono>
#include <cscore/CvSource.h>
#include <cscore/MjpegServer.h>
#include <cscore/cscore_oo.h>
#include <fmt/format.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <wpi/DataLog.h>
#include <wpi/json.h>

#include "util/Log.h"
#include "util/PoseUtils.h"

using namespace std::chrono_literals;

Pipeline::Pipeline(int id, const std::string& name, wpi::log::DataLog& log)
    : m_id(id),
      m_name(name),
      m_log(log),
      m_log_is_running(log, fmt::format("pipeline/{}/is_running", name)),
      m_log_camera_id(log, fmt::format("pipeline/{}/camera_id", name)),
      m_log_camera_name(log, fmt::format("pipeline/{}/camera_name", name)) {}

Pipeline::~Pipeline() {
  m_run.store(false);

  if (m_process_handle.joinable()) {
    m_process_handle.join();
  }
  if (m_network_handle.joinable()) {
    m_network_handle.join();
  }
  if (m_setup_mode_handle.joinable()) {
    m_setup_mode_handle.join();
  }
  Log(m_name) << "pipeline shutdown complete" << std::endl;
}

void Pipeline::start() {
  Log(m_name) << "starting pipeline" << std::endl;
  m_process_handle = std::thread(&Pipeline::m_process_thread, this);
}

void Pipeline::m_process_thread() {
  Log(m_name) << "process thread started" << std::endl;
  m_log_camera_id.Append(m_id);
  m_log_camera_name.Append(m_name);

  // create interfaces
  m_camera = std::make_unique<Camera>(m_name, m_id, m_log);
  m_config_interface = std::make_unique<ConfigInterface>(m_name, m_log);
  m_field_interface = std::make_unique<FieldInterface>(m_name, m_log);

  // wait for config to be populated
  while (m_run.load() && m_config_interface->get_role().empty()) {
    Log(m_name) << "waiting for config" << std::endl;
    std::this_thread::sleep_for(1s);
  }

  // load camera calibration from config
  m_camera->load_calibration(m_config_interface->get_camera_matrix(),
                             m_config_interface->get_dist_coeffs());

  // create detectors & estimators
  m_fiducial_detector = std::make_unique<FiducialDetector>(m_name, m_log);
  m_object_detector = std::make_unique<ObjectDetector>(
      "/home/gompei/GompeiVision/models/reefscape_yolov8n.onnx",
      "/home/gompei/GompeiVision/models/reefscape.names");
  m_tag_angle_calculator = std::make_unique<TagAngleCalculator>(m_name, m_log);
  m_camera_pose_estimator =
      std::make_unique<CameraPoseEstimator>(m_name, m_log);

  // start network thread
  m_network_handle = std::thread(&Pipeline::m_network_thread, this);

  bool last_setup_mode = false;

  while (m_run.load()) {
    m_log_is_running.Append(true);

    // check for setup mode change
    bool setup_mode = m_config_interface->get_setup_mode();
    if (setup_mode && !last_setup_mode) {
      if (m_setup_mode_handle.joinable()) {
        m_setup_mode_handle.join();
      }
      m_setup_mode_handle = std::thread(&Pipeline::m_setup_mode_thread, this);
    }
    last_setup_mode = setup_mode;

    // TODO: check for role change

    // update camera config
    m_camera->set_exposure(m_config_interface->get_exposure());

    // grab frame
    QueuedFrame q_frame;
    if (!m_camera->getFrame(q_frame)) {
      // failed to get frame, wait and try again
      std::this_thread::sleep_for(100ms);
      continue;
    }

    AprilTagResult result;
    result.capture_timestamp = q_frame.timestamp;

    std::vector<FiducialImageObservation> fiducial_observations;
    std::vector<ObjDetectObservation> object_observations;

    m_fiducial_detector->detect(*q_frame, fiducial_observations);
    m_object_detector->detect(*q_frame, object_observations);

    // Update latest observations for setup mode stream
    {
      std::lock_guard<std::mutex> lock(m_observation_mutex);
      m_latest_fiducial_observations = fiducial_observations;
      m_latest_obj_observations = object_observations;
    }

    m_tag_angle_calculator->calculate(
        fiducial_observations, m_camera->get_camera_matrix(),
        m_camera->get_dist_coeffs(), result.tag_data);

    m_camera_pose_estimator->estimate(
        fiducial_observations, m_field_interface->get_layout(),
        m_camera->get_camera_matrix(), m_camera->get_dist_coeffs(),
        result.primary_pose, result.secondary_pose);

    // TODO: object estimation

    m_output_queue.push(result);
  }
}

void Pipeline::m_network_thread() {
  Log(m_name) << "network thread started" << std::endl;

  // wait for config to be populated
  while (m_run.load() && m_config_interface->get_role().empty()) {
    Log(m_name) << "waiting for config" << std::endl;
    std::this_thread::sleep_for(1s);
  }

  // create publisher
  m_output_publisher = std::make_unique<OutputPublisher>(m_name, m_log);
  while (m_run.load()) {
    auto result = m_output_queue.pop();
    if (result.has_value()) {
      m_output_publisher->publish(result.value());
    }
  }
}

void Pipeline::m_setup_mode_thread() {
  Log(m_name) << "setup mode thread started" << std::endl;

  cs::MjpegServer server("camera_server", 5800 + m_id);
  cs::CvSource source("cv_source", cs::VideoMode::kMJPEG, 1280, 720, 30);
  server.SetSource(source);
  Log(m_name) << "mjpeg server started on port " << std::to_string(5800 + m_id)
              << std::endl;

  while (m_run.load()) {
    if (!m_config_interface->get_setup_mode()) {
      Log(m_name) << "exiting setup mode" << std::endl;
      // cs::RemoveServer("camera_server"); // TODO: does this work as expected?
      break;
    }

    QueuedFrame q_frame;
    if (!m_camera->getFrame(q_frame)) {
      Log(m_name, true) << "failed to get frame for setup mode" << std::endl;
      std::this_thread::sleep_for(100ms);
      continue;
    }
    
    cv::Mat setup_frame = q_frame.frame.clone();

    // Get the latest observations safely
    std::vector<FiducialImageObservation> local_fiducial_obs;
    std::vector<ObjDetectObservation> local_object_obs;
    {
      std::lock_guard<std::mutex> lock(m_observation_mutex);
      local_fiducial_obs = m_latest_fiducial_observations;
      local_object_obs = m_latest_obj_observations;
    }

    // Annotate Fiducials
    for (const auto& obs : local_fiducial_obs) {
      // Create a vector of cv::Point for polylines
      std::vector<cv::Point> points;
      for (size_t i = 0; i < obs.corner_pixels.size(); i += 2) {
        points.emplace_back(obs.corner_pixels[i], obs.corner_pixels[i + 1]);
      }
      
      // Draw the outline of the tag
      if (points.size() == 4) {
        cv::polylines(setup_frame, points, true, cv::Scalar(0, 255, 0), 2);
      }

      // Draw the tag ID
      std::string tag_text = "ID: " + std::to_string(obs.tag_id);
      cv::putText(setup_frame, tag_text, points[0], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }

    // Annotate Objects
    if (m_object_detector) { // Ensure object detector is initialized
      for (const auto& obs : local_object_obs) {
        if (obs.corner_pixels.size() < 8) continue;

        // Define the bounding box corners from the corner_pixels vector
        cv::Point tl(obs.corner_pixels[0], obs.corner_pixels[1]);
        cv::Point br(obs.corner_pixels[4], obs.corner_pixels[5]);
        cv::rectangle(setup_frame, tl, br, cv::Scalar(255, 0, 0), 2);
        
        // Prepare label text (class name and confidence)
        std::string label = "Unknown";
        if (obs.obj_class < m_object_detector->m_class_names.size()) {
            label = m_object_detector->m_class_names[obs.obj_class];
        }
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << obs.confidence;
        label += ": " + ss.str();
        
        // Put the label above the bounding box
        int baseline;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
        cv::Point label_origin(tl.x, tl.y - 10 > 0 ? tl.y - 10 : tl.y + label_size.height);
        cv::putText(setup_frame, label, label_origin, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
      }
    }


    std::vector<uchar> buf;
    cv::imencode(".jpg", setup_frame, buf);
    server.write(buf.data(), buf.size());
  }
}
