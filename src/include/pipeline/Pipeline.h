#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "apriltag.h"
#include "capture/Camera.h"
#include "detector/FiducialDetector.h"
#include "detector/ObjectDetector.h"
#include "estimator/CameraPoseEstimator.h"
#include "estimator/ObjectEstimator.h"
#include "estimator/TagAngleCalculator.h"
#include "io/ConfigInterface.h"
#include "io/FieldInterface.h"
#include "io/OutputPublisher.h"
#include "util/ThreadSafeQueue.h"
#include "wpi/DataLog.h"

// forward declare
class Log;

class Pipeline {
 public:
  explicit Pipeline(int id, const std::string& name, wpi::log::DataLog& log);
  ~Pipeline();
  void start();

 private:
  void m_process_thread();
  void m_network_thread();
  void m_setup_mode_thread();

  int m_id;
  std::string m_name;

  std::atomic<bool> m_run = true;
  std::thread m_process_handle;
  std::thread m_network_handle;
  std::thread m_setup_mode_handle;

  std::unique_ptr<Camera> m_camera;
  std::unique_ptr<ConfigInterface> m_config_interface;
  std::unique_ptr<FieldInterface> m_field_interface;
  std::unique_ptr<FiducialDetector> m_fiducial_detector;
  std::unique_ptr<ObjectDetector> m_object_detector;
  std::unique_ptr<ObjectEstimator> m_object_estimator;
  std::unique_ptr<TagAngleCalculator> m_tag_angle_calculator;
  std::unique_ptr<CameraPoseEstimator> m_camera_pose_estimator;
  std::unique_ptr<OutputPublisher> m_output_publisher;

  // intermediate queues
  ThreadSafeQueue<AprilTagResult> m_output_queue;

  wpi::log::DataLog& m_log;
  wpi::log::BooleanLogEntry m_log_is_running;
  wpi::log::IntegerLogEntry m_log_camera_id;
  wpi::log::StringLogEntry m_log_camera_name;

  // For setup mode annotations
  std::vector<FiducialImageObservation> m_latest_fiducial_observations;
  std::vector<ObjDetectObservation> m_latest_obj_observations;
  std::mutex m_observation_mutex;
};
