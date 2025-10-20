#pragma once

#include <httplib.h>
#include <libudev.h>

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "../io/OutputPublisher.h"
#include "capture/Camera.h"
#include "cscore_cv.h"
#include "detector/FiducialDetector.h"
#include "detector/ObjectDetector.h"
#include "io/ConfigInterface.h"
#include "io/FieldInterface.h"
#include "util/QueuedFiducialData.h"
#include "util/QueuedObjectData.h"  
#include "util/ThreadSafeQueue.h"
#include "util/QueuedFrame.h"
#include "util/AnnotationData.h"


struct UdevContextDeleter {
  void operator()(struct udev* ctx) const {
    if (ctx) udev_unref(ctx);
  }
};

class Pipeline {
 public:
  Pipeline(const std::string& device_path, const std::string& hardware_id,
           const int stream_port, nt::NetworkTableInstance& nt_inst);
  ~Pipeline();

  void start();
  void during();
  void stop();

  bool isRunning() const;

 private:
  void processing_loop();
  void networktables_loop();
  void apriltag_detection_loop();
  void object_detection_loop();
  void annotation_loop();

  std::atomic<bool> m_is_running{false};

  std::string m_hardware_id;
  std::string m_role;
  int m_stream_port;
  std::mutex m_role_mutex;

  // Cached settings to prevent unnecessary reloads
  int m_active_width = 0;
  int m_active_height = 0;
  int m_active_exposure = 0;
  int m_active_gain = 0;
  cv::Mat m_camera_matrix;
  cv::Mat m_dist_coeffs;
  bool m_intrinsics_loaded = false;

  int m_active_model_index = -1;

  std::unique_ptr<Camera> m_camera;

  std::unique_ptr<cs::MjpegServer> m_mjpeg_server;
  std::unique_ptr<cs::CvSource> m_cv_source;

  std::unique_ptr<cs::MjpegServer> m_annotated_mjpeg_server;
  std::unique_ptr<cs::CvSource> m_annotated_cv_source;


  // --- Detectors ---
  FiducialDetector m_AprilTagDetector;
  std::unique_ptr<ObjectDetector> m_ObjectDetector;

  // --- Pipeline Data Flow ---
  ThreadSafeQueue<AprilTagResult> m_estimated_poses;
  ThreadSafeQueue<ObjectDetectResult> m_object_detections;
  ThreadSafeQueue<QueuedFrame> m_frames_for_apriltag_detection;
  ThreadSafeQueue<QueuedFrame> m_frames_for_object_detection;
  ThreadSafeQueue<AnnotationData> m_frames_for_annotation;

  // Threads & Control
  std::thread m_processing_thread;
  std::thread m_networktables_thread;
  std::thread m_apriltag_thread;
  std::thread m_object_detection_thread;
  std::thread m_annotation_thread;

  // --- NetworkTables Interface ---
  std::unique_ptr<ConfigInterface> m_config_interface;
  std::unique_ptr<OutputPublisher> m_output_publisher;
};
