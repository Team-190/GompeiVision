#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>

#include "calibrator/CalibrationSession.h"
#include "frc/geometry/Pose3d.h"
#include "nlohmann/json.hpp"

class PipelineHelper {
 public:
  PipelineHelper();
  ~PipelineHelper();

  static void start_calibration_session(
      const std::string& role, const std::atomic<bool>& isCalibrating,
      std::mutex& calibrationMutex,
      std::unique_ptr<CalibrationSession>& calibrationSession,
      std::atomic<bool>& captureNextFrameForCalib,
      std::mutex& calibrationStatusMutex, std::string& calibrationStatusMessage,
      int squares_x, int squares_y, float square_length_m,
      float marker_length_m);
  static void add_calibration_frame(
      std::atomic<bool>& captureNextFrameForCalib);
  static void async_finish_calibration(
      std::atomic<bool>& isCalibrating, std::mutex& calibrationMutex,
      std::unique_ptr<CalibrationSession>& calibrationSession,
      std::mutex& calibrationStatusMutex, std::string& calibrationStatusMessage,
      const std::string& output_file);

  static bool load_camera_intrinsics(const std::string& role,
                                     cv::Mat& cameraMatrix,
                                     cv::Mat& distCoeffs);

  static std::map<int, frc::Pose3d> load_field_layout(
      const std::string& field_name);
};