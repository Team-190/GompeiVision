#include "pipeline/PipelineHelper.h"

#include <opencv2/opencv.hpp>

PipelineHelper::PipelineHelper() {}
PipelineHelper::~PipelineHelper() {}

void PipelineHelper::start_calibration_session(
    const std::string& role, const std::atomic<bool>& isCalibrating,
    std::mutex& calibrationMutex,
    std::unique_ptr<CalibrationSession>& calibrationSession,
    std::atomic<bool>& captureNextFrameForCalib,
    std::mutex& calibrationStatusMutex, std::string& calibrationStatusMessage,
    int squares_x, int squares_y, float square_length_m,
    float marker_length_m) {
  std::lock_guard lock(calibrationMutex);
  if (isCalibrating) {
    std::cerr << "[" << role
              << "] Cannot start new session while calibration is running."
              << std::endl;
    return;
  }
  calibrationSession = std::make_unique<CalibrationSession>(
      squares_x, squares_y, square_length_m, marker_length_m,
      cv::aruco::DICT_5X5_1000);
  captureNextFrameForCalib = false;

  // Reset status message
  std::lock_guard<std::mutex> status_lock(calibrationStatusMutex);
  calibrationStatusMessage = "Session started. Capture at least 10 frames.";
}

void PipelineHelper::add_calibration_frame(
    std::atomic<bool>& captureNextFrameForCalib) {
  captureNextFrameForCalib = true;
}

void PipelineHelper::async_finish_calibration(
    std::atomic<bool>& isCalibrating, std::mutex& calibrationMutex,
    std::unique_ptr<CalibrationSession>& calibrationSession,
    std::mutex& calibrationStatusMutex, std::string& calibrationStatusMessage,
    const std::string& output_file) {
  double error;
  {
    std::lock_guard<std::mutex> lock(calibrationMutex);
    if (!calibrationSession) {
      error = -1.0;  // Should not happen if logic is correct
    } else {
      error = calibrationSession->finish_calibration(output_file);
      calibrationSession.reset();
    }
  }

  {
    std::lock_guard<std::mutex> lock(calibrationStatusMutex);
    if (error < 0) {
      calibrationStatusMessage =
          "ERROR: Calibration failed. Check console for details.";
    } else {
      calibrationStatusMessage =
          "OK: Calibration finished for " + output_file +
          " with reprojection error: " + std::to_string(error);
    }
  }
  isCalibrating = false;
}