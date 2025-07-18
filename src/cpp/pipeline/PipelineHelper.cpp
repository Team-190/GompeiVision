#include "pipeline/PipelineHelper.h"

#include <fields/fields.h>
#include <linux/limits.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <nlohmann/json.hpp>
#include <units/length.h>

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

bool PipelineHelper::load_camera_intrinsics(const std::string& role,
                                            cv::Mat& cameraMatrix,
                                            cv::Mat& distCoeffs) {
  std::string file_path;
  const char* home_dir = getenv("HOME");
  if (home_dir) {
    file_path =
        std::string(home_dir) + "/GompeiVision/" + role + "_calibration.yml";
  } else {
    file_path = role + "_calibration.yml";  // Fallback to current directory
  }
  try {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "[" << role
                << "] ERROR: Could not open calibration file: " << file_path
                << std::endl;
      return false;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    std::cout << "[" << role << "] Calibration data loaded successfully."
              << std::endl;
  } catch (const cv::Exception& e) {
    std::cerr << "[" << role << "] ERROR: Failed to read calibration data. "
              << e.what() << std::endl;
    return false;
  }
  return true;
}

std::map<int, frc::Pose3d> PipelineHelper::load_field_layout(
    const std::string& field_name) {
  std::vector<std::filesystem::path> search_paths;

  // 1. Path relative to the executable. This is the most robust method for an
  // installed application on Linux. It works regardless of install location or
  // current working directory.
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  if (count != -1) {
    std::filesystem::path exe_path =
        std::filesystem::path(std::string(result, count));
    // Search path for a standard installation (e.g., /usr/local/):
    // exe is in <prefix>/bin/GompeiVision
    // data is in <prefix>/share/GompeiVision/fields/
    search_paths.push_back(exe_path.parent_path().parent_path() / "share" /
                           "GompeiVision" / "fields" / field_name);
  }

  // 2. Path relative to current working directory (for development)
  // Allows running from the build directory if you copy 'fields' there.
  search_paths.push_back(std::filesystem::current_path() / "fields" /
                         field_name);

  for (const auto& p : search_paths) {
    std::ifstream f(p);
    if (f.is_open()) {
      std::cout << "[PipelineHelper] Found and loading field layout from: "
                << std::filesystem::absolute(p) << std::endl;
      try {
        nlohmann::json data = nlohmann::json::parse(f);
        const auto& tags = data.at("tags");

        std::map<int, frc::Pose3d> layout;
        for (const auto& tag_item : tags) {
          int id = tag_item.at("ID");

          const auto& pose_json = tag_item.at("pose");
          const auto& translation_json = pose_json.at("translation");
          const auto& quat_json = pose_json.at("rotation").at("quaternion");

          frc::Translation3d translation(
              units::meter_t{translation_json.at("x").get<double>()},
              units::meter_t{translation_json.at("y").get<double>()},
              units::meter_t{translation_json.at("z").get<double>()});

          frc::Quaternion quaternion(quat_json.at("W").get<double>(),
                                     quat_json.at("X").get<double>(),
                                     quat_json.at("Y").get<double>(),
                                     quat_json.at("Z").get<double>());

          layout[id] = frc::Pose3d(translation, frc::Rotation3d(quaternion));
        }
        std::cout << "[PipelineHelper] Successfully loaded " << layout.size()
                  << " tags." << std::endl;
        return layout;

      } catch (const nlohmann::json::exception& e) {
        std::cerr << "[PipelineHelper] ERROR: Failed to parse field layout from "
                  << p << ". Details: " << e.what() << std::endl;
      }
    }
  }

  std::cerr << "---" << std::endl;
  std::cerr << "[PipelineHelper] FATAL: Could not find field layout file '"
            << field_name << "'." << std::endl;
  std::cerr << "[PipelineHelper] Searched in the following locations:"
            << std::endl;
  for (const auto& p : search_paths) {
    std::cerr << " - " << std::filesystem::absolute(p) << std::endl;
  }
  std::cerr << "---" << std::endl;

  return {};  // Return empty map if file not found or failed to parse
}
