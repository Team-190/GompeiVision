#include "pipeline/PipelineHelper.h"

#include <fields/fields.h>

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
    file_path = role + "_calibration.yml"; // Fallback to current directory
  }
    try {
      cv::FileStorage fs(file_path, cv::FileStorage::READ);
      if (!fs.isOpened()) {
        std::cerr << "[" << role << "] ERROR: Could not open calibration file: "
                  << file_path << std::endl;
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
  {
    std::map<int, frc::Pose3d> layout;

    // 1. Get the list of all available fields
    const std::span<const fields::Field> all_fields = fields::GetFields();

    // 2. Find the field with the matching name
    const fields::Field* target_field = nullptr;
    for (const auto& field : all_fields) {
      if (field.name == field_name) {
        target_field = &field;
        break;
      }
    }

    if (!target_field) {
      std::cerr << "ERROR: Could not find field layout for '" << field_name
                << "'" << std::endl;
      return layout;  // Return empty map
    }

    // 3. Get the JSON data and parse it
    try {
      nlohmann::json layout_json =
          nlohmann::json::parse(target_field->getJson());

      // 4. Iterate through the tags and create frc::Pose3d objects
      for (const auto& tag_data : layout_json["tags"]) {
        int id = tag_data["ID"];
        auto t = tag_data["pose"]["translation"];
        auto q = tag_data["pose"]["rotation"]["quaternion"];

        layout[id] = frc::Pose3d(
            frc::Translation3d(units::meter_t{t["x"]}, units::meter_t{t["y"]},
                               units::meter_t{t["z"]}),
            frc::Rotation3d(frc::Quaternion(q["W"], q["X"], q["Y"], q["Z"])));
      }
      std::cout << "Successfully loaded field layout for '" << field_name
                << "' with " << layout.size() << " tags." << std::endl;
    } catch (const nlohmann::json::exception& e) {
      std::cerr << "ERROR: Failed to parse field layout JSON for '"
                << field_name << "'. Details: " << e.what() << std::endl;
    }

    return layout;
  }
}
