#include "calibrator/CalibrationSession.h"

#include <iostream>
#include <opencv2/calib3d.hpp>

CalibrationSession::CalibrationSession(
    const int squares_x, const int squares_y, const float square_length_m,
    const float marker_length_m,
    const cv::aruco::PredefinedDictionaryType dictionary_name)
    : m_dictionary(cv::aruco::getPredefinedDictionary(dictionary_name)),
      m_board(cv::makePtr<cv::aruco::CharucoBoard>(
          cv::Size(squares_x, squares_y), square_length_m, marker_length_m,
          m_dictionary)),
      m_charuco_detector(*m_board) {
  m_board->setLegacyPattern(true);
  auto params = cv::aruco::CharucoParameters();
  params.minMarkers = 1;
  m_charuco_detector.setCharucoParameters(params);
  std::cout << "[CalibrationSession] Session created." << std::endl;
}

bool CalibrationSession::process_frame(
    const cv::Mat& image, std::atomic<bool>& capture_request_flag) {
  if (image.empty()) {
    return false;
  }

  // Set the image size from the first frame processed
  if (m_image_size.empty()) {
    m_image_size = image.size();
  }

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;

  // Detect markers and then interpolate the board corners
  m_charuco_detector.detectBoard(image, charuco_corners, charuco_ids);

  // --- FIX: Check for the minimum number of corners required for calibration
  // --- The calibration algorithm requires at least 4 points. A higher number
  // can lead to a more stable result, but 4 is the minimum to prevent a crash.
  if (charuco_ids.size() >= 4) {
    // Draw detections on the mutable input image for visual feedback
    cv::aruco::drawDetectedCornersCharuco(image, charuco_corners, charuco_ids);

    if (capture_request_flag.load()) {
      m_all_charuco_corners.push_back(charuco_corners);
      m_all_charuco_ids.push_back(charuco_ids);
      capture_request_flag = false;  // Reset the flag after fulfilling request
    }
    return true;
  }

  return false;
}

double CalibrationSession::finish_calibration(
    const std::string& output_file) const {
  if (m_all_charuco_corners.size() < 10) {
    std::cerr << "[CalibrationSession] ERROR: Not enough valid frames for "
                 "calibration. Need at least 10."
              << std::endl;
    return -1.0;
  }

  std::cout << "[CalibrationSession] Running final calibration on "
            << m_all_charuco_corners.size() << " frames..." << std::endl;

  // Prepare data for calibration
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  // --- KEY CHANGE: Use the correct ChArUco calibration function ---
  // This function is specifically designed for this task and handles the
  // 3D point matching internally.
  double reprojection_error = cv::aruco::calibrateCameraCharuco(
      m_all_charuco_corners, m_all_charuco_ids, m_board, m_image_size,
      camera_matrix, dist_coeffs, rvecs, tvecs);

  // Save the results to a file
  cv::FileStorage fs(output_file, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cerr << "[CalibrationSession] ERROR: Could not open output file: "
              << output_file << std::endl;
    return -1.0;
  }

  fs << "camera_matrix" << camera_matrix;
  fs << "distortion_coefficients" << dist_coeffs;
  fs << "image_width" << m_image_size.width;
  fs << "image_height" << m_image_size.height;
  fs << "reprojection_error" << reprojection_error;
  fs.release();

  std::cout << "[CalibrationSession] Calibration results saved to "
            << output_file << std::endl;
  return reprojection_error;
}

size_t CalibrationSession::get_frame_count() const {
  return m_all_charuco_corners.size();
}

std::vector<cv::Point2f> CalibrationSession::get_all_corners() const {
  std::vector<cv::Point2f> all_points;
  for (const auto& corner_set : m_all_charuco_corners) {
    all_points.insert(all_points.end(), corner_set.begin(), corner_set.end());
  }
  return all_points;
}
