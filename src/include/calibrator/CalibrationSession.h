#pragma once

#include <atomic>  // ADDED: For std::atomic
#include <opencv2/aruco/charuco.hpp>
#include <string>
#include <vector>

/**
 * @class CalibrationSession
 * @brief Manages the state and process of a camera calibration session.
 *
 * This class accumulates data from multiple frames and performs the final
 * calibration calculation, designed for use in an interactive calibration tool.
 */
class CalibrationSession {
 public:
  /**
   * @brief Constructs a calibration session with specific board properties.
   * @param squares_x The number of squares on the board's X axis.
   * @param squares_y The number of squares on the board's Y axis.
   * @param square_length_m The measured length of a square's side in meters.
   * @param marker_length_m The measured length of a marker's side in meters.
   * @param dictionary_name The ArUco dictionary used to generate the board.
   */
  CalibrationSession(int squares_x, int squares_y, float square_length_m,
                     float marker_length_m,
                     cv::aruco::PredefinedDictionaryType dictionary_name);

  /**
   * @brief Processes a single video frame.
   *
   * Detects the ChArUco board in the frame. If 'save_this_frame' is true
   * and a valid board is found, the data is stored for the final calibration.
   * @param image The input image to process. The function will draw detections
   * on this image.
   * @param capture_request_flag
   * set.
   * @return True if a board was successfully detected, false otherwise.
   */
  bool process_frame(const cv::Mat& image, std::atomic<bool>& capture_request_flag);
  /**
   * @brief Performs the final calibration calculation using all saved frame
   * data.
   * @param output_file The path to the .yml file where results will be saved.
   * @return The final reprojection error. Returns a negative value on failure.
   */
  double finish_calibration(const std::string& output_file) const;

  /**
   * @brief Gets the number of valid frames collected so far.
   */
  size_t get_frame_count() const;

  /**
   * @brief Gets all the corner points collected so far.
   * @return A flat vector of all corner points (cv::Point2f).
   */
  std::vector<cv::Point2f> get_all_corners() const;

 private:
  // m_dictionary must be constructed before it is passed to the m_board
  // constructor.
  cv::aruco::Dictionary m_dictionary;
  cv::Ptr<cv::aruco::CharucoBoard> m_board;
  cv::aruco::CharucoDetector m_charuco_detector;

  // Collected data
  std::vector<std::vector<cv::Point2f>> m_all_charuco_corners;
  std::vector<std::vector<int>> m_all_charuco_ids;
  std::vector<std::vector<cv::Point2f>> allImagePoints;
  std::vector<std::vector<cv::Point3f>> allObjectPoints;

  cv::Size m_image_size;
};
