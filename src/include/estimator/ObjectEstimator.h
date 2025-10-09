#pragma once

#include <opencv2/core/mat.hpp>
#include <vector>
#include <frc/geometry/Pose3d.h>

#include "util/QueuedObjectData.h"

struct GamePieceData {
    int class_id; 
    double center_x;
    double center_y;
    double width;
    double height;
    double x_position;
    double y_position;
};

/**
 * @class ObjectEstimator
 * @brief Calculates the viewing angles to the corners of a detected object
 * and its distance from the camera.
 *
 * This class is analogous to the TagAngleCalculator and provides the necessary
 * data for robot alignment and targeting.
 */
class ObjectEstimator {
 public:
  /**
   * @brief Calculates angles and distances for all detected objects.
   *
   * For each object in the observation, this function:
   * 1. Undistorts the 2D corner points of the bounding box.
   * 2. Calculates the horizontal and vertical viewing angle to each corner.
   * 3. Uses the known physical width of the object to estimate distance.
   * 4. Populates the `corner_angles` vector in the provided observation.
   *
   * @param observation A reference to the object observation to be populated.
   * @param cameraMatrix The camera's intrinsic matrix.
   * @param distCoeffs The camera's distortion coefficients.
   */
  static void calculate(ObjDetectObservation& observation,
                        const cv::Mat& cameraMatrix,
                        const cv::Mat& distCoeffs);

  static void loadData(const std::string& path);

 private:

    static std::vector<GamePieceData> m_game_piece_data;
    static bool m_data_loaded;

    static std::vector<GamePieceData> find_matching_rows(const ObjDetectObservation& observation,
                                                       int& used_tolerance,
                                                       int start_tol = 25,
                                                       int max_tol = 40,
                                                       int step = 2);
};
