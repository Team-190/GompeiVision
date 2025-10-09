#include "estimator/ObjectEstimator.h"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation3d.h>

std::vector<GamePieceData> ObjectEstimator::m_game_piece_data;
bool ObjectEstimator::m_data_loaded = false;

void ObjectEstimator::loadData(const std::string& path) {
    if (m_data_loaded) {
        return;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Could not open game piece data file: " << path << std::endl;
        return;
    }

    m_game_piece_data.clear();
    std::string line;
    // Skip header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        GamePieceData data;

        std::getline(ss, cell, ',');
        data.class_id = std::stoi(cell); // Load class_id
        std::getline(ss, cell, ',');
        data.center_x = std::stod(cell);
        std::getline(ss, cell, ',');
        data.center_y = std::stod(cell);
        std::getline(ss, cell, ',');
        data.width = std::stod(cell);
        std::getline(ss, cell, ',');
        data.height = std::stod(cell);
        std::getline(ss, cell, ',');
        data.x_position = std::stod(cell);
        std::getline(ss, cell, ',');
        data.y_position = std::stod(cell);

        m_game_piece_data.push_back(data);
    }
    m_data_loaded = true;
    std::cout << "Loaded " << m_game_piece_data.size() << " rows of game piece data." << std::endl;
}

std::vector<GamePieceData> ObjectEstimator::find_matching_rows( //TODO: Fix later
    const ObjDetectObservation& observation, int& used_tolerance,
    int start_tol, int max_tol, int step) {
            return {};
}
void ObjectEstimator::calculate(ObjDetectObservation& observation,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distCoeffs) {
  if (observation.corner_pixels.size() != 8) {
    return; // Must be a 4-corner bounding box
  }

  // 1. Get raw corner pixels for this object
  std::vector<cv::Point2f> image_points;
  image_points.reserve(4);
  for (size_t j = 0; j < observation.corner_pixels.size(); j += 2) {
    image_points.emplace_back(observation.corner_pixels[j],
                              observation.corner_pixels[j + 1]);
  }
  
  // 2. Undistort corner points for accurate angle calculation
  std::vector<cv::Point2f> undistorted_points;
  cv::undistortPoints(image_points, undistorted_points, cameraMatrix,
                      distCoeffs, cv::noArray(), cameraMatrix);

  // 3. Calculate viewing angles from undistorted points
  const cv::Mat invCameraMatrix = cameraMatrix.inv();
  observation.corner_angles.reserve(8);

  for (const auto& corner : undistorted_points) {
    cv::Mat p(3, 1, CV_64F);
    p.at<double>(0, 0) = corner.x;
    p.at<double>(1, 0) = corner.y;
    p.at<double>(2, 0) = 1.0;

    cv::Mat vec = invCameraMatrix * p;

    observation.corner_angles.push_back(std::atan(vec.at<double>(0, 0))); // Angle X
    observation.corner_angles.push_back(std::atan(vec.at<double>(1, 0))); // Angle Y
  }


  // Estimate distance and pose (this part is now class-aware)
  if (m_data_loaded) {
    int used_tolerance = 0;
    std::vector<GamePieceData> matching_rows = find_matching_rows(observation, used_tolerance);

    if (!matching_rows.empty()) {
        double avg_x = 0;
        double avg_y = 0;
        for (const auto& row : matching_rows) {
            avg_x += row.x_position;
            avg_y += row.y_position;
        }
        avg_x /= matching_rows.size();
        avg_y /= matching_rows.size();

        observation.pose = frc::Pose3d(
            units::meter_t{avg_x},
            units::meter_t{avg_y},
            0_m,
            frc::Rotation3d()
        );
        observation.distance = std::sqrt(avg_x * avg_x + avg_y * avg_y);

    } else {
        observation.distance = 0;
    }
  }
}