#include "estimator/ObjectEstimator.h"

#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <vector>

static std::vector<GamePieceRow> LoadGamePieceTable() {
  std::vector<GamePieceRow> table;
  // Ensure this path matches where you deploy the file on the robot/coprocessor
  std::string path = "/usr/local/share/GompeiVision/gamepiece_table.csv";
  std::ifstream file(path);

  if (!file.is_open()) {
    std::cerr << "Failed to open " << path << std::endl;
    return table;
  }

  std::string line;
  // Skip the header line (assuming the first line contains "Image,x_center...")
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::istringstream iss(line);
    std::string image_name_dummy;  // To hold the "Image" column (filename)
    char comma;
    double image_angle_dummy;  // To hold "Image_angle"
    GamePieceRow row;

    // 1. Read the filename string up to the comma
    if (std::getline(iss, image_name_dummy, ',') &&
        iss >> row.centerX >> comma >> row.centerY >> comma >> row.width >>
            comma >> row.height >> comma >> image_angle_dummy >> comma >>
            row.xPosition >> comma >> row.yPosition >> comma >> row.angle) {
      table.push_back(row);
    }
  }

  return table;
}

ObjectEstimator* ObjectEstimator::instance = nullptr;

ObjectEstimator& ObjectEstimator::GetInstance(const float width,
                                              const float height) {
  if (instance == nullptr) {
    instance = new ObjectEstimator(width, height);
  }
  return *instance;
}

ObjectEstimator::ObjectEstimator(const float width, const float height)
    : posEstimator(width, height, LoadGamePieceTable()) {}

void ObjectEstimator::calculate(ObjDetectObservation& observation,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distCoeffs) const {
  // Must have exactly 4 corners
  if (observation.corner_pixels.size() != 8) {
    return;
  }

  // 1. Collect distorted corner pixels
  std::vector<cv::Point2f> image_points;
  image_points.reserve(4);

  for (size_t i = 0; i < 8; i += 2) {
    image_points.emplace_back(observation.corner_pixels[i],
                              observation.corner_pixels[i + 1]);
  }

  // 2. Undistort corner pixels into pixel space
  std::vector<cv::Point2f> undistorted_points;
  cv::undistortPoints(image_points, undistorted_points, cameraMatrix,
                      distCoeffs, cv::noArray(), cameraMatrix);

  // 3. Compute bounding box in undistorted space
  cv::Rect2f bbox = cv::boundingRect(undistorted_points);

  // Reject degenerate boxes
  if (bbox.width <= 1.0f || bbox.height <= 1.0f) {
    return;
  }

  // 4. Estimate field position using lookup table
  auto estimate =
      posEstimator.estimatePosition(bbox.x, bbox.y, bbox.width, bbox.height);

  if (!estimate.has_value()) {
    return;
  }

  auto [position, certainty] = estimate.value();

  // 5. Write results to observation
  observation.pose = position;

  observation.confidence = certainty;
}
