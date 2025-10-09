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
            observation.distance = -9;
}
