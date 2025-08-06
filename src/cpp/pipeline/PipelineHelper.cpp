#include "pipeline/PipelineHelper.h"

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <networktables/NetworkTableType.h>
#include <units/length.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "io/ConfigInterface.h"

PipelineHelper::PipelineHelper() {}
PipelineHelper::~PipelineHelper() {}

bool PipelineHelper::load_camera_intrinsics(
    const ConfigInterface& configInterface, cv::Mat& cameraMatrix,
    cv::Mat& distortionCoefficients) {
  cameraMatrix = configInterface.getCameraMatrix();
  distortionCoefficients = configInterface.getDistortionCoeffs();

  if (cameraMatrix.empty() || distortionCoefficients.empty()) {
    std::cerr << "[PipelineHelper] ERROR: Camera intrinsics (matrix or "
                 "distortion coefficients) are empty."
              << std::endl;
    return false;
  }

  return true;
}

std::map<int, frc::Pose3d> PipelineHelper::load_field_layout() {
  std::map<int, frc::Pose3d> layout;
  const auto inst = nt::NetworkTableInstance::GetDefault();
  const auto fields_table = inst.GetTable("field");

  if (!fields_table) {
    std::cerr << "[PipelineHelper] ERROR: Could not find 'field' table in "
                 "NetworkTables."
              << std::endl;
    return layout;
  }

  for (const auto& topic : fields_table->GetKeys()) {
    std::cout << "tag: " << topic << std::endl;
    if (topic.rfind("tag_", 0) == 0) {
      try {
        int tag_id = std::stoi(topic.substr(4));

        if (auto data = fields_table->GetEntry(topic).GetDoubleArray({});
            data.size() == 7) {
          const frc::Translation3d translation{units::meter_t{data[0]},
                                               units::meter_t{data[1]},
                                               units::meter_t{data[2]}};
          frc::Quaternion quaternion{data[3], data[4], data[5], data[6]};

          layout.emplace(tag_id,
                         frc::Pose3d{translation, frc::Rotation3d(quaternion)});

        } else {
          std::cerr << "[PipelineHelper] WARN: Topic '" << topic
                    << "' has incorrect data size (" << data.size()
                    << "), expected 7." << std::endl;
        }
      } catch (const std::invalid_argument& e) {
        std::cerr
            << "[PipelineHelper] WARN: Could not parse tag ID from topic '"
            << topic << "'." << std::endl;
      }
    }
  }

  if (layout.empty()) {
    std::cerr << "[PipelineHelper] WARN: No valid tag poses found in "
                 "'field' table."
              << std::endl;
  }

  return layout;
}
