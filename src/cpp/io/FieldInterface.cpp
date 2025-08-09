#include "io/FieldInterface.h"

#include <networktables/NetworkTableInstance.h>

#include <iostream>

FieldInterface::FieldInterface() {
  const auto field = nt::NetworkTableInstance::GetDefault().GetTable("/field");
  const auto tag = field->GetEntry("tag_1").GetDoubleArray({}).size();
  std::cout << "size: " << tag << std::endl;
  update();
}

FieldInterface::~FieldInterface() {}

void FieldInterface::update() {
  for (int i = 0; i < 22; i++) {
    int tag_id = i + 1;
    std::cout << "entry: " << "tag_" + std::to_string(tag_id) << std::endl;
    if (std::vector<double> pose_data =
            nt::NetworkTableInstance::GetDefault()
                .GetTable("field")
                ->GetEntry("tag_" + std::to_string(tag_id))
                .GetDoubleArray({});
        pose_data.size() == 7) {
      // Pose3d: x, y, z, qw, qx, qy, qz
      const frc::Translation3d t{units::meter_t{pose_data[0]},
                                 units::meter_t{pose_data[1]},
                                 units::meter_t{pose_data[2]}};
      const frc::Rotation3d r{frc::Quaternion{pose_data[3], pose_data[4],
                                              pose_data[5], pose_data[6]}};

      tagMap.emplace(tag_id, frc::Pose3d{t, r});
    } else {
      std::cerr << "FieldInterface: Received pose data of unexpected size ("
                << pose_data.size() << ") for tag " << tag_id << std::endl;
    }
  }

  std::cout << "Total tags: " << tagMap.size() << std::endl;
}

std::map<int, frc::Pose3d> FieldInterface::getMap() { return tagMap; }