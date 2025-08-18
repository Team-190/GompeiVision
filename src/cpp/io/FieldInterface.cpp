#include "io/FieldInterface.h"
#include <iostream>
#include "networktables/NetworkTableInstance.h"

void FieldInterface::initialize(const nt::NetworkTableInstance& nt_inst) {
  subscribers.clear();
  tagMap.clear();

  const auto fieldTable = nt_inst.GetTable("field");

  // Create subscribers for all tags
  for (int i = 0; i < 22; i++) {
    int tag_id = i + 1;
    auto sub = fieldTable->GetDoubleArrayTopic("tag_" + std::to_string(tag_id))
                   .Subscribe({});
    subscribers.emplace(tag_id, std::move(sub));
  }

  numTags = 22; // we know how many tags to expect
}

bool FieldInterface::update() {
  tagMap.clear();

  for (const auto& [tag_id, sub] : subscribers) {
    if (std::vector<double> pose_data = sub.Get(); pose_data.size() == 7) {
      const frc::Translation3d t{
        units::meter_t{pose_data[0]},
        units::meter_t{pose_data[1]},
        units::meter_t{pose_data[2]}};

      const frc::Rotation3d r{
        frc::Quaternion{pose_data[3], pose_data[4],
                        pose_data[5], pose_data[6]}};

      tagMap.emplace(tag_id, frc::Pose3d{t, r});

    } else {
      // std::cerr << "FieldInterface: Received pose data of unexpected size ("
      //           << pose_data.size() << ") for tag " << tag_id << std::endl;
    }
  }

  return tagMap.size() == numTags;
}

std::map<int, frc::Pose3d> FieldInterface::getMap() {
  return tagMap;
}

bool FieldInterface::isInitialized() {
  return !subscribers.empty();
}