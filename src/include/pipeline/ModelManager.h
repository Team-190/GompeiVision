#pragma once

#include <string>
#include <vector>

// Define a structure to hold the paths for a single model.
struct ObjectModel {
  std::string modelPath;
  std::string namesPath;
};

// Define the global, constant list of all available models.
inline const std::vector<ObjectModel> available_models = {
    // Index 0: Your primary model for reefscape objects
    {"/usr/share/GompeiVision/reefscape_yolo11n.onnx",
     "/usr/share/GompeiVision/reefscape.names"},
    // To add a new model, just add a new line here.
};