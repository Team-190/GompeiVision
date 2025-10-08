#pragma once

#include <opencv2/dnn.hpp>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

#include "util/QueuedFrame.h"
#include "util/QueuedObjectData.h"

/**
 * @class ObjectDetector
 * @brief Finds the 2D locations of specified objects in an image using a
 * neural network.
 *
 * This class is responsible for running an inference model to detect objects,
 * extracting their bounding boxes, class IDs, and confidences. 
 */
class ObjectDetector {
 public:
  /**
   * @brief Constructs the ObjectDetector and loads the ONNX model.
   * @param model_path The file path to the .onnx object detection model.
   * @param class_names_path The file path to the list of class names.
   */
  ObjectDetector(const std::string& model_path,
                 const std::string& class_names_path);
  ~ObjectDetector();

  // Disable copy and move semantics for resource safety.
  ObjectDetector(const ObjectDetector&) = delete;
  ObjectDetector& operator=(const ObjectDetector&) = delete;

  /**
   * @brief Detects all objects in a given frame.
   * @param frame The input image from the camera.
   * @param result A reference to the result object to be populated with
   * raw 2D detections.
   */
  void detect(const QueuedFrame& frame,
              std::vector<ObjDetectObservation>& observations);
  /**
   * @brief Gets the class names used by the model.
   * @return A constant reference to the vector of class name strings.
   */
  const std::vector<std::string>& getClassNames() const;


 private:
  cv::dnn::Net m_net;
  std::vector<std::string> m_class_names;

  // --- Model Parameters (CHANGE IF NEEDED) ---
  const float m_input_width = 640.0;
  const float m_input_height = 640.0;
  const float m_confidence_threshold = 0.5;
  const float m_nms_threshold = 0.4;

  // Logging helpers to maintain consistency.
  void logInfo(const std::string& message) const;
  void logError(const std::string& message) const;
};
