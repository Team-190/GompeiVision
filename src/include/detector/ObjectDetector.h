#pragma once

#include <string>
#include <vector>

#if USE_OPENVINO
#include <openvino/openvino.hpp>
#include <openvino/runtime/properties.hpp>
#endif

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "util/QueuedFrame.h"
#include "util/QueuedObjectData.h"

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
   * @param observations A reference to the result object to be populated.
   * @param final_boxes A vector to be populated with the cv::Rect for each
   * observation.
   */
  void detect(const QueuedFrame& frame,
              std::vector<ObjDetectObservation>& observations,
              std::vector<cv::Rect>& final_boxes,
              std::vector<int>& final_class_ids,
              std::vector<float>& final_confidences);

  /**
   * @brief Gets the class names used by the model.
   * @return A constant reference to the vector of class name strings.
   */
  const std::vector<std::string>& getClassNames() const;

 private:
  void logInfo(const std::string& message) const;
  void logError(const std::string& message) const;

#if USE_OPENVINO
  ov::Core m_core;
  ov::CompiledModel m_compiled_model;
  ov::InferRequest infer_request;
  ov::Shape input_shape;
  ov::element::Type input_element_type;

#else
  cv::dnn::Net m_net;
#endif
      std::vector<std::string>
          m_class_names;

  // MODEL PARAMETERS
  const float m_input_width = 640;
  const float m_input_height = 640;
  const float m_confidence_threshold = 0.5f;
  const float m_nms_threshold = 0.4f;
};
