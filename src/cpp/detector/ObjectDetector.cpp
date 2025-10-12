#include "detector/ObjectDetector.h"

#include <fstream>
#include <iostream>
#include <vector>

ObjectDetector::ObjectDetector(const std::string& model_path,
                               const std::string& class_names_path) {
  logInfo("Initializing...");

  // --- Load Class Names ---
  std::ifstream ifs(class_names_path);
  if (ifs.is_open()) {
    std::string line;
    while (std::getline(ifs, line)) {
      m_class_names.push_back(line);
    }
    logInfo("Loaded " + std::to_string(m_class_names.size()) + " class names.");
  } else {
    logError("Could not open class names file: " + class_names_path);
  }

  // --- Load ONNX Neural Network Model ---
#if USE_OPENVINO
  // --- Use OpenVINO Runtime on x86 ---
  logInfo("Using OpenVINO runtime.");
  try {
    // Load the model from the .onnx file
    m_core = ov::Core();
    auto model = m_core.read_model(model_path);

    // --- Preprocessing (move here later) ---
    // Can build preprocessing steps into the model here
    // For now, handle resizing in the detect loop.

    // Compile the model for the optimal device (e.g., CPU, GPU)
    // 'AUTO' lets OpenVINO choose the best available device.
    m_compiled_model = m_core.compile_model(model, "AUTO");

    auto input_port = m_compiled_model.input();
    input_shape = input_port.get_shape();
    input_element_type = input_port.get_element_type();

    logInfo("OpenVINO model compiled successfully for AUTO device.");

    // Get input shape information
    ov::InferRequest infer_request = m_compiled_model.create_infer_request();
    m_compiled_model.set_property(
        {{ov::hint::performance_mode, ov::hint::PerformanceMode::LATENCY}});
  } catch (const ov::Exception& e) {
    logError("OpenVINO exception: " + std::string(e.what()));
  } catch (const std::exception& e) {
    logError("Standard exception: " + std::string(e.what()));
  }

#else
  // --- Use OpenCV DNN on ARM ---
  m_net = cv::dnn::readNet(model_path);
  if (m_net.empty()) {
    logError("Failed to load object detection model from: " + model_path);
  } else {
    logInfo("ONNX model loaded successfully with OpenCV DNN.");
    // Set backend for ARM
    m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }
#endif
}

ObjectDetector::~ObjectDetector() { logInfo("Shutting down..."); }

void ObjectDetector::detect(const QueuedFrame& q_frame,
                            std::vector<ObjDetectObservation>& observations,
                            std::vector<cv::Rect>& final_boxes,
                            std::vector<int>& final_class_ids,
                            std::vector<float>& final_confidences) {
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

#if USE_OPENVINO
  if (q_frame.frame.empty() || !m_compiled_model) {
    return;
  }
  // --- 2. Preprocess and Set Input Tensor ---

  // Resize the frame to match the model's input size
  cv::Mat blob;
  cv::dnn::blobFromImage(q_frame.frame, blob, 1. / 255.,
                         cv::Size(m_input_width, m_input_height), cv::Scalar(),
                         true, false);

  // Create an OpenVINO tensor from the preprocessed cv::Mat data
  // This tensor will share data with the cv::Mat, avoiding a copy
  ov::Tensor input_tensor(input_element_type, input_shape, blob.ptr<float>());


  infer_request.set_input_tensor(input_tensor);

  // --- 3. Perform Inference ---
  infer_request.infer();

  // --- 4. Post-process Results ---
  const ov::Tensor& output_tensor = infer_request.get_output_tensor();
  const float* detections = output_tensor.data<const float>();

  // The output shape for YOLOv8 is [1, 84, 8400]
  // where 84 = 4 (bbox) + 80 (class scores)
  // We need to process this flat data structure.
  const int num_classes = m_class_names.size();
  const int elements_per_detection = num_classes + 4;  // 4 for bbox
  const int num_detections = output_tensor.get_shape()[2];

  float x_factor = q_frame.frame.cols / m_input_width;
  float y_factor = q_frame.frame.rows / m_input_height;

  // Transpose the data from [1, 84, 8400] to [1, 8400, 84] for easier iteration
  cv::Mat output_matrix(elements_per_detection, num_detections, CV_32F,
                        (void*)detections);
  cv::Mat detections_mat = output_matrix.t();

  for (int i = 0; i < detections_mat.rows; ++i) {
    // For each detection, find the class with the highest score
    cv::Mat classes_scores = detections_mat.row(i).colRange(4, 4 + num_classes);
    cv::Point class_id_point;
    double max_class_score;
    cv::minMaxLoc(classes_scores, 0, &max_class_score, 0, &class_id_point);

    if (max_class_score > m_confidence_threshold) {
      confidences.push_back(max_class_score);
      class_ids.push_back(class_id_point.x);

      // Extract bounding box coordinates (center_x, center_y, width, height)
      float cx = detections_mat.at<float>(i, 0);
      float cy = detections_mat.at<float>(i, 1);
      float w = detections_mat.at<float>(i, 2);
      float h = detections_mat.at<float>(i, 3);

      // Convert from center coordinates to top-left corner
      int left = static_cast<int>((cx - 0.5 * w) * x_factor);
      int top = static_cast<int>((cy - 0.5 * h) * y_factor);
      int width = static_cast<int>(w * x_factor);
      int height = static_cast<int>(h * y_factor);
      boxes.emplace_back(left, top, width, height);
    }
  }

#else
  // --- ARM IMPLEMENTATION (Original OpenCV DNN) ---
  if (q_frame.frame.empty() || m_net.empty()) {
    return;
  }

  // --- 1. Preprocess Frame ---
  cv::Mat blob;
  cv::dnn::blobFromImage(q_frame.frame, blob, 1. / 255.,
                         cv::Size(m_input_width, m_input_height), cv::Scalar(),
                         true, false);

  // --- 2. Perform Inference ---
  m_net.setInput(blob);

  std::vector<cv::Mat> outs;
  m_net.forward(outs, m_net.getUnconnectedOutLayersNames());

  if (outs.empty()) {
    logError("Inference failed: model returned no outputs.");
    return;  // Exit the detect function early
  }

  // --- 3. Post-process Results for YOLOv8/v11 ---
  // The output of YOLOv8 is a single matrix of shape [1, 84, 8400]
  // where 84 = 4 (bbox) + 80 (class scores)
  // and 8400 is the number of detections.
  // We need to transpose it to [1, 8400, 84] for easier processing.
  cv::Mat detection_matrix = outs[0];
  cv::Mat detections =
      detection_matrix
          .reshape(1, {detection_matrix.size[1], detection_matrix.size[2]})
          .t();

  float x_factor = q_frame.frame.cols / m_input_width;
  float y_factor = q_frame.frame.rows / m_input_height;

  for (int i = 0; i < detections.rows; ++i) {
    // For each detection, find the class with the highest score
    cv::Mat classes_scores =
        detections.row(i).colRange(4, 4 + m_class_names.size());
    cv::Point class_id_point;
    double max_class_score;
    cv::minMaxLoc(classes_scores, 0, &max_class_score, 0, &class_id_point);

    if (max_class_score > m_confidence_threshold) {
      confidences.push_back(max_class_score);
      class_ids.push_back(class_id_point.x);

      // Extract bounding box coordinates (center_x, center_y, width, height)
      float cx = detections.at<float>(i, 0);
      float cy = detections.at<float>(i, 1);
      float w = detections.at<float>(i, 2);
      float h = detections.at<float>(i, 3);

      // Convert from center coordinates to top-left corner
      int left = static_cast<int>((cx - 0.5 * w) * x_factor);
      int top = static_cast<int>((cy - 0.5 * h) * y_factor);
      int width = static_cast<int>(w * x_factor);
      int height = static_cast<int>(h * y_factor);
      boxes.emplace_back(left, top, width, height);
    }
  }
#endif

  // Apply Non-Maximum Suppression to filter out overlapping boxes
  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, m_confidence_threshold, m_nms_threshold,
                    nms_result);

  observations.clear();
  final_boxes.clear();
  final_class_ids.clear();
  final_confidences.clear();

  observations.reserve(nms_result.size());
  final_boxes.reserve(nms_result.size());
  final_class_ids.reserve(nms_result.size());
  final_confidences.reserve(nms_result.size());

  for (int idx : nms_result) {
    const auto& box = boxes[idx];
    final_boxes.push_back(box);
    final_class_ids.push_back(class_ids[idx]);
    final_confidences.push_back(confidences[idx]);

    observations.emplace_back(
        class_ids[idx], confidences[idx], static_cast<double>(box.tl().x),
        static_cast<double>(box.tl().y), static_cast<double>(box.br().x),
        static_cast<double>(box.tl().y), static_cast<double>(box.br().x),
        static_cast<double>(box.br().y), static_cast<double>(box.tl().x),
        static_cast<double>(box.br().y));
  }
}

void ObjectDetector::logInfo(const std::string& message) const {
  std::cout << "[ObjectDetector INFO] " << message << std::endl;
}

void ObjectDetector::logError(const std::string& message) const {
  std::cerr << "[ObjectDetector ERROR] " << message << std::endl;
}

const std::vector<std::string>& ObjectDetector::getClassNames() const {
  return m_class_names;
}