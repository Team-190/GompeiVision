#include "detector/ObjectDetector.h"

#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
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
  m_net = cv::dnn::readNet(model_path);
  if (m_net.empty()) {
    logError("Failed to load object detection model from: " + model_path);
  } else {
    logInfo("ONNX model loaded successfully.");
    // Set backend and target for better performance (optional but recommended)

    try {
      m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
    } catch (const cv::Exception& e) {
      std::cerr << "[WARN] OpenVINO backend unavailable, using default CPU "
                   "backend.\n";
      m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    }

    try {
      // Try GPU (Iris Xe)
      if (true) { // Assume FP16 for now
        m_net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
        logInfo("Using OpenVINO GPU backend (FP16, OpenCL_FP16).");
      } else {
        m_net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);  // fallback FP32
        logInfo("Using OpenVINO GPU backend (FP32, OpenCL).");
      }
    } catch (const cv::Exception& e) {
      logInfo("[WARN] GPU backend unavailable, using OpenVINO CPU backend.");
      m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
  }
}

ObjectDetector::~ObjectDetector() { logInfo("Shutting down..."); }

void ObjectDetector::detect(const QueuedFrame& q_frame,
                            std::vector<ObjDetectObservation>& observations,
                            std::vector<cv::Rect>& final_boxes) {
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
    return; // Exit the detect function early
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

  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

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

  // Apply Non-Maximum Suppression to filter out overlapping boxes
  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, m_confidence_threshold, m_nms_threshold,
                    nms_result);

  // Clear output vectors before populating
  observations.clear();
  final_boxes.clear();

  for (int idx : nms_result) {
    ObjDetectObservation obs;
    obs.obj_class = class_ids[idx];
    obs.confidence = confidences[idx];

    const auto& box = boxes[idx];
    // Define the 4 corners of the bounding box
    obs.corner_pixels = {
        (double)box.tl().x, (double)box.tl().y,  // Top-Left
        (double)box.br().x, (double)box.tl().y,  // Top-Right
        (double)box.br().x, (double)box.br().y,  // Bottom-Right
        (double)box.tl().x, (double)box.br().y   // Bottom-Left
    };
    observations.push_back(obs);
    final_boxes.push_back(box);  // Populate the final boxes vector
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