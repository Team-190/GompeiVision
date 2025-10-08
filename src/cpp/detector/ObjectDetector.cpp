#include "detector/ObjectDetector.h"

#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <vector>

ObjectDetector::ObjectDetector(const std::string& model_path,
                               const std::string& class_names_path) {
  logInfo("Initializing...");
  logInfo("DEBUG: Model path: " + model_path);
  logInfo("DEBUG: Class names path: " + class_names_path);

  // --- Load Class Names ---
  std::ifstream ifs(class_names_path);
  if (ifs.is_open()) {
    std::string line;
    while (std::getline(ifs, line)) {
      m_class_names.push_back(line);
      logInfo("DEBUG: Loaded class: " + line);
    }
    logInfo("Loaded " + std::to_string(m_class_names.size()) + " class names.");
  } else {
    logError("Could not open class names file: " + class_names_path);
  }

  // --- Load ONNX Neural Network Model ---
  logInfo("DEBUG: Loading ONNX model...");
  m_net = cv::dnn::readNet(model_path);
  if (m_net.empty()) {
    logError("Failed to load object detection model from: " + model_path);
  } else {
    logInfo("ONNX model loaded successfully.");
    logInfo("DEBUG: Setting backend and target...");
    // Set backend and target for better performance (optional but recommended)
    m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    logInfo("DEBUG: Backend/target configuration complete");
  }
}

ObjectDetector::~ObjectDetector() { 
  logInfo("Shutting down..."); 
  logInfo("DEBUG: ObjectDetector destructor called");
}

void ObjectDetector::detect(
    const QueuedFrame& q_frame,
    std::vector<ObjDetectObservation>& observations) {
  
  logInfo("DEBUG: detect() method called");
  
  if (q_frame.frame.empty() || m_net.empty()) {
    if (q_frame.frame.empty()) {
      logError("DEBUG: Input frame is empty!");
    }
    if (m_net.empty()) {
      logError("DEBUG: Neural network is not loaded!");
    }
    return;
  }

  logInfo("DEBUG: Frame dimensions: " + std::to_string(q_frame.frame.cols) + "x" + std::to_string(q_frame.frame.rows));

  // --- 1. Preprocess Frame ---
  logInfo("DEBUG: Starting preprocessing...");
  cv::Mat blob;
  cv::dnn::blobFromImage(q_frame.frame, blob, 1. / 255.,
                         cv::Size(m_input_width, m_input_height), cv::Scalar(),
                         true, false);
  logInfo("DEBUG: Blob created with input size: " + std::to_string(m_input_width) + "x" + std::to_string(m_input_height));

  // --- 2. Perform Inference ---
  logInfo("DEBUG: Setting network input...");
  m_net.setInput(blob);
  logInfo("DEBUG: Running forward pass...");
  std::vector<cv::Mat> outs;
  m_net.forward(outs, m_net.getUnconnectedOutLayersNames());
  logInfo("DEBUG: Forward pass complete, got " + std::to_string(outs.size()) + " output matrices");

  // --- 3. Post-process Results for YOLOv8/v11 ---
  logInfo("DEBUG: Starting post-processing...");
  // The output of YOLOv8 is a single matrix of shape [1, 84, 8400]
  // where 84 = 4 (bbox) + 80 (class scores)
  // and 8400 is the number of detections.
  // We need to transpose it to [1, 8400, 84] for easier processing.
  cv::Mat detection_matrix = outs[0];
  logInfo("DEBUG: Detection matrix dimensions: " + std::to_string(detection_matrix.dims) + "D");
  cv::Mat detections = detection_matrix.reshape(1, {detection_matrix.size[1], detection_matrix.size[2]}).t();
  logInfo("DEBUG: Reshaped detections: " + std::to_string(detections.rows) + " rows x " + std::to_string(detections.cols) + " cols");

  float x_factor = q_frame.frame.cols / m_input_width;
  float y_factor = q_frame.frame.rows / m_input_height;
  logInfo("DEBUG: Scale factors - X: " + std::to_string(x_factor) + ", Y: " + std::to_string(y_factor));

  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  int detections_above_threshold = 0;
  logInfo("DEBUG: Processing " + std::to_string(detections.rows) + " detections with confidence threshold: " + std::to_string(m_confidence_threshold));

  for (int i = 0; i < detections.rows; ++i) {
    // For each detection, find the class with the highest score
    cv::Mat classes_scores = detections.row(i).colRange(4, 4 + m_class_names.size());
    cv::Point class_id_point;
    double max_class_score;
    cv::minMaxLoc(classes_scores, 0, &max_class_score, 0, &class_id_point);

    if (max_class_score > m_confidence_threshold) {
      detections_above_threshold++;
      logInfo("DEBUG: Detection " + std::to_string(detections_above_threshold) + 
              " - Class ID: " + std::to_string(class_id_point.x) + 
              ", Confidence: " + std::to_string(max_class_score));
      
      confidences.push_back(max_class_score);
      class_ids.push_back(class_id_point.x);

      // Extract bounding box coordinates (center_x, center_y, width, height)
      float cx = detections.at<float>(i, 0);
      float cy = detections.at<float>(i, 1);
      float w = detections.at<float>(i, 2);
      float h = detections.at<float>(i, 3);

      logInfo("DEBUG: Raw bbox - center: (" + std::to_string(cx) + ", " + std::to_string(cy) + 
              "), size: " + std::to_string(w) + "x" + std::to_string(h));

      // Convert from center coordinates to top-left corner
      int left = static_cast<int>((cx - 0.5 * w) * x_factor);
      int top = static_cast<int>((cy - 0.5 * h) * y_factor);
      int width = static_cast<int>(w * x_factor);
      int height = static_cast<int>(h * y_factor);
      
      logInfo("DEBUG: Scaled bbox - position: (" + std::to_string(left) + ", " + std::to_string(top) + 
              "), size: " + std::to_string(width) + "x" + std::to_string(height));
      
      boxes.emplace_back(left, top, width, height);
    }
  }

  logInfo("DEBUG: Found " + std::to_string(detections_above_threshold) + " detections above threshold");

  // Apply Non-Maximum Suppression to filter out overlapping boxes
  logInfo("DEBUG: Applying NMS with threshold: " + std::to_string(m_nms_threshold));
  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, m_confidence_threshold, m_nms_threshold,
                    nms_result);
  logInfo("DEBUG: NMS kept " + std::to_string(nms_result.size()) + " detections");

  for (int idx : nms_result) {
    ObjDetectObservation obs;
    obs.obj_class = class_ids[idx];
    obs.confidence = confidences[idx];
    
    const auto& box = boxes[idx];
    // Define the 4 corners of the bounding box
    obs.corner_pixels = {
      (double)box.tl().x, (double)box.tl().y, // Top-Left
      (double)box.br().x, (double)box.tl().y, // Top-Right
      (double)box.br().x, (double)box.br().y, // Bottom-Right
      (double)box.tl().x, (double)box.br().y  // Bottom-Left
    };
    
    logInfo("DEBUG: Final observation - Class: " + std::to_string(obs.obj_class) + 
            ", Confidence: " + std::to_string(obs.confidence) + 
            ", Corner count: " + std::to_string(obs.corner_pixels.size()));
    
    observations.push_back(obs);
  }
  
  logInfo("DEBUG: detect() returning " + std::to_string(observations.size()) + " final observations");
}

void ObjectDetector::logInfo(const std::string& message) const {
  std::cout << "[ObjectDetector INFO] " << message << std::endl;
}

void ObjectDetector::logError(const std::string& message) const {
  std::cerr << "[ObjectDetector ERROR] " << message << std::endl;
}
