#include "pipeline/Pipeline.h"

#include <cscore/cscore_cv.h>
#include <httplib.h>
#include <libudev.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "capture/Camera.h"
#include "estimator/MultiTagPoseEstimator.h"
#include "estimator/SingleTagPoseEstimator.h"
#include "pipeline/PipelineHelper.h"

Pipeline::Pipeline(const int deviceIndex, const std::string& hardware_id,
                   const int width, const int height, const bool useMJPG,
                   const std::string& role, const int stream_port,
                   const int control_port)
    : m_hardware_id(hardware_id),
      m_role(role),
      m_control_port(control_port),
      m_stream_port(stream_port),
      m_mjpeg_server(role + "_stream", stream_port) {
  std::cout << "[" << m_role << "] Initializing pipeline..." << std::endl;

  m_camera = std::make_unique<Camera>(deviceIndex, hardware_id, width, height,
                                      useMJPG);

  if (!m_camera || !m_camera->isConnected()) {
    std::cerr << "[" << m_role << "] ERROR: Failed to create or connect Camera."
              << std::endl;
    return;
  }

  m_stream_width = m_camera->getWidth();
  m_stream_height = m_camera->getHeight();

  m_cv_source = std::make_unique<cs::CvSource>(
      role + "_source", cs::VideoMode::PixelFormat::kBGR, m_stream_width,
      m_stream_height, 30);

  CS_Status status = 0;
  cs::SetSinkSource(m_mjpeg_server.GetHandle(), m_cv_source.get()->GetHandle(),
                    &status);

  std::cout << "[" << m_role << "] MJPEG stream available at port "
            << m_stream_port << std::endl;

  m_field = PipelineHelper::load_field_layout("2025 Reefscape");
  m_frame_queue.setMaxQueue(10);
  if (auto res = m_camera->setExposure(100)) {
    std::cout << "[" << m_role << "] Exposure set to 50." << std::endl;
  } else {
    std::cout << "[" << m_role << "] Failed to set exposure" << std::endl;
  }
}

Pipeline::~Pipeline() {
  std::cout << "[" << m_role << "] Shutting down pipeline..." << std::endl;
  stop();
}

void Pipeline::start() {
  if (m_is_running) return;
  m_is_running = true;
  m_capture_thread = std::thread(&Pipeline::capture_loop, this);
  m_apriltag_detector_thread =
      std::thread(&Pipeline::apriltag_detector_loop, this);
  m_pose_estimator_thread = std::thread(&Pipeline::pose_estimator_loop, this);
  m_networktables_thread = std::thread(&Pipeline::networktables_loop, this);
  m_server_thread = std::thread(&Pipeline::server_loop, this);
}

void Pipeline::stop() {
  if (!m_is_running) return;
  m_is_running = false;

  // Unblock any threads that might be waiting on blocking operations like
  // queues or the server's listen call. This prevents deadlocks during
  // shutdown.
  m_frame_queue.shutdown();
  m_apriltag_detector_result_queue.shutdown();
  m_estimated_poses.shutdown();
  m_server.stop();

  // Now that all threads have been signaled to stop and unblocked, join them.
  if (m_capture_thread.joinable()) m_capture_thread.join();
  if (m_apriltag_detector_thread.joinable()) m_apriltag_detector_thread.join();
  if (m_pose_estimator_thread.joinable()) m_pose_estimator_thread.join();
  if (m_networktables_thread.joinable()) m_networktables_thread.join();
  if (m_server_thread.joinable()) m_server_thread.join();

  if (m_calibration_worker_thread.joinable())
    m_calibration_worker_thread.join();
  std::cout << "[" << m_role << "] All threads stopped." << std::endl;
}

void Pipeline::capture_loop() {
  cv::Mat frame;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;

  while (m_is_running) {
    if (m_camera && m_camera->getFrame(frame, timestamp)) {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
      std::lock_guard<std::mutex> lock(m_calibration_mutex);
      if (m_calibration_session) {
        m_calibration_session->process_frame(frame,
                                             m_capture_next_frame_for_calib);

        const std::vector<cv::Point2f> all_points =
            m_calibration_session->get_all_corners();
        for (const auto& point : all_points) {
          cv::circle(frame, point, 6, cv::Scalar(0, 255, 0), -1);
        }
      }
      {
        QueuedFrame queuedFrame;
        queuedFrame.frame = frame.clone();
        queuedFrame.timestamp = timestamp;
        {
          std::lock_guard<std::mutex> role_lock(m_role_mutex);
          queuedFrame.cameraRole = m_role;
        }
        m_frame_queue.push(queuedFrame);
      }
      if (m_cv_source) m_cv_source->PutFrame(frame);
    }
  }
}

void Pipeline::apriltag_detector_loop() {
  while (m_is_running) {
    // if (m_is_calibrating.load()) continue;
    QueuedFrame frame;
    if (!m_frame_queue.waitAndPop(frame)) {
      continue;  // Queue is empty and we are shutting down
    }

    FiducialImageObservation observation;
    observation.timestamp = frame.timestamp;
    m_AprilTagDetector.detect(frame, observation);
    if (!observation.tag_ids.empty()) {
      m_apriltag_detector_result_queue.push(observation);
    }
  }
}

void Pipeline::pose_estimator_loop() {
  // --- 1. Initialization ---
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  auto last_time = std::chrono::steady_clock::now();
  double smoothed_fps = 0.0;
  constexpr double alpha = 0.05;  // Smoothing factor for EMA

  // Load camera intrinsics from the calibration file.
  if (!PipelineHelper::load_camera_intrinsics(m_role, cameraMatrix,
                                              distCoeffs)) {
    std::cerr << "[" << m_role
              << "] ERROR: Cannot run Pose Estimator without valid "
                 "calibration. Thread is exiting."
              << std::endl;
    return;
  }

  // This should return a map of tag IDs to their 3D poses (cv::Pose3d).
  // For now, we'll use an empty map.

  // Define the 3D coordinates of a standard AprilTag's corners (size in meters)
  // The order (TL, TR, BR, BL) must match the detector's corner output.
  constexpr double tag_size_m = 0.1651;  // 6.5 inches
  const std::vector<cv::Point3f> standard_object_points = {
      {-tag_size_m / 2.0f, tag_size_m / 2.0f, 0.0f},  // Top-left
      {tag_size_m / 2.0f, tag_size_m / 2.0f, 0.0f},   // Top-right
      {tag_size_m / 2.0f, -tag_size_m / 2.0f, 0.0f},  // Bottom-right
      {-tag_size_m / 2.0f, -tag_size_m / 2.0f, 0.0f}  // Bottom-left
  };

  // --- 2. Main Processing Loop ---
  while (m_is_running) {
    if (m_is_calibrating.load()) continue;
    FiducialImageObservation frame_observation;
    if (!m_apriltag_detector_result_queue.waitAndPop(frame_observation)) {
      continue;  // Queue is empty and we are shutting down
    }

    // This structure will hold all results for the current frame.
    AprilTagResult result;
    result.timestamp = frame_observation.timestamp;
    result.camera_role = m_role;

    SingleTagPoseEstimator::estimatePose(frame_observation, result,
                                         cameraMatrix, distCoeffs, tag_size_m);

    MultiTagPoseEstimator::estimatePose(frame_observation, result, cameraMatrix,
                                        distCoeffs, tag_size_m, m_field);

    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - last_time;
    last_time = current_time;

    const double instant_fps = 1.0 / elapsed_seconds.count();
    smoothed_fps = (1.0 - alpha) * smoothed_fps + alpha * instant_fps;

    std::cout << "[" << m_role << "] FPS: " << smoothed_fps << std::endl;
    std::cout << "[" << m_role << "] Frames Queued: " << m_frame_queue.size()
              << " Detections Queued: "
              << m_apriltag_detector_result_queue.size() << std::endl;

    if (result.single_tag_poses.empty()) continue;

    // Store the smoothed FPS in your result
    result.fps = smoothed_fps;

    m_estimated_poses.push(result);
  }
}

void Pipeline::networktables_loop() {
  while (m_is_running) {
    // if (m_is_calibrating.load()) continue;
    AprilTagResult result;
    m_estimated_poses.waitAndPop(result);
  }
}

void Pipeline::server_loop() {
  std::cout << "[" << m_role << "] Control panel server thread started on port "
            << m_control_port << "." << std::endl;

  const std::string html_content =
      R"raw(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Pipeline: )raw" +
      m_role + R"raw(</title>
    <style>
        body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif; background-color: #f0f2f5; margin: 0; padding: 2em; }
        h1, h2 { color: #1c1e21; }
        .container { display: flex; flex-wrap: wrap; gap: 2em; }
        .stream-container { flex: 2; min-width: 320px; }
        .stream { border: 1px solid #dddfe2; border-radius: 8px; background-color: #000; max-width: 100%; height: auto; }
        .controls { flex: 1; min-width: 300px; background-color: #fff; padding: 1.5em; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        button { background-color: #1877f2; color: white; border: none; padding: 10px 15px; border-radius: 6px; cursor: pointer; font-size: 1em; margin-top: 10px; width: 100%; transition: background-color 0.2s; }
        button:hover { background-color: #166fe5; }
        button:disabled { background-color: #9cb4d8; cursor: not-allowed; }
        p { margin: 0.5em 0; }
        input[type="text"], input[type="number"] { box-sizing: border-box; width: 100%; padding: 8px; margin-bottom: 10px; border: 1px solid #ccc; border-radius: 4px; }
        #frameCount { font-weight: bold; color: #1877f2; font-size: 1.2em; }
        #statusMessage { font-weight: bold; color: #333; display: block; margin-top: 5px; min-height: 40px; }
        .form-grid { display: grid; grid-template-columns: auto 1fr; gap: 5px 10px; align-items: center; }
        .form-grid label { text-align: right; }
    </style>
</head>
<body>
    <h1 id="main-title">GompeiVision Pipeline: )raw" +
      m_role + R"raw(</h1>
    <div class="container">
        <div class="stream-container">
            <h2>Live Stream</h2>
            <img id="stream" class="stream" width=")raw" +
      std::to_string(m_stream_width) + R"raw(" height=")raw" +
      std::to_string(m_stream_height) +
      R"raw(" alt="Live MJPEG Stream">
        </div>
        <div class="controls">
            <!-- FIX: Added Role settings UI back -->
            <h2>Pipeline Settings</h2>
            <form id="roleForm">
                <div class="form-grid">
                    <label for="roleInput">Camera Role:</label>
                    <input type="text" id="roleInput" placeholder="e.g., front_cam" required>
                </div>
                <button type="submit">Set Role</button>
            </form>
            <hr style="margin: 20px 0; border: 1px solid #eee;">

            <h2>Calibration Control</h2>
            <form id="startForm">
                <div class="form-grid">
                    <label for="squaresX">Board Width:</label>
                    <input type="number" id="squaresX" value="11" required>
                    <label for="squaresY">Board Height:</label>
                    <input type="number" id="squaresY" value="8" required>
                    <label for="squareLength">Square (m):</label>
                    <input type="number" id="squareLength" value="0.025" step="0.001" required>
                    <label for="markerLength">Marker (m):</label>
                    <input type="number" id="markerLength" value="0.018" step="0.001" required>
                </div>
                <button type="submit">Start New Calibration</button>
            </form>
            <hr style="margin: 20px 0; border: 1px solid #eee;">
            <p>Frames Captured: <span id="frameCount">0</span></p>
            <p>Status: <br><span id="statusMessage">N/A</span></p>
            <button id="captureBtn">Capture Frame</button>
            <button id="finishBtn">Finish & Save</button>
        </div>
    </div>

    <script>
        document.addEventListener('DOMContentLoaded', () => {
            const streamImg = document.getElementById('stream');
            streamImg.src = `http://${window.location.hostname}:)raw" +
      std::to_string(m_stream_port) + R"raw(/stream.mjpg`;

            // FIX: Added event listener for the new role form
            document.getElementById('roleForm').onsubmit = handleSetRole;
            document.getElementById('startForm').onsubmit = handleStartCalibration;
            document.getElementById('captureBtn').onclick = () => handlePost('/calibration/capture');
            document.getElementById('finishBtn').onclick = () => handlePost('/calibration/finish');

            setInterval(updateStatus, 1500); // Poll for status updates
        });

        const frameCountSpan = document.getElementById('frameCount');
        const statusSpan = document.getElementById('statusMessage');
        const finishBtn = document.getElementById('finishBtn');
        const mainTitle = document.getElementById('main-title');

        // FIX: Added handler function for setting the role
        function handleSetRole(event) {
            event.preventDefault();
            const roleInput = document.getElementById('roleInput');
            const newRole = roleInput.value.trim();
            if (!newRole) {
                statusSpan.textContent = 'Role cannot be empty.';
                return false;
            }
            statusSpan.textContent = `Setting role to ${newRole}...`;
            fetch('/role/set', {
                method: 'POST',
                headers: { 'Content-Type': 'text/plain' },
                body: newRole
            })
            .then(res => res.text())
            .then(text => {
                statusSpan.textContent = text;
                if (text.startsWith('OK')) {
                    mainTitle.textContent = `GompeiVision Pipeline: ${newRole}`;
                    roleInput.value = ''; // Clear input on success
                }
            })
            .catch(error => { statusSpan.textContent = 'Error: ' + error; });
            return false;
        }

        function handleStartCalibration(event) {
            event.preventDefault();
            const data = {
                squares_x: parseInt(document.getElementById('squaresX').value),
                squares_y: parseInt(document.getElementById('squaresY').value),
                square_length_m: parseFloat(document.getElementById('squareLength').value),
                marker_length_m: parseFloat(document.getElementById('markerLength').value)
            };
            statusSpan.textContent = 'Starting new session...';
            fetch('/calibration/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(data)
            }).catch(error => { statusSpan.textContent = 'Error: ' + error; });
            return false;
        }

        function handlePost(url) {
            statusSpan.textContent = 'Command sent...';
            fetch(url, { method: 'POST' })
                .catch(error => { statusSpan.textContent = 'Error: ' + error; });
            return false;
        }

        function updateStatus() {
            fetch('/calibration/status')
                .then(response => response.json())
                .then(data => {
                    frameCountSpan.textContent = data.frame_count;
                    statusSpan.textContent = data.status_message;
                    finishBtn.disabled = data.is_calibrating;
                })
                .catch(error => {
                    console.error('Error fetching status:', error);
                    statusSpan.textContent = 'Error fetching status.';
                });
        }
    </script>
</body>
</html>
)raw";

  // Serve the main HTML UI
  m_server.Get(
      "/", [html_content](const httplib::Request& req, httplib::Response& res) {
        res.set_content(html_content, "text/html");
      });

  // Endpoint to set the role
  m_server.Post(
      "/role/set", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string new_role = req.body;
        if (new_role.empty()) {
          res.status = 400;
          res.set_content("ERROR: Role name cannot be empty.", "text/plain");
          return;
        }
        {
          std::lock_guard<std::mutex> lock(m_role_mutex);
          m_role = new_role;
        }
        std::cout << "[INFO] Role changed to: " << new_role << std::endl;
        res.set_content("OK: Role updated to " + new_role, "text/plain");
      });

  // Calibration control endpoints
  m_server.Post("/calibration/start", [this](const httplib::Request& req,
                                             httplib::Response& res) {
    try {
      auto json_body = nlohmann::json::parse(req.body);
      const int squares_x = json_body.at("squares_x");
      const int squares_y = json_body.at("squares_y");
      const float square_length_m = json_body.at("square_length_m");
      const float marker_length_m = json_body.at("marker_length_m");

      PipelineHelper::start_calibration_session(
          m_role, m_is_calibrating, m_calibration_mutex, m_calibration_session,
          m_capture_next_frame_for_calib, m_calibration_status_mutex,
          m_calibration_status_message, squares_x, squares_y, square_length_m,
          marker_length_m);
      res.set_content("OK: Calibration session started with new parameters.",
                      "text/plain");
    } catch (const nlohmann::json::exception& e) {
      res.status = 400;  // Bad Request
      res.set_content("ERROR: Invalid JSON format or missing parameters. " +
                          std::string(e.what()),
                      "text/plain");
    }
  });

  m_server.Post("/calibration/capture", [this](const httplib::Request& req,
                                               httplib::Response& res) {
    PipelineHelper::add_calibration_frame(m_capture_next_frame_for_calib);
    res.set_content("OK: Capture command received.", "text/plain");
  });

  m_server.Get("/calibration/status", [this](const httplib::Request& req,
                                             httplib::Response& res) {
    nlohmann::json status_json;
    {
      std::lock_guard<std::mutex> lock(m_calibration_mutex);
      status_json["frame_count"] =
          m_calibration_session ? m_calibration_session->get_frame_count() : 0;
    }
    {
      std::lock_guard<std::mutex> lock(m_calibration_status_mutex);
      status_json["status_message"] = m_calibration_status_message;
    }
    status_json["is_calibrating"] = m_is_calibrating.load();

    res.set_content(status_json.dump(), "application/json");
  });

  m_server.Post("/calibration/finish", [this](const httplib::Request& req,
                                              httplib::Response& res) {
    if (m_is_calibrating) {
      res.status = 409;  // Conflict
      res.set_content("ERROR: Calibration is already in progress.",
                      "text/plain");
      return;
    }

    // Join the previous worker thread if it's still joinable
    if (m_calibration_worker_thread.joinable()) {
      m_calibration_worker_thread.join();
    }

    m_is_calibrating = true;
    {
      std::lock_guard<std::mutex> lock(m_calibration_status_mutex);
      m_calibration_status_message = "Processing... This may take a moment.";
    }

    std::string filename;
    {
      std::lock_guard<std::mutex> lock(m_role_mutex);
      filename = "~/GompeiVision/" + m_role + "_calibration.yml";
    }

    // Launch the heavy computation in a new background thread
    m_calibration_worker_thread = std::thread(
        &PipelineHelper::async_finish_calibration, std::ref(m_is_calibrating),
        std::ref(m_calibration_mutex), std::ref(m_calibration_session),
        std::ref(m_calibration_status_mutex),
        std::ref(m_calibration_status_message), filename);

    res.set_content("OK: Final calibration process started in the background.",
                    "text/plain");
  });
  // Start listening
  m_server.listen("0.0.0.0", m_control_port);
  std::cout << "[" << m_role << "] Control panel server stopped." << std::endl;
}