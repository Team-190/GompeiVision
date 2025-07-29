#include "pipeline/PipelineManager.h"

#include <chrono>
#include <iostream>
#include <ranges>

#include "openpnp-capture.h"

// --- RAII Wrapper for CapContext ---
// Ensures that the openpnp-capture context is automatically released.
struct CapContextDeleter {
  void operator()(const CapContext ctx) const {
    if (ctx) {
      Cap_releaseContext(ctx);
      std::cout << "[Manager] OpenPnP Capture context released." << std::endl;
    }
  }
};
using CapContextPtr = std::unique_ptr<void, CapContextDeleter>;
// ---------------------------------

PipelineManager& PipelineManager::getInstance() {
  static PipelineManager instance;
  return instance;
}

PipelineManager::PipelineManager() {
  std::cout << "[Manager] Initializing Pipeline Manager..." << std::endl;
}

PipelineManager::~PipelineManager() {
  std::cout << "[Manager] Shutting down Pipeline Manager..." << std::endl;
  stopAll();
}

void PipelineManager::startAll() {
  if (m_is_running) {
    std::cout << "[Manager] Pipelines are already running." << std::endl;
    return;
  }

  std::cout << "[Manager] Detecting cameras and starting pipelines..."
            << std::endl;

  // --- Camera Enumeration using openpnp-capture C API ---
  const CapContextPtr ctx(Cap_createContext());
  if (!ctx) {
    std::cerr << "[Manager] ERROR: Failed to create openpnp-capture context."
              << std::endl;
    return;
  }

  const uint32_t device_count = Cap_getDeviceCount(ctx.get());
  if (device_count == 0) {
    std::cerr
        << "[Manager] ERROR: No cameras found. GompeiVision will not start."
        << std::endl;
    return;
  }

  std::cout << "[Manager] Found " << device_count << " camera(s)." << std::endl;

  std::lock_guard<std::mutex> lock(m_pipelines_mutex);
  int default_width = 1280;
  int default_height = 720;

  for (uint32_t i = 0; i < device_count; ++i) {
    constexpr int control_port_base = 1182;
    constexpr int stream_port_base = 5800;
    const char* name_cstr = Cap_getDeviceName(ctx.get(), i);
    const char* id_cstr = Cap_getDeviceUniqueID(ctx.get(), i);

    if (!id_cstr || std::string(id_cstr).empty()) {
      std::cerr << "[Manager] WARNING: Skipping camera "
                << (name_cstr ? name_cstr : "at index " + std::to_string(i))
                << " due to missing unique/hardware ID." << std::endl;
      continue;
    }

    const std::string hardware_id = id_cstr;
    const std::string name = name_cstr ? name_cstr : "Unknown Camera";
    const int device_index = static_cast<int>(i);

    std::cout << "[Manager] Found camera: " << name << " (ID: " << hardware_id
              << ", Index: " << device_index << ")" << std::endl;

    // Assign a unique role based on hardware ID.
    std::string role = "cam_" + hardware_id;

    // Assign unique network ports for each pipeline to avoid conflicts.
    int stream_port = stream_port_base + (device_index * 2);
    int control_port = control_port_base + (device_index * 2);

    auto pipeline = std::make_unique<Pipeline>(device_index, hardware_id,
                                               default_width, default_height,
                                               role, stream_port, control_port);

    pipeline->start();
    m_pipelines[hardware_id] = std::move(pipeline);
  }

  if (!m_pipelines.empty()) {
    m_is_running = true;
    m_heartbeat_thread = std::thread(&PipelineManager::heartbeat_loop, this);
    std::cout << "[Manager] Started " << m_pipelines.size() << " pipelines."
              << std::endl;
  } else {
    std::cerr << "[Manager] ERROR: Failed to start any pipelines." << std::endl;
  }
}

void PipelineManager::stopAll() {
  if (!m_is_running.exchange(false)) {
    return;  // Already stopped or stopping
  }

  std::cout << "[Manager] Stopping all pipelines..." << std::endl;

  if (m_heartbeat_thread.joinable()) {
    m_heartbeat_thread.join();
  }

  std::lock_guard<std::mutex> lock(m_pipelines_mutex);
  for (auto& val : m_pipelines | std::views::values) {
    if (val) {
      val->stop();
    }
  }
  m_pipelines.clear();
  std::cout << "[Manager] All pipelines stopped and cleared." << std::endl;
}

void PipelineManager::heartbeat_loop() {
  while (m_is_running) {
    {
      std::lock_guard<std::mutex> lock(m_pipelines_mutex);
      std::cout << "[Manager] Heartbeat: " << m_pipelines.size()
                << " pipelines are active." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
  std::cout << "[Manager] Heartbeat loop stopped." << std::endl;
}