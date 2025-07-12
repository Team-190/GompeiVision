#include <iostream>
#include <string>

#include "openpnp-capture.h"
#include "pipeline/Pipeline.h"

int main() {
  std::cout << "--- Goat Class Web Server Test ---" << std::endl;

  // --- Minimal Camera Setup ---
  // We need to find at least one camera to get a valid hardware ID
  // to pass to the Goat constructor.
  const CapContext ctx = Cap_createContext();
  if (!ctx || Cap_getDeviceCount(ctx) == 0) {
    std::cerr << "ERROR: No camera found. Cannot run test." << std::endl;
    return -1;
  }
  const char* id_ptr = Cap_getDeviceUniqueID(ctx, 0);
  const std::string camera_id(id_ptr ? id_ptr : "");
  Cap_releaseContext(ctx);  // We don't need the context anymore for this test

  if (camera_id.empty()) {
    std::cerr << "ERROR: Could not get a valid hardware ID for the camera."
              << std::endl;
    return -1;
  }

  // --- Test Parameters ---
  const std::string TEST_ROLE = "front_cam";
  constexpr int TEST_PORT = 1181;

  std::cout << "\nAttempting to start Goat worker for camera: " << camera_id
            << std::endl;
  std::cout << "Role: " << TEST_ROLE << std::endl;
  std::cout << "Web Server Port: " << TEST_PORT << std::endl;

  // 1. Create an instance of your Goat class.
  // The constructor will initialize the camera and the web server components.
  Pipeline goat_worker(camera_id, TEST_ROLE, TEST_PORT, TEST_PORT + 1);

  // 2. Start the internal threads (capture loop and server loop).
  goat_worker.start();

  // 3. Keep the main thread alive so the server can run.
  std::cout << "\n--- Server is running ---" << std::endl;
  std::cout << "Open a web browser and navigate to http://localhost:"
            << TEST_PORT + 1 << std::endl;
  std::cout << "You should also see the MJPEG stream at http://localhost:"
            << TEST_PORT << "/stream.mjpg" << std::endl;
  std::cout << "Press [Enter] in this terminal to stop the server and exit."
            << std::endl;

  // Wait for user input
  std::cin.get();

  // 4. Cleanly shut down the worker process.
  std::cout << "Shutdown signal received. Stopping worker..." << std::endl;
  goat_worker.stop();

  std::cout << "--- Test Program Finished ---" << std::endl;
  return 0;
}