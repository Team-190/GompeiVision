#pragma once

#include <limits.h>  // For PATH_MAX
#include <unistd.h>  // For readlink

#include <string>

namespace platform {

/**
 * @brief Gets the full, absolute path to the currently running executable.
 *
 * This implementation is for Linux and works by reading the /proc/self/exe
 * symbolic link.
 *
 * @return The absolute path to the executable, or an empty string on failure.
 */
inline std::string get_self_executable_path() {
  char result[PATH_MAX];
  if (const ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
      count != -1) {
    // The result from readlink is not null-terminated.
    return std::string(result, count);
  }
  // Return an empty string to indicate an error.
  return "";
}

}  // namespace platform