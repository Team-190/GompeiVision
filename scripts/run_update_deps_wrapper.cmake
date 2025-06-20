# File: gompei_vision/scripts/run_update_deps_wrapper.cmake
#
# This wrapper script is executed by CMake's -P mode to run the Python script.
# It uses a timestamp-based check to decide if the Python script *might* need to run.

# --- Define Paths ---
set(VERSIONS_JSON_PATH "${CMAKE_CURRENT_LIST_DIR}/../versions.json")
set(UPDATE_DEPS_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/update_deps.py")
set(GENERATED_VERSIONS_CMAKE_PATH "${CMAKE_CURRENT_LIST_DIR}/../third_party/versions.cmake")

# --- Decide if the Python script needs to run (Timestamp Check) ---
set(SHOULD_RUN_PYTHON_SCRIPT TRUE) # Assume we need to run it by default

# 1. If generated versions.cmake does not exist, we must run.
if (NOT EXISTS ${GENERATED_VERSIONS_CMAKE_PATH})
    set(SHOULD_RUN_PYTHON_SCRIPT TRUE)
    message(STATUS "  versions.cmake does not exist. Python script will run to generate it.")
endif ()

# --- Only run the heavy lifting (Python script) if necessary ---
# Get the path to the Python interpreter (Python 3 is required)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
if (NOT Python3_FOUND)
    message(FATAL_ERROR "Python3 interpreter not found for update_deps.py script. Please install Python 3.")
endif ()

# Set the working directory for the Python script to the project root
set(PROJECT_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/..")

# Execute the Python script
execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${UPDATE_DEPS_SCRIPT}"
        WORKING_DIRECTORY "${PROJECT_ROOT_DIR}" # Run the script from the project root
        RESULT_VARIABLE SCRIPT_EXIT_CODE
        OUTPUT_VARIABLE SCRIPT_OUTPUT
        ERROR_VARIABLE SCRIPT_ERROR
)

# Check the script's exit code for success/failure
if (NOT SCRIPT_EXIT_CODE EQUAL 0)
    message(FATAL_ERROR "Failed to run update_deps.py. Exit Code: ${SCRIPT_EXIT_CODE}\n"
            "Output:\n${SCRIPT_OUTPUT}\n"
            "Error:\n${SCRIPT_ERROR}")
else ()
    message(STATUS "Successfully ran update_deps.py:\n${SCRIPT_OUTPUT}")
endif ()
return() # Exit this wrapper script early
