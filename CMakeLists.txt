# CMakeList.txt : CMake project for TouchstoneInterface, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("TouchstoneInterface")

# Add source to this project's executable.
# Add source to this project's executable.
add_executable (TouchstoneInterface 
    "TouchstoneInterface.cpp" 
    "TouchstoneInterface.h"
    "MagEncoder.cpp"
    "MagEncoder.h"
    "SerialInterface.cpp"
    "SerialInterface.h"
    "DRIFTMotor.cpp"
    "DRIFTMotor.h"
    "DRIFTPlex.cpp"
    "DRIFTPlex.h"
    "include/serialib/serialib.cpp"
)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET TouchstoneInterface PROPERTY CXX_STANDARD 20)
endif()

# Find Eigen3 package
find_package(Eigen3 3.4 REQUIRED)

# Include Eigen3 headers
if (Eigen3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  message(FATAL_ERROR "Eigen3 not found. Please set the Eigen3_DIR environment variable to the directory containing Eigen3Config.cmake.")
endif()

# Uses Eigen as alias for Eigen3
target_link_libraries(${CMAKE_PROJECT_NAME}  Eigen3::Eigen)

add_library(serialib INTERFACE)

target_include_directories(serialib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/serialib>
  $<INSTALL_INTERFACE:include/serialib>
)

target_link_libraries(${CMAKE_PROJECT_NAME}  serialib)

# TODO: Add tests and install targets if needed.
