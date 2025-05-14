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
add_executable (TouchstoneInterface 
    "src/TouchstoneInterface.cpp" 
    "src/TouchstoneInterface.h"
    "src/sensors/MagEncoder.cpp"
    "src/sensors/MagEncoder.h"
    "src/comms/SerialInterface.cpp"
    "src/comms/SerialInterface.h"
    "src/actuators/DRIFTMotor.cpp"
    "src/actuators/DRIFTMotor.h"
    "src/actuators/DRIFTPlex.cpp"
    "src/actuators/DRIFTPlex.h"
 "src/utils/Utils.h" "src/utils/Utils.cpp" "src/utils/Timer.h" "src/utils/Timer.cpp" "src/sensors/IMU.cpp" "src/sensors/IMU.h" "src/sensors/MagTracker.h" "src/sensors/MagTracker.cpp"   "src/sensors/Thimble.h" "src/sensors/Thimble.cpp" "src/comms/HapticRenderServer.h" "src/comms/HapticRenderServer.cpp" "src/comms/DataProtocol.cpp" "src/comms/DataProtocol.h")

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

# Find Boost package
find_package(Boost 1.82.0 REQUIRED)

# Include Boost headers
if (Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Boost not found. Please install Boost.")
endif()

# Link Boost to the executable
target_link_libraries(TouchstoneInterface ${Boost_LIBRARIES})

# TODO: Add tests and install targets if needed.
