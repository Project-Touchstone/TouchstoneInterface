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
    "src/comms/MinBiTSerialNode.cpp"
    "src/comms/MinBiTSerialNode.h"
    "src/actuators/DRIFTMotor.cpp"
    "src/actuators/DRIFTMotor.h"
    "src/actuators/DRIFTPlex.cpp"
    "src/actuators/DRIFTPlex.h"
    "src/utils/Utils.h"
    "src/utils/Utils.cpp"
    "src/utils/Timer.h"
    "src/utils/Timer.cpp"
    "src/sensors/IMU.cpp"
    "src/sensors/IMU.h"
    "src/sensors/MagTracker.h"
    "src/sensors/MagTracker.cpp"
    "src/sensors/Thimble.h"
    "src/sensors/Thimble.cpp"
    "src/comms/MinBiTTcpServer.h"
    "src/comms/MinBiTTcpServer.cpp"
    "src/comms/MinBiTCore.cpp"
    "src/comms/MinBiTCore.h"
    "src/comms/IStream.h"
    "src/comms/SerialStream.h"
    "src/comms/SerialStream.cpp"
    "src/comms/TcpStream.h"
    "src/comms/TcpStream.cpp"
 "src/utils/DynamicConfig.h" "src/utils/DynamicConfig.cpp")

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
target_link_libraries(${CMAKE_PROJECT_NAME} ${Boost_LIBRARIES})

# Header-only libraries
include_directories(${CMAKE_SOURCE_DIR}/include)

# Google Test for unit testing
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  TouchstoneUnitTest
    "src/test/DRIFTUnitTest.cpp"
    "src/test/DataProtocolUnitTest.cpp"
    "src/sensors/MagEncoder.cpp"
    "src/sensors/MagEncoder.h"
    "src/comms/MinBiTSerialNode.cpp"
    "src/comms/MinBiTSerialNode.h"
    "src/actuators/DRIFTMotor.cpp"
    "src/actuators/DRIFTMotor.h"
    "src/actuators/DRIFTPlex.cpp"
    "src/actuators/DRIFTPlex.h"
    "src/utils/Utils.h"
    "src/utils/Utils.cpp"
    "src/utils/Timer.h"
    "src/utils/Timer.cpp"
    "src/sensors/IMU.cpp"
    "src/sensors/IMU.h"
    "src/sensors/MagTracker.h"
    "src/sensors/MagTracker.cpp"
    "src/sensors/Thimble.h"
    "src/sensors/Thimble.cpp"
    "src/comms/MinBiTTcpServer.h"
    "src/comms/MinBiTTcpServer.cpp"
    "src/comms/MinBiTCore.cpp"
    "src/comms/MinBiTCore.h"
    "src/comms/IStream.h"
    "src/comms/SerialStream.h"
    "src/comms/SerialStream.cpp"
    "src/comms/TcpStream.h"
    "src/comms/TcpStream.cpp"
 "src/utils/DynamicConfig.h" "src/utils/DynamicConfig.cpp")
target_link_libraries(
  TouchstoneUnitTest
  GTest::gtest_main
  Eigen3::Eigen
)

include(GoogleTest)
gtest_discover_tests(TouchstoneUnitTest)
