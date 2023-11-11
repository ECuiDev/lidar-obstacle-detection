# CMakeList.txt : CMake project for lidar-obstacle-detection, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.26)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
  CACHE STRING "Vcpkg toolchain file")

project ("lidar-obstacle-detection")

set(PCL_DIR "C:/Users/Edward/source/repos/lidar-obstacle-detection/vcpkg/installed/x64-windows/share/pcl")

find_package(PCL REQUIRED)

# Add source to this project's executable.
add_executable (lidar-obstacle-detection "lidar-obstacle-detection.cpp" "lidar-obstacle-detection.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET lidar-obstacle-detection PROPERTY CXX_STANDARD 20)
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(lidar-obstacle-detection ${PCL_LIBRARIES})

# TODO: Add tests and install targets if needed.
