cmake_minimum_required(VERSION 3.16)
project(robot_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Turn off shared libs for apriltag
set(BUILD_SHARED_LIBS OFF)
set(BUILD_EXAMPLES OFF)
set(OpenCV_STATIC ON)

# Loading 3rd party packages
find_package(absl CONFIG REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

# Add third-party library
add_subdirectory(third_party/apriltag)

add_library(
  vision_proto 
  STATIC
    data_handling.proto)
target_include_directories(
  vision_proto 
  INTERFACE ${CMAKE_BINARY_DIR})
target_link_libraries(
  vision_proto 
  PUBLIC gRPC::grpc++)
protobuf_generate(
  TARGET vision_proto
  LANGUAGE cpp)
protobuf_generate(
  TARGET vision_proto
  LANGUAGE grpc
  PLUGIN protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
  GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc)

add_library(
  robot_vision 
  STATIC   
    camera_handling.cpp
    camera.cpp
    transformations.cpp
    tags.cpp
    apriltag_detector.cpp
    camera_positions.cpp
    cameras.cpp)
target_link_libraries(
  robot_vision 
  PUBLIC
    apriltag
    opencv_core
    opencv_videoio
    opencv_calib3d
    absl::flags
    absl::log)

add_library(
  vision_system 
  STATIC 
    vision_system.cpp)
target_link_libraries(
  vision_system 
  PUBLIC
    opencv_core
    absl::flat_hash_map
    absl::log
    absl::status
    absl::statusor
    absl::synchronization
    vision_proto
    robot_vision)

# Camera calibration program
add_executable(
  camera_calib  
  camera_calib.cpp)
target_link_libraries(
  camera_calib 
  PRIVATE 
    apriltag
    ${OpenCV_LIBRARIES}
    absl::flags
    absl::flags_parse
    absl::log
    robot_vision)

# Local testing of algorithms
add_executable(
  test_position_detection 
  test_position_detection.cpp)
target_link_libraries(
  test_position_detection 
  PRIVATE
    apriltag
    ${OpenCV_LIBRARIES}
    absl::log
    robot_vision)

add_executable(
  generate_transform_matrix 
  generate_transform_matrix.cpp)
target_link_libraries(
  generate_transform_matrix 
  PRIVATE
    ${OpenCV_LIBRARIES}
    absl::log
    robot_vision)

# Vision System server binary
add_executable(
  robot_vision_server 
  server.cpp)
target_link_libraries(
  robot_vision_server 
  PRIVATE
    # API definition
    vision_proto
    # General vision utils
    robot_vision
    # Vision system controller
    vision_system
    # External libraries
    absl::cleanup
    absl::flags
    absl::flags_parse)

# Vision system client binary
add_executable(
  camera_client 
  camera_client.cpp)
target_link_libraries(
  camera_client
  PRIVATE
    # API definition
    vision_proto
    # General vision utils
    robot_vision  
    # External libraries
    absl::flags
    absl::flags_parse
    absl::log
    absl::status
    apriltag
    grpc
    opencv_core)

# Testing camera lookup code
add_executable(
  find_cameras 
  find_cameras.cpp)
target_link_libraries(
  find_cameras
  ${OpenCV_LIBRARIES})