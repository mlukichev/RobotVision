cmake_minimum_required(VERSION 3.16)
project(robot_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Loading 3rd party packages
find_package(absl CONFIG REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

# Add third-party library
add_subdirectory(third_party/apriltag)

add_library(vision_proto data_handling.proto)
target_include_directories(vision_proto INTERFACE ${CMAKE_BINARY_DIR})
target_link_libraries(vision_proto PUBLIC gRPC::grpc++)
protobuf_generate(
  TARGET vision_proto
  LANGUAGE cpp)
protobuf_generate(
  TARGET vision_proto
  LANGUAGE grpc
  PLUGIN protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
  GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc)

add_library(robot_vision   
  camera_handling.cpp
  camera.cpp
  transformations.cpp
  tags.cpp
  apriltag_detector.cpp)
target_link_libraries(robot_vision PRIVATE
  apriltag
  ${OpenCV_LIBRARIES}
  absl::flags
  absl::log)

add_library(vision_system
  vision_system.cpp)
target_link_libraries(vision_system PUBLIC
  ${OpenCV_LIBRARIES}
  absl::flat_hash_map
  absl::log
  absl::status
  absl::statusor
  absl::synchronization
  vision_proto)

add_executable(camera_calib  camera_calib.cpp)
target_link_libraries(camera_calib PRIVATE 
  apriltag
  ${OpenCV_LIBRARIES}
  absl::flags
  absl::flags_parse
  absl::log
  robot_vision)

add_executable(test_position_detection test_position_detection.cpp)
target_link_libraries(test_position_detection PRIVATE
  apriltag
  ${OpenCV_LIBRARIES}
  # absl::flags
  # absl::flags_parse
  absl::log
  robot_vision)

add_executable(generate_transform_matrix generate_transform_matrix.cpp)
target_link_libraries(generate_transform_matrix PRIVATE
  ${OpenCV_LIBRARIES}
  absl::log
  robot_vision)

add_executable(robot_vision_server server.cpp)
target_link_libraries(robot_vision_server PRIVATE
  vision_proto
  vision_system
  absl::flags
  absl::cleanup)

# add_executable(camera_client camera_client.cpp)
# target_link_libraries(camera_client
#   absl::flags
#   absl::log
#   absl::status
#   grpc
#   apriltag
#   ${OpenCV_LIBRARIES}
  
#   vision_proto
#   robot_vision
# )

add_executable(find_cameras find_cameras.cpp)
target_link_libraries(find_cameras
  ${OpenCV_LIBRARIES}
)

# target_link_libraries(camera_handling PRIVATE 
#   apriltag 
#   ${OpenCV_LIBRARIES}
#   absl::flags
#   # grpc
#   camera
#   transformations
# )