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
  apriltag_detector.cpp
  camera_positions.cpp
  cameras.cpp)
target_link_libraries(robot_vision PRIVATE
  apriltag
  ${OpenCV_LIBRARIES}
  absl::flags
  absl::log)

add_library(vision_system vision_system.cpp)
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
  robot_vision
  vision_proto
  vision_system
  absl::flags
  absl::flags_parse
  absl::cleanup)

add_executable(camera_client camera_client.cpp)
target_link_libraries(camera_client PRIVATE
  absl::flags
  absl::flags_parse
  absl::log
  absl::status
  grpc
  apriltag
  ${OpenCV_LIBRARIES}
  
  vision_proto
  robot_vision)

add_executable(find_cameras find_cameras.cpp)
target_link_libraries(find_cameras
  ${OpenCV_LIBRARIES})

find_package(Crow REQUIRED)

add_executable(proxy proxy.cpp)
target_link_libraries(proxy PRIVATE
  Crow::Crow
  gRPC::grpc++
  vision_proto
  absl::flags
  absl::flags_parse
  absl::log
)

set(JAVA_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_generated)
file(MAKE_DIRECTORY ${JAVA_OUT_DIR})
add_custom_command(
    OUTPUT ${JAVA_OUT_DIR}/frc/robot/protos
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --java_out=${JAVA_OUT_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/data_handling.proto
    DEPENDS data_handling.proto
    COMMENT "Generating Java bindings from ${CMAKE_CURRENT_SOURCE_DIR}/data_handling.proto"
)
add_custom_command(
    OUTPUT ${JAVA_OUT_DIR}/frc/robot/protos/service
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS    --plugin=protoc-gen-grpc-java=${GRPC_JAVA_PLUGIN}
            --java_out=${JAVA_OUT_DIR}
            --grpc-java_out=$<TARGET_FILE:gRPC::grpc_java_plugin>
            -I ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/data_handling.proto
    DEPENDS data_handling.proto
    COMMENT "Generating gRPC Java bindings from ${CMAKE_CURRENT_SOURCE_DIR}/data_handling.proto"
)
add_custom_target(
  generate_proto_java
  DEPENDS ${JAVA_OUT_DIR}/frc/robot/protos ${JAVA_OUT_DIR}/frc/robot/protos/service)