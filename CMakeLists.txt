cmake_minimum_required(VERSION 3.16)
project(robot_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(protobuf_BUILD_TESTS OFF CACHE BOOL "Disable building protobuf tests" FORCE)
set(BUILD_PYTHON_WRAPPER OFF)
set(protobuf_ABSL_PROVIDER "package" CACHE STRING "" FORCE)
set(protobuf_INSTALL OFF CACHE BOOL "" FORCE)
set(utf8_range_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)

# Add third-party library
add_subdirectory(third_party/apriltag)
add_subdirectory(third_party/abseil-cpp)
add_subdirectory(third_party/protobuf)

set(_OpenCV_REQUIRED_COMPONENTS core imgproc videoio highgui calib3d)
find_package(OpenCV COMPONENTS ${_OpenCV_REQUIRED_COMPONENTS} QUIET CONFIG)

find_package(Protobuf REQUIRED)

add_library(data_handling)

protobuf_generate(
  TARGET data_handling
  LANGUAGE cpp
  PROTOS data_handling.proto
  OUT_VAR data_handling_protos)

add_library(robot_vision   
  camera_handling.cpp
  camera.cpp
  transformations.cpp
  tags.cpp
  apriltag_detector.cpp
)

add_executable(camera_calib  camera_calib.cpp)
add_executable(test_position_detection test_position_detection.cpp)
add_executable(generate_transform_matrix generate_transform_matrix.cpp)

target_link_libraries(data_handling PRIVATE
  protobuf::libprotobuf
)

# # Link with apriltag library (assuming target is named 'apriltag')
target_link_libraries(robot_vision PRIVATE
  apriltag
  ${OpenCV_LIBRARIES}
  absl::flags
  absl::log
)  

# target_link_libraries(camera_handling PRIVATE 
#   apriltag 
#   ${OpenCV_LIBRARIES}
#   absl::flags
#   # grpc

#   camera
#   transformations
# )

target_link_libraries(camera_calib PRIVATE 
  apriltag
  ${OpenCV_LIBRARIES}
  absl::flags
  absl::flags_parse
  absl::log

  robot_vision
)

target_link_libraries(test_position_detection PRIVATE
  apriltag
  ${OpenCV_LIBRARIES}
  # absl::flags
  # absl::flags_parse
  absl::log

  robot_vision
)

target_link_libraries(generate_transform_matrix PRIVATE
  ${OpenCV_LIBRARIES}
  absl::log

  robot_vision
)


# target_link_libraries(camera_handling PRIVATE
#   ${OpenCV_LIBRARIES}
#   grpc
  
# )