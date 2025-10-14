cmake_minimum_required(VERSION 3.16)
project(robot_vision)

# find_package(Protobuf REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add third-party library
add_subdirectory(third_party/apriltag)
add_subdirectory(third_party/abseil-cpp)
# add_subdirectory(third_party/grpc)

set(_OpenCV_REQUIRED_COMPONENTS core imgproc videoio highgui calib3d)
find_package(OpenCV COMPONENTS ${_OpenCV_REQUIRED_COMPONENTS} QUIET CONFIG)

add_library(robot_vision   
  camera_handling.cpp
  camera.cpp
  transformations.cpp
  tags.cpp
  april_tag_detector.cpp
)

add_executable(camera_calib  camera_calib.cpp)
add_executable(test_position_detection test_position_detection.cpp)
add_executable(generate_transform_matrix generate_transform_matrix.cpp)

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