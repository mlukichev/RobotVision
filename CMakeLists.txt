cmake_minimum_required(VERSION 3.10)
project(robot_vision)

# Add third-party library
add_subdirectory(third_party/apriltag)

set(_OpenCV_REQUIRED_COMPONENTS core imgproc videoio highgui calib3d)
find_package(OpenCV COMPONENTS ${_OpenCV_REQUIRED_COMPONENTS} QUIET CONFIG)

# Add executable
add_executable(robot_vision robot_vision.cpp)

# Link with apriltag library (assuming target is named 'apriltag')
target_link_libraries(robot_vision PRIVATE apriltag ${OpenCV_LIBRARIES})