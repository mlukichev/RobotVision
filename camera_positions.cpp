#include "camera_positions.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>

#include "absl/log/check.h"

namespace robot_vision {

CameraPositions::CameraPositions() {
  robot_to_camera_[2] = Transformation((cv::Mat_<double>(4, 4) << 
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  ));
}

bool CameraPositions::CameraExists(CameraId cam) const {
  return robot_to_camera_.find(cam) != robot_to_camera_.end();
}

const Transformation& CameraPositions::GetRobotToCamera(CameraId cam) const {
  auto it = robot_to_camera_.find(cam);
  CHECK(it != robot_to_camera_.end()) << "Camera " << cam << " doesn't exist";
  return it->second;
}

}