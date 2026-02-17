#include "camera_positions.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>
#include <iostream>
#include <fstream>

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
  // CHECK(it != robot_to_camera_.end()) << "Camera " << cam << " doesn't exist";
  return it->second;
}

void CameraPositions::emplace(int cam, Transformation pos) {
  robot_to_camera_.emplace(cam, pos);
}

CameraPositions ReadCameraPositions(const std::string& filename) {

  std::ifstream file(filename);
  CHECK(file) << "Could not open file " << filename;

  CameraPositions cams;

  int cam_num;
  CHECK(file >> cam_num) << "Error reading from " << filename;
  for (int i=0; i<cam_num; ++i) {
    int cam;
    CHECK(file >> cam) << "Error reading from " << filename;
    cv::Mat pos(4, 4, CV_64F);
    for (int j=0; j<4; ++j) {
      for (int k=0; k<4; ++k) {
        double val;
        CHECK(file >> val) << "Error reading from " << filename;
        pos.at<double>(j, k) = val;
      }
    }

    cams.emplace(cam, Transformation(pos));
  }

  return cams;
}

}