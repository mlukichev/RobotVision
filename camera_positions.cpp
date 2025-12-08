#include "camera_positions.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>

namespace robot_vision {

CameraPositions::CameraPositions() {
  // camera_positions_[0] = Transformation((cv::Mat_<double>(4, 4) << 
  //   0, 0, -1, 0,
  //   1, 0, 0, 0,
  //   0, -1, 0, 0,
  //   0, 0, 0, 1
  // ));
  // camera_positions_[1] = Transformation((cv::Mat_<double>(4, 4) << 
  //   1, 0, 0, 248.92,
  //   0, 0, 1, 1022.35,
  //   0, -1, 0, 0,
  //   0, 0, 0, 1
  // ));
}

bool CameraPositions::CameraExists(CameraId cam) const {
  return camera_positions_.find(cam) != camera_positions_.end();
}

Transformation CameraPositions::GetCameraPositionById(CameraId cam) const {
  auto it = camera_positions_.find(cam);
  // CHECK(it != tags_.end()) << "Tag " << tag << " doesn't exist";
  return it->second;
}

}