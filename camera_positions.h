#ifndef CAMERA_POSITIONS_H
#define CAMERA_POSITIONS_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <unordered_map>

#include "transformations.h"

namespace robot_vision {

using CameraId = int;

class CameraPositions {
 public:
  CameraPositions();
  bool CameraExists(CameraId tag) const;
  const Transformation& GetRobotToCamera(CameraId tag) const;
   
 private:
  // camera-id -> (coord transformation: Robot -> Camera)
  std::unordered_map<int, Transformation> robot_to_camera_;
};

}

#endif // CAMERA_POSITIONS_H