#ifndef CAMERAS_H
#define CAMERAS_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <unordered_map>

#include "camera.h"

namespace robot_vision {

using CamId = int;

class Cameras {
 public:
  Cameras();
  bool CameraExists(CamId tag) const;
  Camera GetCameraByID(CamId tag) const;
  std::vector<int> GetKeys() const;
   
 private:
  std::unordered_map<int, Camera> cameras_;
};

}

#endif // CAMERAS_H