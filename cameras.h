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
  bool CameraExists(CamId id) const;
  Camera GetCameraByID(CamId id) const;
  std::vector<int> GetKeys() const;
  void emplace(int id, Camera camera);

 private:
  std::unordered_map<int, Camera> cameras_;
};

Cameras ReadCameraCoefficients(const std::string& filename);

}

#endif // CAMERAS_H