#include "cameras.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>

namespace robot_vision {

Cameras::Cameras() {
  cameras_.emplace(2, Camera(2, 
    (cv::Mat_<double>(3, 3) << 
      1168.36, 0, 456.197,
      0, 1154.91, 555.766,
      0, 0, 1
    ), 
    (cv::Mat_<double>(1, 5) << 
      -0.481087, 0.603502, -0.0180071, 0.0253094, -0.419686 
  )));
} 

bool Cameras::CameraExists(CamId camera) const {
  return cameras_.find(camera) != cameras_.end();
}

Camera Cameras::GetCameraByID(CamId camera) const {
  auto it = cameras_.find(camera);
  // CHECK(it != cameras_.end()) << "Camera " << camera << " doesn't exist";
  return it->second;
}

std::vector<int> Cameras::GetKeys() const {
  std::vector<int> out;
  for (auto [e, _] : cameras_) {
    out.push_back(e);
  }
  return out;
}

}