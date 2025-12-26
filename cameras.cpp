#include "cameras.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>

namespace robot_vision {


//922.9237643349662,0.0,647.4788707248217,0.0,919.8858077883355,399.87229770900007,0.0,0.0,1.0]},"distCoeffs":{"rows":1,"cols":8,"type":6,"data":[0.05877060800641068,-0.10744454024922885,3.2631843588809995E-4,0.0014886030530316236,0.04142037702520378,6.850399028674193E-4,0.0014547294832255185,-0.0037595003107658287]}

Cameras::Cameras() {
  cameras_.emplace(2, Camera(2, 
    (cv::Mat_<double>(3, 3) << 
      922.9237643349662, 0, 647.4788707248217,
      0, 919.8858077883355, 399.87229770900007,
      0, 0, 1
    ), 
    (cv::Mat_<double>(1, 8) << 
      0.05877060800641068,-0.10744454024922885,3.2631843588809995E-4,0.0014886030530316236,0.04142037702520378,6.850399028674193E-4,0.0014547294832255185,-0.0037595003107658287 
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