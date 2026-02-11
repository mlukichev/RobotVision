#include "cameras.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>
#include <iostream>
#include <fstream>
#include <absl/log/check.h>

namespace robot_vision {


//922.9237643349662,0.0,647.4788707248217,0.0,919.8858077883355,399.87229770900007,0.0,0.0,1.0]},"distCoeffs":{"rows":1,"cols":8,"type":6,"data":[0.05877060800641068,-0.10744454024922885,3.2631843588809995E-4,0.0014886030530316236,0.04142037702520378,6.850399028674193E-4,0.0014547294832255185,-0.0037595003107658287]}

bool Cameras::CameraExists(CamId camera) const {
  return cameras_.find(camera) != cameras_.end();
}

Camera Cameras::GetCameraByID(CamId camera) const {
  auto it = cameras_.find(camera);
  return it->second;
}

std::vector<int> Cameras::GetKeys() const {
  std::vector<int> out;
  for (auto [e, _] : cameras_) {
    out.push_back(e);
  }
  return out;
}

void Cameras::emplace(int id, Camera camera) {
  cameras_.emplace(id, camera);
}

Cameras ReadCameraCoefficients(const std::string& filename) {

  Cameras cameras;

  std::ifstream file(filename);
  CHECK(file) << "Could not open file " << filename;

  int cam_num;
  CHECK(file >> cam_num) << "Error reading from " << filename;
  for (int i=0; i<cam_num; ++i) {
    int cam;
    CHECK(file >> cam) << "Error reading from " << filename;
    
    cv::Mat cam_mat = cv::Mat::zeros(3, 3, CV_64F);

    for (int j=0; j<3; ++j) {
      for (int k=0; k<3; ++k) {
        double cam_val;
        CHECK(file >> cam_val) << "Error reading from " << filename;
        if (cam_val != 1.0) {
          cam_val /= 2;
        }
        cam_mat.at<double>(j, k) = cam_val;
      }
    }

    int dist_val;
    CHECK(file >> dist_val) << "Error reading from " << filename;
    cv::Mat dist_mat = cv::Mat::zeros(1, dist_val, CV_64F);
    for (int i=0; i<dist_val; ++i) {
      double dist_val;
      CHECK(file >> dist_val)  << "Error reading from " << filename;
      dist_mat.at<double>(0, i) = dist_val;
    }

    cameras.emplace(cam, Camera(cam, cam_mat, dist_mat));
  }

  return cameras;
}

}