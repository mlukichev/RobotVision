#ifndef FIND_CAMERAS_H
#define FIND_CAMERAS_H

#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

class OpenedCamera {
 public:
  OpenedCamera(int id /* , 
    Cannot pass unique_ptr into a constructor due to ownership constraints
    std::unique_ptr<cv::VideoCapture> cap
    */): id_{id} /*, cap_{cap} */ {}
 private:
  int id_;
  std::unique_ptr<cv::VideoCapture> cap_;
};

#endif // FIND_CAMERAS_H