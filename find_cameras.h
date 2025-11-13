#ifndef FIND_CAMERAS_H
#define FIND_CAMERAS_H

#include <iostream>
#include <vector>

class OpenedCamera {
 public:
  
 private:
  int id;
  std::unique_ptr<cv::VideoCapture> cap;
}

#endif // FIND_CAMERAS_H