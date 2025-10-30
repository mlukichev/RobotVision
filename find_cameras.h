#ifndef FIND_CAMERAS_H
#define FIND_CAMERAS_H

#include <iostream>
#include <vector>

class USBCamera {
 public:
  USBCamera(int id, std::string port) : id_{id}, port_{port} {}

 private:
  int id_;
  std::string port_;
}

std::vector<USBCamera> GetConnectedCameras();

#endif // FIND_CAMERAS_H