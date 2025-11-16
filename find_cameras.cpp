#include "find_cameras.h"

#include <iostream>
#include <filesystem>
#include <vector>

#include "opencv2/opencv.hpp"

namespace fs = std::filesystem;

int GetIdFromName(std::string name) {
  int loc = name.find("camera-");
  if (loc == -1) {
    return -1;
  }
  return (int)(name[loc+7]-'0');
}

/*
std::unique_ptr OpenConnectedCameras() {
  // std::vector<USBCamera> out;
  fs::path usb_directory("/dev/v4l/by-id");
  for (auto& p : fs::directory_iterator(usb_directory)) {
    fs::path current_camera = fs::read_symlink(p);
    std::string video_num = current_camera;
    if (video_num.find("video") != 6) {
      continue;
    }
    int id = (int)(video_num[11]-'0');
    cv::VideoCapture cap(id);
    if (cap.isOpened()) {
      // std::cout << " id: " << id << " GOOD" << std::endl;
    }
  }
}
*/

int main() {
  std::cout << GetIdFromName("usb-Arducam_Technology_Co.__Ltd._camera-2_UC762-video-index0") << std::endl;
  // GetConnectedCameras();
}