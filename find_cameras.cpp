#include <iostream>
#include <filesystem>
#include <vector>
#include <opencv2/opencv.hpp>

#include "find_cameras.h"

namespace fs = std::filesystem;

int GetIdFromName(std::string name) {
  
}

std::vector<USBCamera> GetConnectedCameras() {
  std::vector<USBCamera> out;
  fs::path usb_directory("/dev/v4l/by-id");
  // fs::path usb_directory("/sys/bus/usb/devices");
  for (auto& p : fs::directory_iterator(usb_directory)) {
    fs::path current_camera = p;
    cv::VideoCapture cap(current_camera, cv::CAP_V4L);
    if (cap.isOpened()) {
      out.push_back(USBCamera(GetIdFromName(current_camera), current_camera));
    }
  }
}