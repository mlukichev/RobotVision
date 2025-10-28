#include <iostream>
#include <filesystem>
#include <vector>

// #include "find_cameras.h"

namespace fs = std::filesystem;

void /*std::vector<USBCamera>*/ GetCameraNames() {
  fs::path usb_directory("/sys/bus/usb/devices/");
  for (auto& p : fs::directory_iterator(usb_directory)) {
    std::cout << p << std::endl;
  }
}

int main() {
  GetCameraNames();
}