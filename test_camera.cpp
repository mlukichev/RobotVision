#include <opencv2/opencv.hpp>
#include <filesystem>
#include <optional>

#include "absl/log/log.h"

namespace fs = std::filesystem;

std::optional<int> GetIdFromName(const std::string& name) {
  int loc = name.find("camera-");
  if (loc == -1) {
    return std::nullopt;
  }
  return (int)(name[loc+7]-'0');
}

int main() {
  fs::path usb_directory("/dev/v4l/by-id");
  for (auto& p : fs::directory_iterator(usb_directory)) {
    fs::path current_camera = fs::read_symlink(p);
    std::string video_num = current_camera;
    if (video_num.find("video") != 6) {
      continue;
    }
    int port = (int)(video_num[11]-'0');
    std::unique_ptr<cv::VideoCapture> cap(new cv::VideoCapture(port));
    
    LOG(INFO) << "Port: " << port << " | Camera Id: " << *camera_id << " | "<< static_cast<fs::path>(p).filename();

    cv::Mat frame;
    if (!cap->read(frame)) {
      LOG(INFO) << "Write failed at camera: " + ;
    }
  }
}