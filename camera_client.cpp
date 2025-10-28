#include <iostream>
#include <optional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "data_handling.pb.h"
#include "data_handling.grpc.pb.h"
#include "grpcpp/grpcpp.h"

#include "transformations.h"
#include "camera_handling.h"
#include "apriltag_detector.h"

namespace robot_vision {

class CameraSet {
 public:
  CameraSet(std::unordered_map<int, Camera> cameras): cameras_{cameras} {}
  
  absl::StatusOr<Camera> GetCamera(int port) {
    auto it = cameras_.find(port);
    if (it == cameras_.end()) {
      LOG(ERROR) << "No camera on port " << port;
      return absl::InternalError("Invalid port");
    }
    return it->second;
  }

  void ForEachCamera(std::function<void(int port, const Camera&)> visitor) {
    for (const auto& [port, camera] : cameras_) {
      visitor(port, camera);
    }
  }

  // std::unordered_map<int, T> Apply(std::function<T(int port, const Camera&)> visitor) {

  //   for (auto [port, camera] : cameras_) {

  //   }
  // }
 private:
  std::unordered_map<int, Camera> cameras_;
};

namespace {

void Run(CameraSet& camera_set) {
  std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel("localhost:50051", grpc::InsecureChannelCredentials());
  std::unique_ptr<VisionSystem::Stub> vision_system = VisionSystem::NewStub(channel);

  grpc::ClientContext context;

  std::unique_ptr<grpc::ClientReaderWriter<ClientRequest, ServerRequest>> stream  = vision_system->OpenControlStream(&context);

  cv::VideoCapture cap(0);
  ApriltagDetector detector;
  cv::Mat frame, gray;
  std::vector<Transformation> sols; 

  while (true) {
    // // Get camera coords...
    // cap.read(frame);
    // cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // std::vector<TagPoints> out = detector.Detect(gray);
    // std::vector<Transformation> sols;
    // for (int i=0; i<out.size(); ++i) {
    //   std::optional<std::pair<Transformation, Transformation>> pos1 = TransformTagToCam(cam, tags, out[i].id, out[i].points, 64.29);
    //   sols.push_back(pos1->first);
    //   sols.push_back(pos1->second);
    // }
    // ClientRequest req;
    // // Set request fields...
    // ReportCameraPositions* camera_positions = req.mutable_report_camera_positions();
    // camera_set.ForEachCamera(
    //   [&camera_positions](int port, const Camera& cam) {      
    //     CameraPosition* camera_position = camera_positions->add_camera_position();
    //     camera_position->set_camera_id(cam.id);
    //   });
    
    // if (!stream->Write(req)) {
    //   LOG(ERROR) << "The stream has been closed.";
    //   return;
    // }


    for (int cam=0; cam<camera_set.size(); ++i) {
      cap.read(frame);
      cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      std::vector<TagPoints> out = detector.Detect(gray);
      ClientRequest req;
      ReportCameraPositions* camera_positions = req.mutable_report_camera_positions();
      std::vector<Transformation> sols;
      for (int i=0; i<out.size(); ++i) {
        std::optional<std::pair<Transformation, Transformation>> pos1 = TransformTagToCam(camera_set.GetCamera(cam), tags, out[i].id, out[i].points, 64.29);
        CameraPosition* camera_position = camera_positions->add_camera_position();
        camera_position->set_camera_id(cam.id);
        sols.push_back(pos1->first);
        sols.push_back(pos1->second);
      }
    }
  }
}

}  // namespace
}  // namespace robot_vision

int main(int argc, char* argv[]) {
  robot_vision::CameraSet camera_set;
  while (true) {
    Run();
  }
}