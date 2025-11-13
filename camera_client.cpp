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

class OpenedCamera {
 public:
  OpenedCamera(const Camera& cam, std::unique_ptr<cv::VideoCapture> cap) cam_{cam}, cap_{cap} {}
 private:
  Camera cam_;
  std::unique_ptr<cv::VideoCapture> cap_;
}
class CameraSet {
 public:
  CameraSet(const std::unordered_map<int, OpenedCamera>& cameras): cameras_{cameras} {}

  
  absl::StatusOr<Camera> GetCamera(int port) {
    auto it = cameras_.find(port);
    if (it == cameras_.end()) {
      LOG(ERROR) << "No camera on port " << port;
      return absl::InternalError("Invalid port");
    }
    return it->second;
  }

  // std::unordered_map<int, T> Apply(std::function<T(int port, const Camera&)> visitor) {

  //   for (auto [port, camera] : cameras_) {

  //   }
  // }
 private:
  std::unordered_map<int, Camera> cameras_;
};

namespace {
class VisionSystemClient {
 public:
  VisionSystemImpl() {}

  absl::Status Run();
  absl::Status ReportCameraPositions(const CameraSet& camera_set);
 private:
  absl::Status SendMessage(const ClientRequest& msg);

  absl::Mutex mu_;
  std::optional<ClientReaderWriter<ServerRequest, ClientRequest>*> server_;
};

CameraSet BuildCameraSet() {
  CameraSet cam_set;
  fs::path usb_directory("/dev/v4l/by-id");
  for (auto& p : fs::directory_iterator(usb_directory)) {
    fs::path current_camera = fs::read_symlink(p);
    std::string video_num = current_camera;
    if (video_num.find("video") != 6) {
      continue;
    }
    int id = (int)(video_num[11]-'0');
    std::unique_ptr<cv::VideoCapture> cap(id);
    if (cap.isOpened() && cam_set.cameras_.find(id) != cam_set.cameras_.end()) {
      cam_set.cameras_[id] = OpenedCamera(cap, );
    }
  }
}

absl::Status VisionSystemClient::Run() {
  std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());
  std::unique_ptr<VisionSystem::Stub> vision_system = VisionSystem::NewStub(channel);

  grpc::ClientContext context;

  std::unique_ptr<grpc::ClientReaderWriter<ClientRequest, ServerRequest>> stream  = vision_system->OpenControlStream(&context);

  {
    absl::MutexLock lock{&mu_};
    if (!server_.has_value()) {
      LOG(ERROR) << "Client is already connected.";
      return absl::InvalidArgumentError("Client already connected.");
    }
    server_ = stream.get();
  }

  absl::Cleanup cleanup_client = [&]() { 
    absl::MutexLock lock{&mu_};
    server_.reset(); 
  }; 

  ServerRequest req;
  while (stream->Read(&req)) {
    switch (req.msg_case()) {
    case ServerRequest::kSetCameraCoefficients:
      SetCameraCoefficients(req.set_camera_coefficients());
      break;
    default:
      LOG(WARNING) << "Dropping unrecognized message: " << req.DebugString();
      break;
    }
  }

  return absl::OkStatus();

}

absl::Status VisionSystemImpl::SendMessage(const ClientRequest& msg) {
  absl::MutexLock lock{&mu_};
  if (!server_.has_value()) {
    LOG(ERROR) << "SendMessage: Not connected to server";
    return absl::InvalidArgumentError("Not connected");
  }

  if (!(*server_)->Write(msg, WriteOptions())) {
    LOG(ERROR) << "SendMessage: Write failed to server";
    return absl::InternalError("Write failed");
  }
}

absl::Status ReportCameraPositions(const CameraSet& camera_set) {
  ClientRequest req;
  // TODO fill out ClientRequest
  
 return SendMessage(req);
}

void ComputePosition() {

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