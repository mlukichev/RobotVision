#include <iostream>
#include <optional>
#include <memory>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "data_handling.pb.h"
#include "data_handling.grpc.pb.h"
#include "grpcpp/server_builder.h"
#include "vision_system.h"

ABSL_FLAG(std::string, server_address, "0.0.0.0:50001", "Vision system server address");

namespace robot_vision {
namespace {

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReaderWriter;
using grpc::Status;

class VisionSystemImpl: public VisionSystem::Service {
 public:
  VisionSystemImpl(VisionSystemCore* vision_system_core): 
    vision_system_core_{vision_system_core} {}

  virtual Status OpenControlStream(ServerContext* context,
                                   ServerReaderWriter<ServerRequest, ClientRequest>* stream); 
 private:
  VisionSystemCore* vision_system_core_;
};

Status VisionSystemImpl::OpenControlStream(
  ServerContext* context,
  ServerReaderWriter<ServerRequest, ClientRequest>* stream) {
  ClientRequest req;
  while (stream->Read(&req)) {
    switch (req.msg_case()) {
    case ClientRequest::kReportCameraPositions:
      for (const CameraPosition& pos : req.report_camera_positions().camera_position()) {
        absl::Status status = vision_system_core_->ReportCameraPosition(pos);
        if (!status.ok()) {
          LOG(WARNING) << "Error processing ReportCameraPosition request: " << status;
        }
      }
      break;
    default:
      LOG(WARNING) << "Dropping unrecognized message: " << req.DebugString();
      break;
    }
  }
  return Status::OK;
}

void RunServer(VisionSystemCore* vision_system_core, const std::string& server_address) {
  VisionSystemImpl service(vision_system_core);

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());

  LOG(INFO) << "Server listening on " << server_address;
  server->Wait();
}

}  // namespace
}  // namespace robot_vision

int main() {
  robot_vision::VisionSystemCore vision_system_core;

  robot_vision::RunServer(&vision_system_core, absl::GetFlag(FLAGS_server_address));
}