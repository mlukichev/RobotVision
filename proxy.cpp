#include <crow.h>
#include <grpcpp/grpcpp.h>
#include "data_handling.grpc.pb.h"
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/log/log.h>

ABSL_FLAG(std::string, server_address, "localhost:50051", "Address of the gRPC server");
ABSL_FLAG(int32_t, http_port, 8080, "Port for the HTTP server");

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using robot_vision::VisionSystem;
using robot_vision::GetRobotPositionRequest;
using robot_vision::GetRobotPositionResponse;

class VisionSystemClient {
 public:
  VisionSystemClient(std::shared_ptr<Channel> channel)
      : stub_(VisionSystem::NewStub(channel)) {}

  std::vector<double> GetRobotPosition() {
    GetRobotPositionRequest request;
    GetRobotPositionResponse response;
    ClientContext context;

    Status status = stub_->GetRobotPosition(&context, request, &response);

    if (status.ok()) {
      std::vector<double> matrix;
      for (double val : response.mat()) {
        matrix.push_back(val);
      }
      return matrix;
    } else {
      LOG(ERROR) << status.error_code() << ": " << status.error_message();
      return {};
    }
  }

 private:
  std::unique_ptr<VisionSystem::Stub> stub_;
};

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  std::string server_address = absl::GetFlag(FLAGS_server_address);
  VisionSystemClient client(grpc::CreateChannel(
      server_address, grpc::InsecureChannelCredentials()));

  crow::SimpleApp app;

  CROW_ROUTE(app, "/robot_position")
  ([&client]() {
    std::vector<double> matrix = client.GetRobotPosition();
    if (matrix.empty()) {
       return crow::response(500, "Failed to get robot position");
    }
    
    crow::json::wvalue x;
    for(size_t i = 0; i < matrix.size(); ++i) {
        x["matrix"][i] = matrix[i];
    }
    return crow::response(x);
  });

  app.port(absl::GetFlag(FLAGS_http_port)).multithreaded().run();

  return 0;
}
