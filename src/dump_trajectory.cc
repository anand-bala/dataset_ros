#include "ros/ros.h"

#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "google/protobuf/text_format.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(pbfile, "",
    "Path to the protobuf stream file outputted by cartographer");


namespace dataset_ros {
  void RunPbDump(std::string pbfile) {
    cartographer::io::ProtoStreamReader reader(pbfile);
    cartographer::mapping::proto::PoseGraph posegraph;
    CHECK(reader.ReadProto(&posegraph)) << "unable to read pbfile";
    std::string proto_str;
    google::protobuf::TextFormat::PrintToString(posegraph, &proto_str);
    LOG(INFO) << proto_str;
  }
} // namespace dataset_ros


int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbfile.empty()) << "-pbfile missing.";
  dataset_ros::RunPbDump(FLAGS_pbfile);
}

