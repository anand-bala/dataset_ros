#include "ros/ros.h"

#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "google/protobuf/text_format.h"
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include <iostream>
#include <fstream>

DEFINE_string(pbfile, "",
    "Path to the protobuf stream file outputted by cartographer");

DEFINE_string(outfile, "trajectory.txt",
    "Path to the file where you want to output the trajectory."
    "The directory where the file is present must exist");

namespace dataset_ros {
  void RunPbDump(std::string pbfile, std::string outfile) {
    cartographer::io::ProtoStreamReader reader(pbfile);
    cartographer::mapping::proto::PoseGraph posegraph;
    CHECK(reader.ReadProto(&posegraph)) << "unable to read pbfile";

    std::ofstream output_file;
    output_file.open(outfile);
    google::protobuf::TextFormat::Print(posegraph,
        new google::protobuf::io::OstreamOutputStream(&output_file));
  }
} // namespace dataset_ros


int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbfile.empty()) << "-pbfile missing.";
  CHECK(!FLAGS_outfile.empty()) << "-outfile missing." 
    << "Outputting to trajectory.txt";
  dataset_ros::RunPbDump(FLAGS_pbfile, FLAGS_outfile);
}

