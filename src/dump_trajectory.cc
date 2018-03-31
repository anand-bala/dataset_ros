#include "ros/ros.h"

#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

#include "google/protobuf/text_format.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"

#include "cartographer/common/time.h"
#include "cartographer_ros/time_conversion.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include <iostream>
#include <fstream>

DEFINE_string(pbfile, "",
    "Path to the protobuf stream file outputted by cartographer");

DEFINE_string(outfile, "trajectory.csv",
    "Path to the file where you want to output the trajectory."
    "The directory where the file is present must exist");

namespace dataset_ros {

  void RunPbDump(std::string pbfile, std::string outfile) {
    cartographer::io::ProtoStreamReader reader(pbfile);
    cartographer::mapping::proto::PoseGraph posegraph;
    CHECK(reader.ReadProto(&posegraph)) << "unable to read pbfile";
    LOG(INFO) << "Number of Trajectories: " << posegraph.trajectory_size();

    auto tstamp = posegraph.trajectory(0).node(0).timestamp();
    auto protoTime = cartographer::common::FromUniversal(tstamp);
    LOG(INFO) << "First Timestamp is " << tstamp;
    LOG(INFO) << " ---> " << cartographer_ros::ToRos(protoTime);
    std::ofstream ofile;
    ofile.open(outfile, std::ios::out | std::ios::trunc );
    ofile << "TrajectoryID,NodeID,Timestamp,PositionX,PositionY,QuaternionZ,QuaternionW," << std::endl;

    for (int i = 0; i < posegraph.trajectory_size(); i++) {
      const cartographer::mapping::proto::Trajectory &t = posegraph.trajectory(i);
      for(int j = 0; j < t.node_size(); j++) {
        const cartographer::mapping::proto::Trajectory::Node &n = t.node(j);
        auto ts = cartographer::common::FromUniversal(n.timestamp());
        ofile << (i+1) << ',' << j+1 << ','
          << cartographer_ros::ToRos(ts) << ','
          << n.pose().translation().x() << ','
          << n.pose().translation().y() << ','
          << n.pose().rotation().z() << ','
          << n.pose().rotation().w() << ','
          << std::endl;
      }
      LOG(INFO) << "Nodes Parsed: " << t.node_size();
    }

    ofile.close();
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

