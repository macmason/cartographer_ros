#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <google/protobuf/io/coded_stream.h>

#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);

  for (const auto& filename : args) {
    cartographer::io::ProtoStreamReader reader(filename);
    cartographer::mapping::proto::SparsePoseGraph pose_graph;

    std::string decompressed_data;
    CHECK(reader.Read(&decompressed_data));
    google::protobuf::io::CodedInputStream stream(reinterpret_cast<const uint8*>(decompressed_data.data()), decompressed_data.size());
    stream.SetTotalBytesLimit(std::numeric_limits<int>::max(), -1);
    pose_graph.ParseFromCodedStream(&stream);

    std::cout << filename << std::endl
              << "\tTrajectories: " << pose_graph.trajectory_size() << std::endl
              << "\tConstraints: " << pose_graph.constraint_size() << std::endl;

    int inter = 0;
    int intra = 0;

    for (const auto& constraint : pose_graph.constraint()) {
      if (constraint.tag() == cartographer::mapping::proto::SparsePoseGraph::Constraint::INTRA_SUBMAP) {
        intra++;
      } else {
        inter++;
      }
    }

    std::cout << "\tInter: " << inter << std::endl
              << "\tIntra: " << intra << std::endl;
  }
}
