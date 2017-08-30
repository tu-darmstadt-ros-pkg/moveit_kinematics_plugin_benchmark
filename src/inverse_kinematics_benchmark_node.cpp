#include <moveit_kinematics_plugin_benchmark/inverse_kinematics_benchmark.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "inverse_kinematics_benchmark_node");

  ros::NodeHandle pnh("~");
  std::vector<std::string> groups;
  if (!pnh.getParam("groups", groups)) {
    ROS_ERROR_STREAM("No groups specified in namespace '" << pnh.getNamespace() << "/groups'. Exiting.");
    return 0;
  }
  if (groups.empty()) {
    ROS_ERROR_STREAM("Group list is empty. Exiting.");
    return 0;
  }
  moveit_kinematics_plugin_benchmark::InverseKinematicsBenchmark bm;

  bm.generateCenterSeedState(groups[0]);

  if (pnh.param("random_samples", true)) {
    int num_random_samples;
    pnh.param("num_random_samples", num_random_samples, 1000);
    ROS_INFO_STREAM("Generating " << num_random_samples << " random samples..");
    bm.generateRandomStates(groups[0], num_random_samples);
  } else {
    XmlRpc::XmlRpcValue sample_states;
    if (pnh.getParam("sample_states", sample_states)) {
      ROS_ASSERT(sample_states.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (unsigned int i = 0; i < sample_states.size(); i++) {
        ROS_ASSERT(sample_states[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        std::vector<double> sample_state;
        for (unsigned int j = 0; j < sample_states[i].size(); j++) {
          double pos;
          if (sample_states[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            pos = static_cast<double>(sample_states[i][j]);
          } else if (sample_states[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            int pos_i = static_cast<int>(sample_states[i][j]);
            pos = double(pos_i);
          } else {
            ROS_ERROR_STREAM("Sample position is invalid.");
            return 0;
          }
          sample_state.push_back(pos);

        }
        bm.addSampleState(sample_state);
      }
    } else {
      ROS_ERROR_STREAM("Could not find parameter 'sample_states' in ns '" << pnh.getNamespace() << "'.");
      return 0;
    }
  }


  ROS_INFO_STREAM("Running benchmarks..");
  std::vector<moveit_kinematics_plugin_benchmark::BenchmarkResult> results(groups.size(), moveit_kinematics_plugin_benchmark::BenchmarkResult());
  for (unsigned int g = 0; g < groups.size(); g++) {
    bm.runBenchmark(groups[g], results[g]);
  }
  for (unsigned int i = 0; i < results.size(); i++) {
    ROS_INFO_STREAM("Solver for group " << results[i].group_name << " found " << results[i].num_solutions << " solutions (" << 100.0*results[i].successPercentage()
                    << "\%) with an average of " << results[i].timePerSample() * 1e3 <<" ms per sample");
  }
  ROS_INFO_STREAM("Finished all benchmarks..");

  return 0;
}
