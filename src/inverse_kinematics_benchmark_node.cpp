#include <moveit_kinematics_plugin_benchmark/inverse_kinematics_benchmark.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "inverse_kinematics_benchmark_node");

  ros::NodeHandle pnh("~");
  std::vector<std::string> groups;
  if (!pnh.getParam("groups", groups)) {
    ROS_ERROR_STREAM("No groups specified in namespace '" << pnh.getNamespace() << "/groups'. Exiting.");
    return 0;
  }
  moveit_kinematics_plugin_benchmark::InverseKinematicsBenchmark bm;
  ROS_INFO_STREAM("Generating random states..");
  bm.generateRandomStates(groups[0]);

  ROS_INFO_STREAM("Running benchmarks..");
  std::vector<moveit_kinematics_plugin_benchmark::BenchmarkResult> results(groups.size(), moveit_kinematics_plugin_benchmark::BenchmarkResult());
  for (unsigned int g = 0; g < groups.size(); g++) {
    bm.runBenchmark(groups[g], results[g]);
  }
  for (unsigned int i = 0; i < results.size(); i++) {
    ROS_INFO_STREAM("Solver for group " << results[i].group_name << " found " << results[i].num_solutions << " solutions ( " << 100.0*results[i].successPercentage()
                    << "\%) with an average of " << results[i].timePerSample() <<" secs per sample");
  }
  ROS_INFO_STREAM("Finished all benchmarks..");

  return 0;
}
