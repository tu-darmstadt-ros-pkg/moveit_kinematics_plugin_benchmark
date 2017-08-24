#ifndef INVERSE_KINEMATICS_BENCHMARK_H
#define INVERSE_KINEMATICS_BENCHMARK_H

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <boost/date_time.hpp>

namespace moveit_kinematics_plugin_benchmark {
  struct BenchmarkResult {
    std::string group_name;
    double total_time; // seconds
    int num_solutions;
    int num_samples;

    double timePerSample() {
      return total_time / (double) num_samples;
    }

    double successPercentage() {
      return num_solutions / (double) num_samples;
    }
  };

  class InverseKinematicsBenchmark {
  public:
    InverseKinematicsBenchmark();
    bool generateRandomStates(std::string group_name);
    bool runBenchmark(std::string group_name, BenchmarkResult& result);
  private:
    ros::NodeHandle nh_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;

    std::vector<double> seed_state_;
    std::vector<std::vector<double>> random_states_;

    // configuration
    int num_samples_;
  };
}

#endif
