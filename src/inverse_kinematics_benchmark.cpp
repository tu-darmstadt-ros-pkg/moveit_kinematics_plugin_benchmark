#include <moveit_kinematics_plugin_benchmark/inverse_kinematics_benchmark.h>

namespace moveit_kinematics_plugin_benchmark {

double fRand(double min, double max) {
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

InverseKinematicsBenchmark::InverseKinematicsBenchmark()
  : nh_("")
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader());
  robot_model_ = robot_model_loader_->getModel();
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
}

bool InverseKinematicsBenchmark::generateCenterSeedState(std::string group_name) {
  // load joint model group
  const robot_model::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_name);
  if (joint_model_group == NULL) {
      ROS_ERROR_STREAM("Joint model group '" << group_name << "' doesn't exist.");
      return false;
  }
  // Create seed state
  robot_model::JointBoundsVector bounds = joint_model_group->getActiveJointModelsBounds();
  seed_state_.resize(joint_model_group->getActiveJointModels().size());
  for (unsigned int i = 0; i < seed_state_.size(); i++) {
    seed_state_[i] = ((*bounds[i])[0].min_position_ + (*bounds[i])[0].max_position_) / 2;
  }
  ROS_INFO_STREAM("Seed state: " << vecToString(seed_state_));
}

bool InverseKinematicsBenchmark::generateRandomStates(std::string group_name, int num_samples) {
  // load joint model group
  const robot_model::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_name);
  if (joint_model_group == NULL) {
      ROS_ERROR_STREAM("Joint model group '" << group_name << "' doesn't exist.");
      return false;
  }

  // Create random sample states
  robot_model::JointBoundsVector bounds = joint_model_group->getActiveJointModelsBounds();
  sample_states_.resize(num_samples);
  for (unsigned int i = 0; i < sample_states_.size(); i++) {
    sample_states_[i] = getRandomState(bounds);
  }
  return true;
}

std::vector<double> InverseKinematicsBenchmark::getRandomState(const robot_model::JointBoundsVector& bounds) {
  std::vector<double> random_state(bounds.size());
  for (unsigned int j = 0; j < random_state.size(); j++) {
    random_state[j] = fRand((*bounds[j])[0].min_position_, (*bounds[j])[0].max_position_);
  }
  return random_state;
}

void InverseKinematicsBenchmark::addSampleState(const std::vector<double> joint_state) {
  sample_states_.push_back(joint_state);
}

bool InverseKinematicsBenchmark::runBenchmark(std::string group_name, BenchmarkResult &result) {
  if (seed_state_.empty() || sample_states_.empty()) {
    ROS_ERROR_STREAM("Please run generateRandomStates() before starting benchmarks. Exiting.");
    return false;
  }
  // load joint model group
  const robot_model::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_name);
  if (joint_model_group == NULL) {
      ROS_ERROR_STREAM("Joint model group '" << group_name << "' doesn't exist.");
      return false;
  }
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  // Retrieve solver
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();
  if (!solver){
    ROS_ERROR("No IK solver loaded for group %s, cannot set group configuration via IK.", joint_model_group->getName().c_str());
    return false;
  }
  std::string tip_frame = solver->getTipFrame();
  robot_model::JointBoundsVector bounds = joint_model_group->getActiveJointModelsBounds();

  // Debug output
  std::stringstream debug;
  debug << "Starting benchmark with joint group '" << group_name << "'. " << std::endl;
  debug << "Base frame: " << solver->getBaseFrame() << std::endl;
  debug << "Tip frame: " << tip_frame << std::endl;
  debug << "Joints:" << std::endl;
  for (unsigned int i = 0; i < joint_names.size(); i++) {
      debug << i << ": " << joint_names[i] << "[" << (*bounds[i])[0].min_position_ << ", " << (*bounds[i])[0].max_position_ << "]" << std::endl;
  }
  ROS_INFO_STREAM(debug.str());

  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration time_diff;

  double total_time = 0;
  int found_solutions = 0;
  for (unsigned int i = 0; i < sample_states_.size(); i++) {
    ROS_INFO_STREAM("Sample: " << i);
    ROS_INFO_STREAM("Sample state: " << vecToString(sample_states_[i]));
    std::vector<geometry_msgs::Pose> target_poses;
    solver->getPositionFK(solver->getTipFrames(), sample_states_[i], target_poses);
//    ROS_INFO_STREAM("Target pose: " << target_poses[0]);
    start_time = boost::posix_time::microsec_clock::local_time();
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;
    bool result = solver->getPositionIK(target_poses[0], seed_state_, solution, error_code);


    ROS_INFO_STREAM("Time: " << time_diff.total_nanoseconds() / 1e6 << " ms");

    int max_tries = 3;
    int current_try = 0;
    while (!result && current_try < max_tries) {
      std::vector<double> random_seed = getRandomState(bounds);
      ROS_INFO_STREAM("Retrying with random seed: " << vecToString(random_seed));
      result = solver->getPositionIK(target_poses[0], random_seed, solution, error_code);
      current_try++;
    }
//    ROS_INFO_STREAM("Solution: " << vecToString(solution));
    time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
    total_time += time_diff.total_nanoseconds() / 1e9;

    if (result) {
      found_solutions++;
      ROS_INFO_STREAM("Success on try " << current_try);
    } else {
      ROS_INFO_STREAM("Failure");
    }
    ROS_INFO_STREAM("-------------------------------------");
  }

  result.group_name = group_name;
  result.num_samples = sample_states_.size();
  result.num_solutions = found_solutions;
  result.total_time = total_time;
  return true;
}

}
