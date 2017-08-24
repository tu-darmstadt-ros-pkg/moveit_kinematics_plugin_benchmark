#include <moveit_kinematics_plugin_benchmark/inverse_kinematics_benchmark.h>

namespace moveit_kinematics_plugin_benchmark {

double fRand(double min, double max) {
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

InverseKinematicsBenchmark::InverseKinematicsBenchmark()
  : nh_("")
{
  ros::NodeHandle pnh("~");
  pnh.param("num_samples", num_samples_, 1000);

  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader());
  robot_model_ = robot_model_loader_->getModel();
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
}

bool InverseKinematicsBenchmark::generateRandomStates(std::string group_name) {
  // load joint model group
  const robot_model::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_name);
  if (joint_model_group == NULL) {
      ROS_ERROR_STREAM("Joint model group '" << group_name << "' doesn't exist.");
      return false;
  }
  robot_model::JointBoundsVector bounds = joint_model_group->getActiveJointModelsBounds();
  seed_state_.resize(joint_model_group->getActiveJointModels().size());
  for (unsigned int i = 0; i < seed_state_.size(); i++) {
    seed_state_[i] = ((*bounds[i])[0].min_position_ + (*bounds[i])[0].max_position_) / 2;
  }

  random_states_.resize(num_samples_);
  for (unsigned int i = 0; i < random_states_.size(); i++) {
    random_states_[i].resize(joint_model_group->getActiveJointModels().size());
    for (unsigned int j = 0; j < random_states_[i].size(); j++) {
      random_states_[i][j] = fRand((*bounds[j])[0].min_position_, (*bounds[j])[0].max_position_);
    }
  }
  return true;
}

bool InverseKinematicsBenchmark::runBenchmark(std::string group_name, BenchmarkResult &result) {
  if (seed_state_.empty() || random_states_.empty()) {
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
  for (unsigned int i = 0; i < random_states_.size(); i++) {
    std::vector<geometry_msgs::Pose> target_poses;
    solver->getPositionFK(solver->getTipFrames(), random_states_[i], target_poses);
    start_time = boost::posix_time::microsec_clock::local_time();
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;
    bool result = solver->getPositionIK(target_poses[0], seed_state_, solution, error_code);
    time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
    total_time += time_diff.total_nanoseconds() / 1e9;

    if (result) {
      found_solutions++;
    }
  }

  result.group_name = group_name;
  result.num_samples = num_samples_;
  result.num_solutions = found_solutions;
  result.total_time = total_time;
  return true;
}

std::vector<double> generateRandomJointState(const robot_model::JointModelGroup* joint_model_group) {

}


}
