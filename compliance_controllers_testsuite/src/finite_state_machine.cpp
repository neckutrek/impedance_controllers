#include <finite_state_machine.h>

#include <kdl/jntarray.hpp>

#include <cstdlib>
#include <fstream>



#define MARKER_LIFETIME 10

FiniteStateMachine::FiniteStateMachine(ros::NodeHandle& nh, std::string controller_name) 
: start_pose_counter_(0), state_(STARTUP_FSM), controller_name_(controller_name) {
  spg_ = std::make_shared<StartPoseGenerator>(nh);
  tpp_ = std::make_shared<TrajectoryPosePublisher>(nh, controller_name_);
  switch_controller_client_ = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
  load_controller_client_ = nh.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");

  F_e_ = Eigen::VectorXd::Zero(6);
  F_e_max_ = Eigen::VectorXd::Zero(6);

  marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      controller_name_ + "/visualization_marker", 1);

  force_sensor_calibration_pub_ = nh.advertise<std_msgs::Bool>("/ethdaq_zero", 1);

  //contact_sensor_subscriber_ = nh.subscribe(
  //    controller_name_ + "/contact_forces", 1000, &FiniteStateMachine::contactSensorSubscriberCallback, this);
}

void FiniteStateMachine::setJointNames(std::vector<std::string> joint_names) {
  joint_names_ = joint_names;
}

void FiniteStateMachine::setStartConfigurations(std::string urdf_file, std::string root_link, std::string tip_link) {
  root_link_ = root_link;
	spg_->init(urdf_file, root_link, tip_link);
  //spg_->setNominalPosition(0.7, 0.3, 0.45); // simulation
  spg_->setNominalPosition(0.6, 0.1, 0.3); // in-real-life
  spg_->setPositionMargins(0.1, 0.1, 0.1);
  spg_->setPositionDensity(1, 1, 1);
  spg_->setOrientationMargins(10, 10, 1, 1);
  spg_->computeAllJntConfigurations();
}

void FiniteStateMachine::setTrajectoryParams(double T, double min, double max) {
  tpp_->init(T, min, max);
}

int FiniteStateMachine::update() {
  // 1. goto start position
  // 2. check state error to know when reached
  // 3. switch controller to impedance controller
  // 4. start publishing trajectory
  // 5. when published end pose of trajectory, start waiting for steady state of manipulator
  // 6. measure time to go on trajectory, measure jnt config and ee error
  // 7. write to csv-file
  // 8. get next start position
  // 9. repeat from 1.

  switch (state_) {
  case STARTUP_FSM:
    startupFsm();
    break;
  case INITIALIZE_ROBOT:
    initializeRobot();
    break;
  case GOTO_START_POSE:
    gotoStartPose();
    break;
  case FOLLOW_TRACK_FRW:
    followTrackForward();
    break;
  case CONVERGE_AT_END_POSE:
    convergeAtEndPose();
    break;
  case FOLLOW_TRACK_BCK:
    followTrackBackwards();
    break;
  case WRITE_TO_FILE:
    writeToFile();
    break;
  case RESTART:
    if (restart() != 0) return 1;
    break;
  }

  return 0;
}

void FiniteStateMachine::startupFsm() {
  std::cout << " - STARTUP FSM - \n";

  start_pose_counter_ = 0;

  {
    controller_manager_msgs::LoadController::Request req;
    controller_manager_msgs::LoadController::Response resp;
    req.name = controller_name_;
    load_controller_client_.call(req, resp);
  }

  for (int i=0; i<100; ++i) {
    std_msgs::Bool msg;
    msg.data = true;
    force_sensor_calibration_pub_.publish(msg);
  }

  std::string input = "n";
  std::cout << "Start controllers? (write anything and press enter) ";
  std::cin >> input;
  std::cout << "Starting controller\n";

  {
    controller_manager_msgs::SwitchController::Request req;
    controller_manager_msgs::SwitchController::Response resp;
    req.start_controllers.push_back(controller_name_);
    req.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    switch_controller_client_.call(req, resp);
  }

  KDL::Frame start_pose = spg_->getPose(start_pose_counter_);
  publishPoseMarkers(start_pose);
  tpp_->setStartPose(start_pose);
  tpp_->publishStartPose();

  {
    // turn on gravity
  }

  std::ofstream log(controller_name_ + "_logfile.txt", std::ios_base::trunc | std::ios_base::out);
  log << "start_pose_id"
      << ",start_pose_x,start_pose_y,start_pose_z,start_pose_qx,start_pose_qy,start_pose_qz,start_pose_qw,"
      << ",traj_cycle_time,traj_distance"
      << ",conv_time,conv_timer_flag"
      << ",contact_force_x,contact_force_y,contact_force_z,contact_force_tx,contact_force_ty,contact_force_tz"
      << ",conv_pose_err_x,conv_pose_err_y,conv_pose_err_z,conv_pose_err_qx,conv_pose_err_qy,conv_pose_err_qz,conv_pose_err_qw"
      << "\n";
  log.close();

  initialize_robot_time_ = ros::Time::now();
  state_ = GOTO_START_POSE;
  std::cout << " - INITIALIZE ROBOT - \n";
  std::cout << "Current starting pose: " << start_pose_counter_ << "/" << spg_->getNPoses() << "\n";
}

void FiniteStateMachine::initializeRobot() {
  publishPoseMarkers(spg_->getPose(start_pose_counter_));
  tpp_->publishStartPose();

  std_msgs::Bool msg;
  msg.data = true;
  force_sensor_calibration_pub_.publish(msg);

  double elapsed_time = (ros::Time::now() - initialize_robot_time_).toSec();
  if (elapsed_time >= 1.0 && tpp_->reachedStartPose()) {
    goto_start_time_ = ros::Time::now();
    state_ = GOTO_START_POSE;
    std::cout << " - GOTO START POSE - \n";
    std::cout << "Current starting pose: " << start_pose_counter_ << "/" << spg_->getNPoses() << "\n";
  }
}

void FiniteStateMachine::gotoStartPose() {
  publishPoseMarkers(spg_->getPose(start_pose_counter_));
  tpp_->publishStartPose();

  double elapsed_time = (ros::Time::now() - goto_start_time_).toSec();
  if (elapsed_time >= 1.0 && tpp_->reachedStartPose()) {
    std::string input = "n";
    std::cout << "Start pose reached, continue? (write anything and press enter) ";
    std::cin >> input;
    std::cout << "Starting to move set point...\n";

    start_rtt_time_ = start_fwd_track_time_ = ros::Time::now();
    state_ = FOLLOW_TRACK_FRW;
    std::cout << " - FOLLOW TRACK FORWARD - \n";
  }
}

void FiniteStateMachine::followTrackForward() {
  publishPoseMarkers(spg_->getPose(start_pose_counter_));

  double elapsed_time = (ros::Time::now() - start_fwd_track_time_).toSec();

  tpp_->publishFrwTrackPose(elapsed_time);

  if (tpp_->finalTimeReached(elapsed_time)) {
    start_converging_time_ = ros::Time::now();
    convergence_timer_flag_ = false;
    state_ = CONVERGE_AT_END_POSE;
    std::cout << " - CONVERGE AT END POSE - \n";
  }
}

void FiniteStateMachine::convergeAtEndPose() {
  publishPoseMarkers(spg_->getPose(start_pose_counter_));

  tpp_->publishEndPose();

  static int count_frames = 0;
  if (tpp_->eeHasConverged()) {
    convergence_time_ = (ros::Time::now() - start_converging_time_).toSec();
    if (count_frames < 5) {
      count_frames++;
    } else {
      calculateRelativeEeError();
      count_frames = 0;
      std::cout << "Convergence time: " << convergence_time_ << " sec\n";
      start_back_track_time_ = ros::Time::now();
      state_ = FOLLOW_TRACK_BCK;
      std::cout << " - FOLLOW TRACK BACKWARDS - \n";
    }
  } else {
    count_frames = 0;
    double convergence_time_ = (ros::Time::now() - start_converging_time_).toSec();
    if (convergence_time_ >= 10.0) {
      calculateRelativeEeError();
      convergence_timer_flag_ = true;
      std::cout << "Convergence time: " << convergence_time_ << " sec (limit exceeded)\n";
      start_back_track_time_ = ros::Time::now();
      state_ = FOLLOW_TRACK_BCK;
      std::cout << " - FOLLOW TRACK BACKWARDS - \n";
    }
  }
}

void FiniteStateMachine::followTrackBackwards() {
  publishPoseMarkers(spg_->getPose(start_pose_counter_));

  double elapsed_time = (ros::Time::now() - start_back_track_time_).toSec();
  tpp_->publishBackTrackPose(elapsed_time);

  if (tpp_->finalTimeReached(elapsed_time)) {
    state_ = WRITE_TO_FILE;
  }
}

void FiniteStateMachine::writeToFile() {
  std::cout << " - WRITE TO FILE - \n";
  
  KDL::Frame start_pose = spg_->getPose(start_pose_counter_);

  publishPoseMarkers(start_pose);
  tpp_->publishStartPose();

  std::ofstream log(controller_name_ + "_logfile.txt", std::ios_base::app | std::ios_base::out);
  
  log << start_pose_counter_;

  log << "," << start_pose.p.x() << "," << start_pose.p.y() << "," << start_pose.p.z();
  double qx, qy, qz, qw;
  start_pose.M.GetQuaternion(qx, qy, qz, qw);
  log << "," << qx << "," << qy << "," << qz << "," << qw;

  log << "," << tpp_->getTrajectoryCycleTime() << "," << tpp_->getTrajectoryDistance();

  log << "," << convergence_time_ << "," << convergence_timer_flag_;

  resource_mutex_.lock();
  for (int i=0; i<6; ++i) log << "," << F_e_max_(i);
  resource_mutex_.unlock();

  log << "," << convergence_position_error_(0) << "," << convergence_position_error_(1) << "," << convergence_position_error_(2);
  log << "," << convergence_rotation_error_.x() << "," << convergence_rotation_error_.y() 
      << "," << convergence_rotation_error_.z() << "," << convergence_rotation_error_.w();

  log << "\n";
  log.close();

  state_ = RESTART;
}

int FiniteStateMachine::restart() {
  std::cout << " - RESTART - \n";
  start_pose_counter_++;
  if (start_pose_counter_ >= spg_->getNPoses()) return 1;
  KDL::Frame start_pose = spg_->getPose(start_pose_counter_);
  publishPoseMarkers(start_pose);
  tpp_->setStartPose(start_pose);
  tpp_->publishStartPose();
  state_ = GOTO_START_POSE;
  std::cout << " - GOTO START POSE - \n";
  std::cout << "Current starting pose: " << start_pose_counter_ << "/" << spg_->getNPoses() << "\n";
  return 0;
}

void FiniteStateMachine::calculateRelativeEeError() {
  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform("/bolt_link", "/desired_bolt_frame", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Error when looking up transform: '%s'", ex.what());
  }
  convergence_position_error_ << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
  convergence_rotation_error_ = Eigen::Quaternion<double>(transform.getRotation().getAxis().w(),
                                                   transform.getRotation().getAxis().x(),
                                                   transform.getRotation().getAxis().y(),
                                                   transform.getRotation().getAxis().z());
}




void FiniteStateMachine::publishPoseMarkers(KDL::Frame pose) {
  visualization_msgs::MarkerArray marker_array;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/" + root_link_;
    marker.header.stamp = ros::Time::now();
    marker.ns = controller_name_;
    marker.id = 10;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = pose.p.x();
    marker.pose.position.y = pose.p.y();
    marker.pose.position.z = pose.p.z();
    Eigen::Matrix3d rot;
    rot << pose.M.data[0], pose.M.data[1], pose.M.data[2], 
           pose.M.data[3], pose.M.data[4], pose.M.data[5], 
           pose.M.data[6], pose.M.data[7], pose.M.data[8];
    Eigen::Quaterniond q(rot);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(MARKER_LIFETIME);
    marker_array.markers.push_back(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/" + root_link_;
    marker.header.stamp = ros::Time::now();
    marker.ns = controller_name_;
    marker.id = 20;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = pose.p.x();
    marker.pose.position.y = pose.p.y();
    marker.pose.position.z = pose.p.z();
    Eigen::Matrix3d rot;
    rot << pose.M.data[0], pose.M.data[1], pose.M.data[2], 
           pose.M.data[3], pose.M.data[4], pose.M.data[5], 
           pose.M.data[6], pose.M.data[7], pose.M.data[8];
    Eigen::Quaterniond q(rot);
    Eigen::Quaterniond rotq(0.70710678118, 0, 0, 0.70710678118);
    Eigen::Quaterniond resq = q*rotq;
    marker.pose.orientation.x = resq.x();
    marker.pose.orientation.y = resq.y();
    marker.pose.orientation.z = resq.z();
    marker.pose.orientation.w = resq.w();
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(MARKER_LIFETIME);
    marker_array.markers.push_back(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/" + root_link_;
    marker.header.stamp = ros::Time::now();
    marker.ns = controller_name_;
    marker.id = 30;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = pose.p.x();
    marker.pose.position.y = pose.p.y();
    marker.pose.position.z = pose.p.z();
    Eigen::Matrix3d rot;
    rot << pose.M.data[0], pose.M.data[1], pose.M.data[2], 
           pose.M.data[3], pose.M.data[4], pose.M.data[5], 
           pose.M.data[6], pose.M.data[7], pose.M.data[8];
    Eigen::Quaterniond q(rot);
    Eigen::Quaterniond rotq(0.70710678118, 0, -0.70710678118, 0);
    Eigen::Quaterniond resq = q*rotq;
    marker.pose.orientation.x = resq.x();
    marker.pose.orientation.y = resq.y();
    marker.pose.orientation.z = resq.z();
    marker.pose.orientation.w = resq.w();
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(MARKER_LIFETIME);
    marker_array.markers.push_back(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/" + root_link_;
    marker.header.stamp = ros::Time::now();
    marker.ns = controller_name_;
    marker.id = 40;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = pose.p.x();
    marker.pose.position.y = pose.p.y();
    marker.pose.position.z = pose.p.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(MARKER_LIFETIME);
    marker_array.markers.push_back(marker);
  }
  marker_array_pub_.publish(marker_array);
}

// For simulation
void FiniteStateMachine::contactSensorSubscriberCallback(const compliance_controllers_msgs::ReloadContactState& msg) {
  resource_mutex_.lock();
  F_e_(0) = msg.wrench.force.x;
  F_e_(1) = msg.wrench.force.y;
  F_e_(2) = msg.wrench.force.z;
  F_e_(3) = msg.wrench.torque.x;
  F_e_(4) = msg.wrench.torque.y;
  F_e_(5) = msg.wrench.torque.z;
  for (int i=0; i<6; ++i)
    F_e_max_(i) = std::max( F_e_(i), F_e_max_(i) );
  resource_mutex_.unlock();
}