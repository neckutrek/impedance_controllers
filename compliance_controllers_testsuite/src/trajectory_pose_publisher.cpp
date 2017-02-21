#include <trajectory_pose_publisher.h>

TrajectoryPosePublisher::TrajectoryPosePublisher(ros::NodeHandle &nh, std::string controller_name)
: nh_(nh), controller_name_(controller_name) {
  trajectory_publisher_ = nh_.advertise<compliance_controllers_msgs::ReloadTrajectoryFrame>(controller_name_ + "/trajectory_frame", 1000);
  ee_state_subscriber_ = nh_.subscribe(controller_name_ + "/ee_state", 1000, &TrajectoryPosePublisher::eeStateCallback, this);
  start_pose_.M = KDL::Rotation::Identity();
  start_pose_.p = KDL::Vector(0.8, 0.0, 1.2);
}

void TrajectoryPosePublisher::init(double T, double min, double max) {
  T_ = T;
  min_ = min;
  max_ = max;
}

void TrajectoryPosePublisher::setStartPose(KDL::Frame pose) {
  start_pose_ = pose;

  std::vector<double> pos = getPosition(T_);
  std::vector<double> rot = getOrientation(T_);
  end_pose_.p.x(pos.at(0));
  end_pose_.p.y(pos.at(1));
  end_pose_.p.z(pos.at(2));
  for (int i=0; i<9; ++i)
    end_pose_.M.data[i] = rot.at(i);
}

void TrajectoryPosePublisher::publishStartPose() {
  compliance_controllers_msgs::ReloadTrajectoryFrame rtf;
  rtf.position = {start_pose_.p.x(), start_pose_.p.y(), start_pose_.p.z()};
  rtf.velocity = {0, 0, 0};
  rtf.acceleration = {0, 0, 0};
  rtf.orientation = {start_pose_.M.data[0], start_pose_.M.data[1], start_pose_.M.data[2],
                     start_pose_.M.data[3], start_pose_.M.data[4], start_pose_.M.data[5],
                     start_pose_.M.data[6], start_pose_.M.data[7], start_pose_.M.data[8]};
  rtf.angular_velocity = {0, 0, 0};
  rtf.angular_acceleration = {0, 0, 0};

  trajectory_publisher_.publish(rtf);
}

void TrajectoryPosePublisher::publishFrwTrackPose(double elapsed_time) {
  compliance_controllers_msgs::ReloadTrajectoryFrame rtf;
  rtf.position = getPosition(elapsed_time);
  rtf.velocity = getVelocity(elapsed_time);
  rtf.acceleration = getAcceleration(elapsed_time);
  rtf.orientation = getOrientation(elapsed_time);
  rtf.angular_velocity = getAngularVelocity(elapsed_time);
  rtf.angular_acceleration = getAngularAcceleration(elapsed_time);

  trajectory_publisher_.publish(rtf);
}

void TrajectoryPosePublisher::publishEndPose() {
  compliance_controllers_msgs::ReloadTrajectoryFrame rtf;
  rtf.position = {end_pose_.p.x(), end_pose_.p.y(), end_pose_.p.z()};
  rtf.velocity = {0, 0, 0};
  rtf.acceleration = {0, 0, 0};
  rtf.orientation = {end_pose_.M.data[0], end_pose_.M.data[1], end_pose_.M.data[2],
                     end_pose_.M.data[3], end_pose_.M.data[4], end_pose_.M.data[5],
                     end_pose_.M.data[6], end_pose_.M.data[7], end_pose_.M.data[8]};
  rtf.angular_velocity = {0, 0, 0};
  rtf.angular_acceleration = {0, 0, 0};

  trajectory_publisher_.publish(rtf);
}

void TrajectoryPosePublisher::publishBackTrackPose(double elapsed_time) {
  compliance_controllers_msgs::ReloadTrajectoryFrame rtf;
  rtf.position = getPosition(elapsed_time + T_);
  rtf.velocity = getVelocity(elapsed_time + T_);
  rtf.acceleration = getAcceleration(elapsed_time + T_);
  rtf.orientation = getOrientation(elapsed_time + T_);
  rtf.angular_velocity = getAngularVelocity(elapsed_time + T_);
  rtf.angular_acceleration = getAngularAcceleration(elapsed_time + T_);

  trajectory_publisher_.publish(rtf);
}

bool TrajectoryPosePublisher::reachedStartPose() {
  double error = 0;
  resource_mutex_.lock();
    error += std::abs(current_ee_pose_.p.x() - start_pose_.p.x());
    error += std::abs(current_ee_pose_.p.y() - start_pose_.p.y());
    //error += std::abs(current_ee_pose_.p.z() - start_pose_.p.z());
    for (int i=0; i<9; ++i) {
      error += std::abs(current_ee_pose_.M.data[i] - start_pose_.M.data[i]);
    }
  resource_mutex_.unlock();
  if (error <= 0.05) return true; // use 0.25 for admittance controller (with gravity), 0.05 for impedance controller

  return false;
}

bool TrajectoryPosePublisher::eeHasConverged() {
  double sum_vel = 0;
  resource_mutex_.lock();
    sum_vel += std::abs(current_ee_vel_.x());
    sum_vel += std::abs(current_ee_vel_.y());
    //sum_vel += std::abs(current_ee_vel_.z());
    sum_vel += std::abs(current_ee_ang_vel_.x());
    sum_vel += std::abs(current_ee_ang_vel_.y());
    sum_vel += std::abs(current_ee_ang_vel_.z());
  resource_mutex_.unlock();
  if (sum_vel <= 0.05) return true; // use 0.25 for admittance controller (with gravity), 0.05 for impedance controller

  return false;
}

void TrajectoryPosePublisher::eeStateCallback(const compliance_controllers_msgs::ReloadEEPose& msg) {
  resource_mutex_.lock();
  
  current_ee_pose_.p.x(msg.position[0]);
  current_ee_pose_.p.y(msg.position[1]);
  current_ee_pose_.p.z(msg.position[2]);

  current_ee_vel_.x(msg.velocity[0]);
  current_ee_vel_.y(msg.velocity[1]);
  current_ee_vel_.z(msg.velocity[2]);

  for (int i=0; i<9; ++i)
    current_ee_pose_.M.data[i] = msg.orientation[i];

  current_ee_ang_vel_.x(msg.angular_velocity[0]);
  current_ee_ang_vel_.y(msg.angular_velocity[1]);
  current_ee_ang_vel_.z(msg.angular_velocity[2]);

  resource_mutex_.unlock();
}

















std::vector<double> TrajectoryPosePublisher::getPosition(double t) {
  double val1 = (std::sin(3.1415/T_ * t)+1)/2 * (max_-min_) + min_;
  double val2 = (std::cos(3.1415/T_ * t)+1)/2 * (max_-min_) + min_;

  double val3 = (-std::cos(3.1415 * t/T_)+1)/2 * (max_-min_) + min_;
  KDL::Vector val3_vec(val3, 0, 0);
  val3_vec = start_pose_.M * val3_vec;

  return {start_pose_.p.x() + val3_vec.x(), 
          start_pose_.p.y() + val3_vec.y(), 
          start_pose_.p.z() + val3_vec.z()};
}

std::vector<double> TrajectoryPosePublisher::getVelocity(double t) {
  double dval1 =  3.1415/(2*T_) * (max_-min_) * std::cos(3.1415/T_ * t);
  double dval2 = -3.1415/(2*T_) * (max_-min_) * std::sin(3.1415/T_ * t);

  double dval3 = 3.1415/(2*T_) * (max_-min_) * std::sin(3.1415/T_ * t);
  KDL::Vector dval3_vec(dval3, 0, 0);
  dval3_vec = start_pose_.M * dval3_vec;

  return {dval3_vec.x(), dval3_vec.y(), dval3_vec.z()};
}

std::vector<double> TrajectoryPosePublisher::getAcceleration(double t) {
  double ddval1 =  3.1415*3.1415/(2*T_*T_) * (max_-min_) * std::sin(3.1415/T_ * t);
  double ddval2 = -3.1415*3.1415/(2*T_*T_) * (max_-min_) * std::cos(3.1415/T_ * t);

  double ddval3 = 3.1415*3.1415/(2*T_*T_) * (max_-min_) * std::cos(3.1415/T_ * t);
  KDL::Vector ddval3_vec(ddval3, 0, 0);
  ddval3_vec = start_pose_.M * ddval3_vec;

  return {ddval3_vec.x(), ddval3_vec.y(), ddval3_vec.z()};
}

std::vector<double> TrajectoryPosePublisher::getOrientation(double t) {
  double angle = 0.5*T_/(2*3.1415)*(1-std::cos(2*3.1415/T_*t));
  double sin = std::sin( angle-1.57079632679 );
  double cos = std::cos( angle-1.57079632679 );
  
   // return {cos, 0, sin,
   //         0, 1, 0,
   //         -sin, 0, cos};

  return {start_pose_.M.data[0], start_pose_.M.data[1], start_pose_.M.data[2],
          start_pose_.M.data[3], start_pose_.M.data[4], start_pose_.M.data[5],
          start_pose_.M.data[6], start_pose_.M.data[7], start_pose_.M.data[8]};
}

std::vector<double> TrajectoryPosePublisher::getAngularVelocity(double t) {
  double val = 0.5*std::sin(2*3.1415/T_*t);
  return {0, 0, 0};
}

std::vector<double> TrajectoryPosePublisher::getAngularAcceleration(double t) {
  double dval = 0.5*2*3.1415/T_*std::cos(2*3.1415/T_*t);
  return {0, 0, 0};
}