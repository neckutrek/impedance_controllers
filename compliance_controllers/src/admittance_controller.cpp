#include "admittance_controller.h"

#include <iostream>
#include <cmath>
#include <memory>

#include "utilities.h"

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/JointState.h>

#include <pluginlib/class_list_macros.h>

#define MARKER_LIFETIME 10

namespace irb4400 {

  void AdmittanceController::initialize() {
    getControllerNodeHandle().getParam("root_name", root_link_name_);
    getControllerNodeHandle().getParam("ee_name", ee_link_name_);
    getControllerNodeHandle().getParam("tip_name", tip_link_name_);

    Md_ = Eigen::MatrixXd::Zero(6, 6);
    Kd_ = Eigen::MatrixXd::Zero(6, 6);
    Kp_ = Eigen::MatrixXd::Zero(6, 6);

    XmlRpc::XmlRpcValue mass_xml;
    if (getControllerNodeHandle().getParam("mass", mass_xml)) {
      for (int i=0; i<6; ++i) {
        Md_(i,i) = static_cast<double>(mass_xml[i]);
      }
    }

    XmlRpc::XmlRpcValue damping_xml;
    if (getControllerNodeHandle().getParam("damping", damping_xml)) {
      for (int i=0; i<6; ++i)
        Kd_(i,i) = static_cast<double>(damping_xml[i]);
    }

    XmlRpc::XmlRpcValue stiffness_xml;
    if (getControllerNodeHandle().getParam("stiffness", stiffness_xml)) {
      for (int i=0; i<6; ++i)
        Kp_(i,i) = static_cast<double>(stiffness_xml[i]);
    }

    getControllerNodeHandle().getParam("pinv_damp_coeff", damp_coeff_);

    F_e_ = Eigen::VectorXd::Zero(6);

    fk_solver_pos_ = std::make_shared<KDL::TreeFkSolverPos_recursive>(getRobotState()->kdl_tree_);
    fk_solver_jac_ = std::make_shared<KDL::TreeJntToJacSolver>(getRobotState()->kdl_tree_);

    ee_jacobian_.resize(getRobotState()->kdl_jnt_array_vel_.q.rows());
    ee_jacobian_pinv_ = Eigen::MatrixXd::Zero(3, getNJoints());

    computeForwardKinematics();
    msd_.r_ = ee_.r_;
    msd_.R_ = ee_.R_;

    double initial_trajectory_pose[6];
    XmlRpc::XmlRpcValue initial_trajectory_pose_xml;
    if (getControllerNodeHandle().getParam("initial_trajectory_pose", initial_trajectory_pose_xml)) {
      for (int i=0; i<6; ++i)
        initial_trajectory_pose[i] = static_cast<double>(initial_trajectory_pose_xml[i]);
    }
    tf_.r_ << initial_trajectory_pose[0], initial_trajectory_pose[1], initial_trajectory_pose[2];
    tf_.R_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    trajectory_subscriber_ = getControllerNodeHandle().subscribe(
      "trajectory_frame", 1000, &AdmittanceController::trajectorySubscriberCallback, this);

    // For simulation
    // contact_sensor_subscriber_ = getControllerNodeHandle().subscribe(
    //   "contact_forces", 1000, &AdmittanceController::contactSensorSubscriberCallback, this);

    // For in-real-life
    contact_sensor_subscriber_ = getControllerNodeHandle().subscribe(
      "/ethdaq_data", 1000, &AdmittanceController::contactSensorSubscriberCallback, this);

    marker_array_pub_ = getControllerNodeHandle().advertise<visualization_msgs::MarkerArray>(
      "visualization_marker", 1);

    ee_pose_pub_ = getControllerNodeHandle().advertise<compliance_controllers_msgs::ReloadEEPose>("ee_state", 1);

    blahblahblah_pub_ = getControllerNodeHandle().advertise<sensor_msgs::JointState>("blah_msg", 1);
  }

  void AdmittanceController::computeControls(Eigen::VectorXd& u) {
    renderEndEffectorAndTrajectoryPoint();
    publishEEPose();
    publishBlahblahblah(u);

    computeForwardKinematics();

    Eigen::VectorXd drw = Eigen::VectorXd::Zero(6);
    drw.head(3) = msd_.dr_;
    drw.tail(3) = msd_.w_;

    u = ee_jacobian_pinv_ * drw;

    static int n = 0;
    double f = 3;
    double speed = 191*M_PI/180;
    double sin = std::sin(n*f/125*2*M_PI);
    double sine = speed * sin;
    double square = speed * (sin >= 0 ? 1 : -1);
    //u << 0, 0, 0, 0, 0, speed*sin;
    //u << 0, 0, 0, 0, 0, sine;
    n++;

    //u << 0, 0, 0, 0, 0, 0;
  
    updateMSD();
  }

  void AdmittanceController::publishBlahblahblah(const Eigen::VectorXd& u) {
    const KDL::JntArray& q = getRobotState()->kdl_jnt_array_vel_.value();
    const KDL::JntArray& dq = getRobotState()->kdl_jnt_array_vel_.deriv();
    static int seq=0;
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp =ros::Time::now ();
    for (int i=0; i<6; ++i) {
      //joint_state.position.push_back(q(i));
      joint_state.velocity.push_back(dq(i)); //actual velocity
      joint_state.effort.push_back(u(i)); //desired velocity
    }

    blahblahblah_pub_.publish(joint_state);
    seq+=1;
  }

  void AdmittanceController::computeForwardKinematics() {
    const KDL::JntArray& q = getRobotState()->kdl_jnt_array_vel_.value();
    const KDL::JntArray& dq = getRobotState()->kdl_jnt_array_vel_.deriv();

    fk_solver_pos_->JntToCart(q, ee_pose_, ee_link_name_);
    fk_solver_jac_->JntToJac(q, ee_jacobian_, ee_link_name_);
    ee_jacobian_pinv_ = dampedPinv(ee_jacobian_.data, damp_coeff_);

    ee_.r_(0) = ee_pose_.p.data[0];
    ee_.r_(1) = ee_pose_.p.data[1];
    ee_.r_(2) = ee_pose_.p.data[2];

    Eigen::VectorXd drw = ee_jacobian_.data * dq.data;
    ee_.dr_ = drw.head(3);
    ee_.R_ << ee_pose_.M.data[0], ee_pose_.M.data[1], ee_pose_.M.data[2],
              ee_pose_.M.data[3], ee_pose_.M.data[4], ee_pose_.M.data[5],
              ee_pose_.M.data[6], ee_pose_.M.data[7], ee_pose_.M.data[8];
    ee_.w_ = drw.tail(3);
  }

  void AdmittanceController::updateMSD() {
    // static int i=1;
    // if (i%125 == 0) {
    //   msd_.r_ = ee_.r_;
    //   msd_.R_ = ee_.R_;
    //   msd_.dr_ = ee_.dr_;
    //   msd_.w_ = ee_.w_;
    //   i -= 100;
    // }
    // ++i;

    Eigen::VectorXd e = Eigen::VectorXd::Zero(6);
    e.head(3) = msd_.r_ - tf_.r_;
    e.tail(3) = 0.5 * (tf_.R_.col(0).cross(msd_.R_.col(0)) + 
                       tf_.R_.col(1).cross(msd_.R_.col(1)) + 
                       tf_.R_.col(2).cross(msd_.R_.col(2)));

    Eigen::VectorXd de = Eigen::VectorXd::Zero(6);
    de.head(3) = msd_.dr_.head(3) - tf_.dr_.head(3);
    de.tail(3) = msd_.w_ - tf_.w_;

    Eigen::VectorXd dde = Eigen::VectorXd::Zero(6);
    dde.head(3) = tf_.ddr_;
    dde.tail(3) = tf_.a_;

    resource_mutex_.lock();
    //F_e_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ddr = Md_.inverse() * (F_e_ - Kd_*de - Kp_*e) + dde;
    msd_.ddr_ = ddr.head(3);
    msd_.a_ = ddr.tail(3);
    resource_mutex_.unlock();

    double dt = getRobotState()->sampling_time_;
    msd_.r_ = msd_.r_ + dt*msd_.dr_;     // msd_.r_(k+1)
    msd_.dr_ = msd_.dr_ + dt*msd_.ddr_;  // msd_.dr_(k+1)

    Eigen::Matrix3d addangle;
    if (msd_.w_(0) == 0 && msd_.w_(1) == 0 && msd_.w_(2) == 0)
      addangle = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());  
    else
      addangle = Eigen::AngleAxisd(dt*msd_.w_.norm(), msd_.w_.normalized());

    msd_.R_ = addangle * msd_.R_;
    msd_.w_ = msd_.w_ + dt*msd_.a_;
  }

  void AdmittanceController::publishEEPose() {
    compliance_controllers_msgs::ReloadEEPose msg;
    
    msg.position.push_back(ee_.r_(0));
    msg.position.push_back(ee_.r_(1));
    msg.position.push_back(ee_.r_(2));

    msg.velocity.push_back(ee_.dr_(0));
    msg.velocity.push_back(ee_.dr_(1));
    msg.velocity.push_back(ee_.dr_(2));

    for (int i=0; i<9; ++i)
      msg.orientation.push_back(ee_.R_(i/3, i%3));

    msg.angular_velocity.push_back(ee_.w_(0));
    msg.angular_velocity.push_back(ee_.w_(1));
    msg.angular_velocity.push_back(ee_.w_(2));

    ee_pose_pub_.publish(msg);
  }

  void AdmittanceController::trajectorySubscriberCallback(const compliance_controllers_msgs::ReloadTrajectoryFrame& msg) {
    resource_mutex_.lock();
    tf_.r_ << msg.position.at(0), msg.position.at(1), msg.position.at(2);
    tf_.dr_ << msg.velocity.at(0), msg.velocity.at(1), msg.velocity.at(2);
    tf_.ddr_ << msg.acceleration.at(0), msg.acceleration.at(1), msg.acceleration.at(2);
    tf_.R_ << msg.orientation.at(0), msg.orientation.at(1), msg.orientation.at(2),
              msg.orientation.at(3), msg.orientation.at(4), msg.orientation.at(5),
              msg.orientation.at(6), msg.orientation.at(7), msg.orientation.at(8);
    tf_.w_ << msg.angular_velocity.at(0), msg.angular_velocity.at(1), msg.angular_velocity.at(2);
    tf_.a_ << msg.angular_acceleration.at(0), msg.angular_acceleration.at(1), msg.angular_acceleration.at(2);
    resource_mutex_.unlock();
  }

  // For simulation
  // void AdmittanceController::contactSensorSubscriberCallback(const compliance_controllers_msgs::ReloadContactState& msg) {
  //   resource_mutex_.lock();
  //   F_e_(0) = msg.wrench.force.x;
  //   F_e_(1) = msg.wrench.force.y;
  //   F_e_(2) = msg.wrench.force.z;
  //   F_e_(3) = msg.wrench.torque.x;
  //   F_e_(4) = msg.wrench.torque.y;
  //   F_e_(5) = msg.wrench.torque.z;
  //   resource_mutex_.unlock();

  //   double eps = 1e-4;
  //   if (F_e_(0) > eps || F_e_(1) > eps || F_e_(2) > eps || F_e_(3) > eps || F_e_(4) > eps || F_e_(5) > eps)
  //     std::cout << "F_e_ = " << F_e_.transpose() << "\n";
  // }

  // For in-real-life (optoforce sensor)
  void AdmittanceController::contactSensorSubscriberCallback(const geometry_msgs::WrenchStamped& msg) {
    double alpha = 0.95;

    resource_mutex_.lock();
    F_e_(0) = (1-alpha) * msg.wrench.force.z + alpha*F_e_(0);
    F_e_(1) = (1-alpha) * msg.wrench.force.y + alpha*F_e_(1);
    F_e_(2) = (1-alpha) * (-msg.wrench.force.x) + alpha*F_e_(2);
    F_e_(3) = (1-alpha) * msg.wrench.torque.z + alpha*F_e_(3);
    F_e_(4) = (1-alpha) * msg.wrench.torque.y + alpha*F_e_(4);
    F_e_(5) = (1-alpha) * (-msg.wrench.torque.x) + alpha*F_e_(5);

    double eps = 0.07;
    for (int i=0; i<6; ++i) {
      F_e_(i) = (std::abs(F_e_(i)) < eps ? 0 : F_e_(i));
    }
    resource_mutex_.unlock();

    // double eps = 1e-4;
    // if (F_e_(0) > eps || F_e_(1) > eps || F_e_(2) > eps || F_e_(3) > eps || F_e_(4) > eps || F_e_(5) > eps)
    //   std::cerr << "F_e_ = " << F_e_.transpose() << "\n";
  }

  void AdmittanceController::renderEndEffectorAndTrajectoryPoint() {
    visualization_msgs::MarkerArray marker_array;

    // Visualize trajectory frame
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "admittance_controller";
      marker.id = 1;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = tf_.r_(0);
      marker.pose.position.y = tf_.r_(1);
      marker.pose.position.z = tf_.r_(2);
      Eigen::Quaterniond q(tf_.R_);
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();
      marker.scale.x = 0.1;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.r = 1.0*0.5;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "admittance_controller";
      marker.id = 2;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = tf_.r_(0);
      marker.pose.position.y = tf_.r_(1);
      marker.pose.position.z = tf_.r_(2);
      Eigen::Quaterniond q(tf_.R_);
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
      marker.color.g = 1.0*0.5;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "admittance_controller";
      marker.id = 3;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = tf_.r_(0);
      marker.pose.position.y = tf_.r_(1);
      marker.pose.position.z = tf_.r_(2);
      Eigen::Quaterniond q(tf_.R_);
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
      marker.color.b = 1.0*0.5;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "admittance_controller";
      marker.id = 4;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = tf_.r_(0);
      marker.pose.position.y = tf_.r_(1);
      marker.pose.position.z = tf_.r_(2);
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }

    // Visualize MSD
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "admittance_controller";
      marker.id = 5;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = msd_.r_(0);
      marker.pose.position.y = msd_.r_(1);
      marker.pose.position.z = msd_.r_(2);
      Eigen::Quaterniond q(msd_.R_);
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
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "admittance_controller";
      marker.id = 6;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = msd_.r_(0);
      marker.pose.position.y = msd_.r_(1);
      marker.pose.position.z = msd_.r_(2);
      Eigen::Quaterniond q(msd_.R_);
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
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "admittance_controller";
      marker.id = 7;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = msd_.r_(0);
      marker.pose.position.y = msd_.r_(1);
      marker.pose.position.z = msd_.r_(2);
      Eigen::Quaterniond q(msd_.R_);
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
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "admittance_controller";
      marker.id = 8;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = msd_.r_(0);
      marker.pose.position.y = msd_.r_(1);
      marker.pose.position.z = msd_.r_(2);
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }
    marker_array_pub_.publish(marker_array);
  }

}

PLUGINLIB_EXPORT_CLASS(irb4400::AdmittanceController, controller_interface::ControllerBase)
