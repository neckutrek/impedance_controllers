#include "impedance_controller.h"

#include <iostream>
#include <cmath>
#include <memory>

#include "utilities.h"

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pluginlib/class_list_macros.h>

#define MARKER_LIFETIME 10

namespace irb4400 {

  void ImpedanceController::initialize() {
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
    tau_e_ = Eigen::VectorXd::Zero(getNJoints());
    ddr_ = Eigen::VectorXd::Zero(6);
    dq_ = Eigen::VectorXd::Zero(getNJoints());
    ddq_ = Eigen::VectorXd::Zero(getNJoints());
    tau_u_ = Eigen::VectorXd::Zero(getNJoints());

    H_.resize(getNJoints());
    Cq_.resize(getNJoints());
    g_.resize(getNJoints());

    getRobotState()->kdl_tree_.getChain(root_link_name_, ee_link_name_, kdl_ee_chain_);
    getRobotState()->kdl_tree_.getChain(root_link_name_, tip_link_name_, kdl_tip_chain_);

    fk_solver_pos_ = std::make_shared<KDL::TreeFkSolverPos_recursive>(getRobotState()->kdl_tree_);
    fk_solver_jac_ = std::make_shared<KDL::TreeJntToJacSolver>(getRobotState()->kdl_tree_);
    fk_solver_jac_dot_ = std::make_shared<KDL::ChainJntToJacDotSolver>(kdl_ee_chain_);

    KDL::Vector gravity = KDL::Vector::Zero();
    gravity(2) = -9.81;
    id_solver_ = std::make_shared<KDL::ChainDynParam>(kdl_tip_chain_, gravity);

    ee_jacobian_.resize(getRobotState()->kdl_jnt_array_vel_.q.rows());
    ee_dot_jacobian_.resize(getRobotState()->kdl_jnt_array_vel_.q.rows());
    ee_jacobian_pinv_ = Eigen::MatrixXd::Zero(6, getNJoints());

    computeForwardKinematics();
    //msd_.r_ = ee_.r_;

    double initial_trajectory_pose[6] = {0, 0, 0, 0, 0, 0};
    XmlRpc::XmlRpcValue initial_trajectory_pose_xml;
    if (getControllerNodeHandle().getParam("initial_trajectory_pose", initial_trajectory_pose_xml)) {
      for (int i=0; i<6; ++i)
        initial_trajectory_pose[i] = static_cast<double>(initial_trajectory_pose_xml[i]);
    }
    tf_.r_ << initial_trajectory_pose[0], initial_trajectory_pose[1], initial_trajectory_pose[2];
    tf_.R_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    initial_joint_configuration_ = Eigen::VectorXd::Zero(getNJoints());
    XmlRpc::XmlRpcValue initial_joint_configuration_xml;
    if (getControllerNodeHandle().getParam("initial_joint_configuration", initial_joint_configuration_xml)) {
      for (int i=0; i<getNJoints(); ++i)
        initial_joint_configuration_[i] = static_cast<double>(initial_joint_configuration_xml[i]);
    }

    trajectory_subscriber_ = getControllerNodeHandle().subscribe(
      "trajectory_frame", 1000, &ImpedanceController::trajectorySubscriberCallback, this);

    contact_sensor_subscriber_ = getControllerNodeHandle().subscribe(
      "contact_forces", 1000, &ImpedanceController::contactSensorSubscriberCallback, this);

    marker_array_pub_ = getControllerNodeHandle().advertise<visualization_msgs::MarkerArray>(
      "visualization_marker", 1);

    ee_pose_pub_ = getControllerNodeHandle().advertise<compliance_controllers_msgs::ReloadEEPose>("ee_state", 1);
  }

  void ImpedanceController::computeControls(Eigen::VectorXd& u) {
    renderEndEffectorAndTrajectoryPoint();
    publishEEPose();

    computeForwardKinematics();
    computeDdq();
    computeTau();
    updateMSD();
    u = tau_u_;
  }

  void ImpedanceController::computeForwardKinematics() {
    const KDL::JntArray& q = getRobotState()->kdl_jnt_array_vel_.value();
    const KDL::JntArray& dq = getRobotState()->kdl_jnt_array_vel_.deriv();

    fk_solver_pos_->JntToCart(q, ee_pose_, ee_link_name_);
    fk_solver_jac_->JntToJac(q, ee_jacobian_, ee_link_name_);
    fk_solver_jac_dot_->JntToJacDot(getRobotState()->kdl_jnt_array_vel_, ee_dot_jacobian_);

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

  void ImpedanceController::computeDdq() {
    // static bool initial_state_reached = false;
    // static int initial_state_counter = 0;

    // if (initial_state_reached) {
      const KDL::JntArray& dq = getRobotState()->kdl_jnt_array_vel_.deriv();
      ddq_ = ee_jacobian_pinv_ * (ddr_ - ee_dot_jacobian_.data * dq.data);
    // } else {
    //   Eigen::VectorXd q = getRobotState()->kdl_jnt_array_vel_.value().data;
    //   Eigen::VectorXd& q_star = initial_joint_configuration_;

    //   Eigen::VectorXd dq = getRobotState()->kdl_jnt_array_vel_.deriv().data;

    //   Eigen::VectorXd e = q_star - q;

    //   static Eigen::VectorXd int_part = Eigen::VectorXd::Zero(6);
    //   double dt = getRobotState()->sampling_time_;
    //   int_part += e*dt;

    //   double p = 100;
    //   double d = 200;
    //   double i = 300;
    //   ddq_ = p*e - d*dq + i*int_part;

    //   double e_err = std::abs(e.dot(e));
    //   double dq_err = std::abs(dq.dot(dq));
    //   //std::cout << e_err << "     " << dq_err << "\n";

    //   if (e_err <= 0.25 && dq_err <= 250) {
    //     initial_state_counter++;
    //     if (initial_state_counter > 2) {
    //       //std::cout << "\nINITIAL STATE REACHED!\n\n";
    //       initial_state_reached = true;
    //     }
    //   } else {
    //     initial_state_counter = 0;
    //   }
    // }
  }

  void ImpedanceController::computeTau() {
    const KDL::JntArray& q = getRobotState()->kdl_jnt_array_vel_.value();
    const KDL::JntArray& dq = getRobotState()->kdl_jnt_array_vel_.deriv();

    id_solver_->JntToMass(q, H_);
    id_solver_->JntToCoriolis(q, dq, Cq_);
    id_solver_->JntToGravity(q, g_);

    tau_e_ = ee_jacobian_.data.transpose() * F_e_;

    // std::cout << "tau_u_.size = " << tau_u_.rows() << "x" << tau_u_.cols() << "\n";
    // std::cout << "H_.data.size = " << H_.data.rows() << "x" << H_.data.cols() << "\n";
    // std::cout << "ddq_.size = " << ddq_.rows() << "x" << ddq_.cols() << "\n";
    // std::cout << "Cq_.data.size = " << Cq_.data.rows() << "x" << Cq_.data.cols() << "\n";
    // std::cout << "g_.data.size = " << g_.data.rows() << "x" << g_.data.cols() << "\n";
    // std::cout << "tau_e_.size = " << tau_e_.rows() << "x" << tau_e_.cols() << "\n";

    tau_u_ = H_.data * ddq_ + Cq_.data + g_.data - tau_e_;
  }

  void ImpedanceController::updateMSD() {
    Eigen::VectorXd e = Eigen::VectorXd::Zero(6);
    e.head(3) = ee_.r_ - tf_.r_;
    e.tail(3) = 0.5 * (tf_.R_.col(0).cross(ee_.R_.col(0)) + 
                       tf_.R_.col(1).cross(ee_.R_.col(1)) + 
                       tf_.R_.col(2).cross(ee_.R_.col(2)));

    Eigen::VectorXd de = Eigen::VectorXd::Zero(6);
    de.head(3) = ee_.dr_.head(3) - tf_.dr_.head(3);
    de.tail(3) = ee_.w_ - tf_.w_;

    Eigen::VectorXd dde = Eigen::VectorXd::Zero(6);
    dde.head(3) = tf_.ddr_;
    dde.tail(3) = tf_.a_;

    resource_mutex_.lock();
    //F_e_ = Eigen::VectorXd::Zero(6);
    ddr_ = Md_.inverse() * (F_e_ - Kd_*de - Kp_*e) + dde;
    resource_mutex_.unlock();
  }

  void ImpedanceController::contactSensorSubscriberCallback(const compliance_controllers_msgs::ReloadContactState& msg) {
    resource_mutex_.lock();
    F_e_(0) = msg.wrench.force.x;
    F_e_(1) = msg.wrench.force.y;
    F_e_(2) = msg.wrench.force.z;
    F_e_(3) = msg.wrench.torque.x;
    F_e_(4) = msg.wrench.torque.y;
    F_e_(5) = msg.wrench.torque.z;
    resource_mutex_.unlock();

    // double eps = 1e-4;
    // if (F_e_(0) > eps || F_e_(1) > eps || F_e_(2) > eps || F_e_(3) > eps || F_e_(4) > eps || F_e_(5) > eps)
    //   std::cout << "F_e_ = " << F_e_.transpose() << "\n";
  }

  void ImpedanceController::trajectorySubscriberCallback(const compliance_controllers_msgs::ReloadTrajectoryFrame& msg) {
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

  void ImpedanceController::publishEEPose() {
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

  void ImpedanceController::renderEndEffectorAndTrajectoryPoint() {
    visualization_msgs::MarkerArray marker_array;

    // Visualize trajectory frame (RED)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "impedance_controller";
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
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(MARKER_LIFETIME);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "impedance_controller";
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
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(MARKER_LIFETIME);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "impedance_controller";
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
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(MARKER_LIFETIME);
      marker_array.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "impedance_controller";
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
      marker.lifetime = ros::Duration(MARKER_LIFETIME);
      marker_array.markers.push_back(marker);
    }

    // Visualize MSD (RED)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "impedance_controller";
      marker.id = 5;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = ee_.r_(0);
      marker.pose.position.y = ee_.r_(1);
      marker.pose.position.z = ee_.r_(2);
      Eigen::Quaterniond q(ee_.R_);
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
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "impedance_controller";
      marker.id = 6;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = ee_.r_(0);
      marker.pose.position.y = ee_.r_(1);
      marker.pose.position.z = ee_.r_(2);
      Eigen::Quaterniond q(ee_.R_);
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
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "impedance_controller";
      marker.id = 7;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = ee_.r_(0);
      marker.pose.position.y = ee_.r_(1);
      marker.pose.position.z = ee_.r_(2);
      Eigen::Quaterniond q(ee_.R_);
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
      marker.header.frame_id = "/" + root_link_name_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "impedance_controller";
      marker.id = 8;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = ee_.r_(0);
      marker.pose.position.y = ee_.r_(1);
      marker.pose.position.z = ee_.r_(2);
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
      marker.lifetime = ros::Duration(MARKER_LIFETIME);
      marker_array.markers.push_back(marker);
    }
    marker_array_pub_.publish(marker_array);
  }

}

PLUGINLIB_EXPORT_CLASS(irb4400::ImpedanceController, controller_interface::ControllerBase)
