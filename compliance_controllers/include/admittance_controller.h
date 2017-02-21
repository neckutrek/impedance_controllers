#ifndef ADMITTANCE_VELOCITY_CONTROLLER_H
#define ADMITTANCE_VELOCITY_CONTROLLER_H

#include <mutex>

#include "base_controller.h"
#include "trajectory_frame.h"

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/WrenchStamped.h> // msg type published by OptoForce sensor

//#include <gazebo_msgs/ContactsState.h>

#include <kdl/jacobian.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

#include <compliance_controllers_msgs/ReloadTrajectoryFrame.h>
#include <compliance_controllers_msgs/ReloadContactState.h>
#include <compliance_controllers_msgs/ReloadEEPose.h>

namespace irb4400 {

  typedef 
  controller_interface::Controller<hardware_interface::VelocityJointInterface>
  JointVelocityController;

  typedef 
  hardware_interface::VelocityJointInterface 
  JointVelocityInterface;

  /*! \brief A joint velocity controller that mimics a mass-spring-damper system following a trajectory.
   *  \author Marcus A Johansson */
  class AdmittanceController : public BaseController<JointVelocityController, JointVelocityInterface> {
  public:
    AdmittanceController() = default;

    ~AdmittanceController() noexcept = default;

    void initialize();

    void computeControls(Eigen::VectorXd& u);

  private:
    AdmittanceController(const AdmittanceController& other) = delete;
    AdmittanceController(AdmittanceController&& other) = delete;
    AdmittanceController& operator=(const AdmittanceController& other) = delete;
    AdmittanceController& operator=(AdmittanceController&& other) noexcept = delete;

    void trajectorySubscriberCallback(const compliance_controllers_msgs::ReloadTrajectoryFrame& msg);
    //void contactSensorSubscriberCallback(const compliance_controllers_msgs::ReloadContactState& msg);
    void contactSensorSubscriberCallback(const geometry_msgs::WrenchStamped& msg);

    void computeForwardKinematics();
    void updateMSD();

    void renderEndEffectorAndTrajectoryPoint();
    void publishEEPose();
    void publishBlahblahblah(const Eigen::VectorXd& u);

    std::string                           root_link_name_;
    std::string                           ee_link_name_;
    std::string                           tip_link_name_;

    TrajectoryFrame                       tf_; // the desired trajectory got externally
    TrajectoryFrame                       msd_; // the trajectory of the mass-spring-damper
    TrajectoryFrame                       ee_; // the trajectory of the actual end effector

    Eigen::MatrixXd                       Md_; // mass of the mass-spring-damper system
    Eigen::MatrixXd                       Kd_; // damping constant of the mass-spring-damper system
    Eigen::MatrixXd                       Kp_; // spring constant of the mass-spring-damper system

    double                                damp_coeff_; // the damped pseudo-inverse damping coefficient

    Eigen::VectorXd                       F_e_; // the current external force and torque sensed by the contact sensor

    KDL::Frame                            ee_pose_; // end-effector pose
    KDL::Jacobian                         ee_jacobian_; // end-effector jacobian
    KDL::Jacobian                         ee_dot_jacobian_; // end-effector time derivative of jacobian
    Eigen::MatrixXd                       ee_jacobian_pinv_; // pseudo inverse of translation jacobian

    std::shared_ptr<KDL::TreeFkSolverPos_recursive>  fk_solver_pos_; // forward kinmetics end-effector position solver
    std::shared_ptr<KDL::TreeJntToJacSolver>         fk_solver_jac_; // forward kinematics end-effector jacobian solver
    //std::shared_ptr<KDL::ChainJntToJacDotSolver>     fk_solver_jac_dot_; // forward kinematics end-effector time derivative of jacobian solver

    ros::Publisher                        marker_array_pub_;
    ros::Publisher                        ee_pose_pub_;
    ros::Publisher                        blahblahblah_pub_;
    ros::Subscriber                       trajectory_subscriber_;
    ros::Subscriber                       contact_sensor_subscriber_;
    std::mutex                            resource_mutex_;
  };

}

#endif
