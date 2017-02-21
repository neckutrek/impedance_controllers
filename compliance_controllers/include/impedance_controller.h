#ifndef IMPEDANCE_VELOCITY_CONTROLLER_H
#define IMPEDANCE_VELOCITY_CONTROLLER_H

#include <mutex>

#include "base_controller.h"
#include "trajectory_frame.h"

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <gazebo_msgs/ContactsState.h>

#include <kdl/jacobian.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include "chainjnttojacdotsolver.hpp"
#include <kdl/chaindynparam.hpp>

#include <compliance_controllers_msgs/ReloadTrajectoryFrame.h>
#include <compliance_controllers_msgs/ReloadContactState.h>
#include <compliance_controllers_msgs/ReloadEEPose.h>

namespace irb4400 {

  typedef 
  controller_interface::Controller<hardware_interface::EffortJointInterface>
  JointEffortController;

  typedef 
  hardware_interface::EffortJointInterface 
  JointEffortInterface;

  /*! \brief A joint effort controller that mimics a mass-spring-damper system following a trajectory.
   *  \author Marcus A Johansson */
  class ImpedanceController : public BaseController<JointEffortController, JointEffortInterface> {
  public:
    ImpedanceController() = default;

    ~ImpedanceController() noexcept = default;

    void initialize();

    void computeControls(Eigen::VectorXd& u);

  private:
    ImpedanceController(const ImpedanceController& other) = delete;
    ImpedanceController(ImpedanceController&& other) = delete;
    ImpedanceController& operator=(const ImpedanceController& other) = delete;
    ImpedanceController& operator=(ImpedanceController&& other) noexcept = delete;

    void trajectorySubscriberCallback(const compliance_controllers_msgs::ReloadTrajectoryFrame& msg);
    void contactSensorSubscriberCallback(const compliance_controllers_msgs::ReloadContactState& msg);

    void computeForwardKinematics();
    void computeDdq(); // computes desired joint accelerations
    void computeTau(); // computes desired joint torques using the internal robot model
    void updateMSD(); // computes the pos/vel/acc of the msd system

    void renderEndEffectorAndTrajectoryPoint();
    void publishEEPose();

    std::string                           root_link_name_;
    std::string                           ee_link_name_;
    std::string                           tip_link_name_;
    KDL::Chain                            kdl_ee_chain_;
    KDL::Chain                            kdl_tip_chain_;

    Eigen::VectorXd                       initial_joint_configuration_;

    TrajectoryFrame                       tf_; // the desired trajectory got externally
    //TrajectoryFrame                       msd_; // the trajectory of the mass-spring-damper
    TrajectoryFrame                       ee_; // the trajectory of the actual end effector

    Eigen::VectorXd                       ddr_; // the desired 6D end-effector pose
    Eigen::VectorXd                       dq_; // the desired joint velocities
    Eigen::VectorXd                       ddq_; // the desired joint accelerations
    Eigen::VectorXd                       tau_u_; // the desired joint torques (the controls)

    Eigen::MatrixXd                       Md_; // mass of the mass-spring-damper system
    Eigen::MatrixXd                       Kd_; // damping constant of the mass-spring-damper system
    Eigen::MatrixXd                       Kp_; // spring constant of the mass-spring-damper system

    double                                damp_coeff_; // the damped pseudo-inverse damping coefficient

    Eigen::VectorXd                       F_e_; // the current external forces sensed by the contact sensor
    Eigen::VectorXd                       tau_e_; // the external torques on each joint due to the external forces in F_e_

    KDL::Frame                            ee_pose_; // end-effector pose
    KDL::Jacobian                         ee_jacobian_; // end-effector jacobian
    KDL::Jacobian                         ee_dot_jacobian_; // end-effector time derivative of jacobian
    Eigen::MatrixXd                       ee_jacobian_pinv_; // pseudo inverse of translation jacobian

    KDL::JntSpaceInertiaMatrix            H_; // inertia matrix
    KDL::JntArray                         Cq_; // centrifugal and coriolis torques
    KDL::JntArray                         g_; // gravity torques

    std::shared_ptr<KDL::TreeFkSolverPos_recursive>  fk_solver_pos_; // forward kinmetics end-effector position solver
    std::shared_ptr<KDL::TreeJntToJacSolver>         fk_solver_jac_; // forward kinematics end-effector jacobian solver
    std::shared_ptr<KDL::ChainJntToJacDotSolver>     fk_solver_jac_dot_; // forward kinematics end-effector time derivative of jacobian solver
    std::shared_ptr<KDL::ChainDynParam>              id_solver_; // inverted dynamics solver

    ros::Publisher                        marker_array_pub_;
    ros::Publisher                        ee_pose_pub_;
    ros::Subscriber                       trajectory_subscriber_;
    ros::Subscriber                       contact_sensor_subscriber_;
    std::mutex                            resource_mutex_;
  };

}

#endif
