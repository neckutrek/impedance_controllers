/*! \file trajectory_frame.h
 *  \author Marcus A Johansson */

#ifndef TRAJECTORY_FRAME_H
#define TRAJECTORY_FRAME_H

namespace irb4400 {

  class TrajectoryFrame {
  public:
    TrajectoryFrame() noexcept {
      r_ = Eigen::Vector3d::Zero();
      dr_ = Eigen::Vector3d::Zero();
      ddr_ = Eigen::Vector3d::Zero();
      R_ = Eigen::Matrix3d::Zero();
      w_ = Eigen::Vector3d::Zero();
      a_ = Eigen::Vector3d::Zero();
    }

    ~TrajectoryFrame() noexcept = default;

    TrajectoryFrame(const TrajectoryFrame& tf) noexcept {
      r_ = tf.r_;
      dr_ = tf.dr_;
      ddr_ = tf.ddr_;

      R_ = tf.R_;
      w_ = tf.w_;
      a_ = tf.a_;
    }

    Eigen::Vector3d r_;   // translation w/ repsect to world frame
    Eigen::Vector3d dr_;  // translational velocity
    Eigen::Vector3d ddr_; // translational acceleration

    Eigen::Matrix3d R_; // orientation w/ respect to world frame
    Eigen::Vector3d w_; // instantaneous angular velocity
    Eigen::Vector3d a_; // instantaneous angular acceleration
  };

} // namespace irb4400

#endif // include guard
