#ifndef UTILITIES_H
#define UTILITIES_H

#include <kdl/tree.hpp>
#include <Eigen/Dense>
#include <iostream>

/*! \brief Calculates the Moore-Penrose Pseudoinverse for any sized matrices.
 *  \author http://eigendobetter.com/ (edited by Marcus A Johansson) */
template<typename Derived>
Derived pinv(const Eigen::MatrixBase<Derived>& a)
{
  typedef typename Eigen::MatrixBase<Derived>::RealScalar RealScalar;
  if (a.rows() < a.cols())
  {
    auto svd = a.derived().transpose().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    RealScalar tolerance = (RealScalar)std::numeric_limits<RealScalar>::epsilon() * std::max((RealScalar)a.cols(), (RealScalar)a.rows()) * svd.singularValues().array().abs().maxCoeff();
    return (svd.matrixV() * Derived((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal() * svd.matrixU().adjoint()).transpose();
  }
  Eigen::JacobiSVD<Derived> svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  RealScalar tolerance = (RealScalar)std::numeric_limits<RealScalar>::epsilon() * std::max((RealScalar)a.cols(), (RealScalar)a.rows()) * svd.singularValues().array().abs().maxCoeff();
  return svd.matrixV() * Derived((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal() * svd.matrixU().adjoint();
}

/*! \brief Computes the damped pseudo inverse that penalizes solutions with large joint velocities.
 *         J_p^(-1)=J^T(J*J^T+\rho^2*I)^(-1)
 *  \author Marcus A Johansson */
template<typename Derived>
Eigen::MatrixXd dampedPinv(const Eigen::MatrixBase<Derived>& a, double rho = 1e-4) {
  // Eigen::MatrixXd b = Eigen::MatrixXd::Zero(a.rows(),a.cols());
  // std::cout << "before\n";
  // b = a.transpose() * (a*a.transpose() + rho*rho*Eigen::MatrixBase<Derived>::Identity(a.rows(), a.rows()) ).inverse();
  // std::cout << "after\n";
  // Derived c = b;
  // std::cout << "after\n";
  return a.transpose() * (a*a.transpose() + rho*rho*Eigen::MatrixBase<Derived>::Identity(a.rows(), a.rows()) ).inverse();
}

int kdl_getAllQNrFromTree(const KDL::Tree& kdl_tree, std::vector<unsigned int>& qnrs);

std::string kdl_getJointNameFromQNr(const KDL::Tree& kdl_tree, unsigned int q_nr);

/*! \brief Gets the q-number (used in KDL for identifying joints) from a joint name.
 *  \author Marcus A Johansson */
int kdl_getQNrFromJointName(const KDL::Tree& kdl_tree, const std::string& joint_name);

Eigen::Quaterniond getQuaternionZYX(double az, double ay, double ax);

#endif