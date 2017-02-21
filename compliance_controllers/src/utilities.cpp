#include "utilities.h"

int kdl_getAllQNrFromTree(const KDL::Tree& kdl_tree, std::vector<unsigned int>& qnrs) {
  qnrs.clear();
  for (auto&& element : kdl_tree.getSegments()) {
    qnrs.push_back(element.second.q_nr);
  }
}

std::string kdl_getJointNameFromQNr(const KDL::Tree& kdl_tree, unsigned int q_nr) {
  for (auto&& it : kdl_tree.getSegments()) {
    if (it.second.q_nr == q_nr) {
      return it.second.segment.getName();
    }
  }
  return "";
}

int kdl_getQNrFromJointName(const KDL::Tree& kdl_tree, const std::string& joint_name) {
  for (auto&& it : kdl_tree.getSegments()) {
    if (it.second.segment.getJoint().getName().compare(joint_name) == 0) {
      return it.second.q_nr;
    }
  }
  return -1;
}

Eigen::Quaterniond getQuaternionZYX(double az, double ay, double ax) {
  Eigen::AngleAxisd axisx(ax, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd axisy(ay, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd axisz(az, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = axisy * axisz * axisx;
  return q;
}