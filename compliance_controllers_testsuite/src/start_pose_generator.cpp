#include <start_pose_generator.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

void StartPoseGenerator::init(std::string urdf_file, std::string root_link, std::string tip_link) {
  root_link_ = root_link;
  tip_link_ = tip_link;

  std::string robot_description_xml;
  nh_.param("robot_description", robot_description_xml, std::string());
  if (!kdl_parser::treeFromString(robot_description_xml, tree_)) {
    std::cerr << "Failed to construct kdl tree from robot_description\n";
  }

  // if (!kdl_parser::treeFromFile(urdf_file, tree_)) {
  //   std::cerr << "Failed to construct kdl tree\n";
  // }

  if (!tree_.getChain(root_link_, tip_link_, chain_)) {
    std::cerr << "Failed to construct kdl chain\n";
  }

  ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
  if (!ik_solver_) {
    std::cerr << "Failed to construct kdl ik position solver\n";
  }
}

int StartPoseGenerator::computeAllJntConfigurations() {
  generatePoses();
  generateConfigurations();
}

unsigned int StartPoseGenerator::getNJntConfigurations() {
  return start_configurations_.size();
}

KDL::JntArray StartPoseGenerator::getJntConfiguration(unsigned int i) {
  return start_configurations_.at(i);
}

KDL::Frame StartPoseGenerator::getPose(unsigned int i) {
  return ee_start_poses_.at(i);
}

void StartPoseGenerator::generatePoses() {
  for (unsigned int i=0; i<pose_grid_.pos_density_x; ++i) {
    double kx = 0.5;
    if (pose_grid_.pos_density_x > 1) {
      kx = static_cast<double>(i)/(static_cast<double>(pose_grid_.pos_density_x)-1.0);
    }
    double x = pose_grid_.nominal_pos_x + pose_grid_.pos_dim_x * (kx-0.5);

    for (unsigned int j=0; j<pose_grid_.pos_density_y; ++j) {
      double ky = 0.5;
      if (pose_grid_.pos_density_y > 1) {
        ky = static_cast<double>(j)/(static_cast<double>(pose_grid_.pos_density_y)-1.0);
      }
      double y = pose_grid_.nominal_pos_y + pose_grid_.pos_dim_y * (ky-0.5);

      for (unsigned int k=0; k<pose_grid_.pos_density_z; ++k) {
        double kz = 0.5;
        if (pose_grid_.pos_density_z > 1) {
          kz = static_cast<double>(k)/(static_cast<double>(pose_grid_.pos_density_z)-1.0);
        }
        double z = pose_grid_.nominal_pos_z + pose_grid_.pos_dim_z * (kz-0.5);

        for (unsigned int u=0; u<pose_grid_.rot_density_x; ++u) {
          double ku = 0.5;
          if (pose_grid_.rot_density_x > 1) {
            ku = static_cast<double>(u)/(static_cast<double>(pose_grid_.rot_density_x)-1.0);
          }
          double uu = pose_grid_.rot_dim_x * (ku-0.5);

          for (unsigned int v=0; v<pose_grid_.rot_density_y; ++v) {
            double kv = 0.5;
            if (pose_grid_.rot_density_y > 1) {
              kv = static_cast<double>(v)/(static_cast<double>(pose_grid_.rot_density_y)-1.0);
            }
            double vv = pose_grid_.rot_dim_y * (kv-0.5);

            KDL::Rotation rot = KDL::Rotation::Identity() * KDL::Rotation::RotX(uu) * KDL::Rotation::RotY(vv);

            KDL::Vector pos(x, y, z);

            KDL::Frame pose(rot, pos);

            ee_start_poses_.push_back(pose);
          }
        }
      }
    }
  }
}

void StartPoseGenerator::generateConfigurations() {
  KDL::JntArray initial_guess(chain_.getNrOfJoints());
  
  for (const KDL::Frame& pose : ee_start_poses_) {
    KDL::JntArray solution(chain_.getNrOfJoints());
    ik_solver_->CartToJnt(initial_guess, pose, solution);
    start_configurations_.push_back(solution);
  }
}


