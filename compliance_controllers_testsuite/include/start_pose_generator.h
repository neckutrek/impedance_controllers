#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include <ros/ros.h>

#include <kdl/tree.hpp>
#include <kdl/chainiksolver.hpp>

/*! \brief Generates starting joint configurations from specifying a grid of end-effector 6D poses. */
class StartPoseGenerator {
public:
  StartPoseGenerator(ros::NodeHandle& nh) : nh_(nh) {};
  ~StartPoseGenerator() = default;

  /*! \brief sets the necessary parameters */
  void init(std::string urdf_file, std::string root_link, std::string tip_link);

  /*! \brief Sets the nominal position of the end-effector */
  inline void setNominalPosition(double x, double y, double z) {
    pose_grid_.nominal_pos_x = x;
    pose_grid_.nominal_pos_y = y;
    pose_grid_.nominal_pos_z = z;
  }

  /*! \brief Sets the margins around the nominal position (i.e. the size of the grid) */
  inline void setPositionMargins(double dx, double dy, double dz) {
    pose_grid_.pos_dim_x = dx;
    pose_grid_.pos_dim_y = dy;
    pose_grid_.pos_dim_z = dz;
  }

  /*! \brief Sets the total margin in degrees for rotation around x and y axis, also sets the set-point density */
  inline void setOrientationMargins(double ax, double ay, double nx, double ny) {
    pose_grid_.rot_dim_x = ax * M_PI / 180.0;
    pose_grid_.rot_dim_y = ay * M_PI / 180.0;
    pose_grid_.rot_density_x = nx;
    pose_grid_.rot_density_y = ny;
  }

  /*! \brief Sets how many points the grids have per dimension */
  inline void setPositionDensity(double x, double y, double z) {
    pose_grid_.pos_density_x = x;
    pose_grid_.pos_density_y = y;
    pose_grid_.pos_density_z = z;
  }

  //void setOrientationDensity(double x, double y, double z);

  /*! \brief Computes all joint configurations for all end-effector starting poses */
  int computeAllJntConfigurations();

  /*! \brief Gets the total number of joint configurations */
  unsigned int getNJntConfigurations();

  /*! \brief Returns the i'th joint configuration */
  KDL::JntArray getJntConfiguration(unsigned int i);

  inline unsigned int getNPoses() {
    return ee_start_poses_.size();
  }

  /*! \brief Gets the i'th pose */
  KDL::Frame getPose(unsigned int i);

private:
  struct Grid {
    double nominal_pos_x;
    double nominal_pos_y;
    double nominal_pos_z;

    double pos_dim_x;
    double pos_dim_y;
    double pos_dim_z;

    double rot_dim_x;
    double rot_dim_y;
    double rot_density_x;
    double rot_density_y;

    double pos_density_x;
    double pos_density_y;
    double pos_density_z;
  };

  void generatePoses();
  void generateConfigurations();

  ros::NodeHandle&                        nh_;

  std::string                             root_link_;
  std::string                             tip_link_;

  KDL::Tree                               tree_;
  KDL::Chain                              chain_;
  std::shared_ptr<KDL::ChainIkSolverPos>  ik_solver_;

  std::vector<KDL::Frame>                 ee_start_poses_;
  std::vector<KDL::JntArray>              start_configurations_;

  Grid                                    pose_grid_;

};