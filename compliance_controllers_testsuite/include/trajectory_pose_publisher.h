#include <mutex>

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <kdl/frames.hpp>

#include <compliance_controllers_msgs/ReloadTrajectoryFrame.h>
#include <compliance_controllers_msgs/ReloadEEPose.h>

class TrajectoryPosePublisher {
public:
  TrajectoryPosePublisher(ros::NodeHandle &nh, std::string controller_name);
  ~TrajectoryPosePublisher() noexcept = default;

  void init(double T, double min, double max);
  void setStartPose(KDL::Frame pose);

  void publishStartPose();
  void publishFrwTrackPose(double elapsed_time);
  void publishEndPose();
  void publishBackTrackPose(double elapsed_time);

  inline double getTrajectoryCycleTime() { return T_; }
  inline double getTrajectoryDistance() { return max_ - min_; }

  bool reachedStartPose();
  bool eeHasConverged();
  inline bool finalTimeReached(double elapsed_time) {
    return (elapsed_time >= T_);
  }

  void eeStateCallback(const compliance_controllers_msgs::ReloadEEPose& msg);

private:
  ros::NodeHandle   nh_;
  ros::Publisher    trajectory_publisher_;
  ros::Subscriber   ee_state_subscriber_;

  std::string       controller_name_;

  double            T_;
  double            min_;
  double            max_;

  std::mutex        resource_mutex_;

  KDL::Frame        start_pose_;
  KDL::Frame        current_ee_pose_;
  KDL::Vector       current_ee_vel_;
  KDL::Vector       current_ee_ang_vel_;
  KDL::Frame        end_pose_;

  std::vector<double> getPosition(double t);

  std::vector<double> getVelocity(double t);

  std::vector<double> getAcceleration(double t);

  std::vector<double> getOrientation(double t);

  std::vector<double> getAngularVelocity(double t);

  std::vector<double> getAngularAcceleration(double t);

};