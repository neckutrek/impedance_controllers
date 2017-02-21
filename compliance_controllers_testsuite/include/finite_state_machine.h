#include <mutex>

#include <ros/ros.h>

#include <start_pose_generator.h>
#include <trajectory_pose_publisher.h>

#include <std_msgs/Bool.h>
#include <compliance_controllers_msgs/ReloadContactState.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>

#include <Eigen/Dense>

class FiniteStateMachine {
public:
  FiniteStateMachine(ros::NodeHandle& nh, std::string controller_name);
  ~FiniteStateMachine() = default;

  void setJointNames(std::vector<std::string> joint_names);
  void setStartConfigurations(std::string urdf_file, std::string root_link, std::string tip_link);
  void setTrajectoryParams(double T, double min, double max);

  int update();

private:
  enum State {STARTUP_FSM, INITIALIZE_ROBOT, GOTO_START_POSE, FOLLOW_TRACK_FRW, CONVERGE_AT_END_POSE, FOLLOW_TRACK_BCK, WRITE_TO_FILE, RESTART};

  void publishPoseMarkers(KDL::Frame pose);

  void contactSensorSubscriberCallback(const compliance_controllers_msgs::ReloadContactState& msg);

  void startupFsm();
  void initializeRobot();
  void gotoStartPose();
  void followTrackForward();
  void convergeAtEndPose();
  void followTrackBackwards();
  void writeToFile();
  int restart();

  void calculateRelativeEeError();

  State                                     state_;

  std::string                               controller_name_;
  std::string                               root_link_;

  ros::Publisher                            marker_array_pub_;
  ros::Publisher                            force_sensor_calibration_pub_;

  ros::Time                                 initialize_robot_time_;
  ros::Time                                 goto_start_time_;
  ros::Time                                 start_fwd_track_time_;
  ros::Time                                 start_back_track_time_;
  ros::Time                                 start_converging_time_;
  ros::Time                                 start_rtt_time_;

  double                                    round_trip_time_;
  double                                    convergence_time_;
  double                                    trajectory_cycling_time_;
  bool                                      convergence_timer_flag_;
  Eigen::Vector3d                           convergence_position_error_;
  Eigen::Quaternion<double>                 convergence_rotation_error_;

  

  std::vector<std::string>                  joint_names_;

  std::shared_ptr<StartPoseGenerator>       spg_;
  unsigned int                              start_pose_counter_;

  ros::ServiceClient                        switch_controller_client_;
  ros::ServiceClient                        load_controller_client_;

  std::shared_ptr<TrajectoryPosePublisher>  tpp_;

  std::mutex                                resource_mutex_;
  Eigen::VectorXd                           F_e_;
  Eigen::VectorXd                           F_e_max_;
  ros::Subscriber                           contact_sensor_subscriber_;

  tf::TransformListener                     tf_listener_;

};


