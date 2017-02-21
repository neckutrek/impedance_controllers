/*! \file trajectory_publisher_node.cpp
 *  \author Marcus A Johansson
 *  \brief A ROS node that generates position, velocity and acceleration for a trajectory and publishes it on a ROS topic.
 */

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <compliance_controllers_msgs/ReloadTrajectoryFrame.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>

using compliance_controllers_msgs::ReloadTrajectoryFrame;

class TrajectoryPublisher {
public:
  TrajectoryPublisher()
  : loop_rate_(10) {}

  ~TrajectoryPublisher() noexcept {}

  void init(ros::NodeHandle &nh) {
    nh_ = nh;
    trajectory_publisher_ = nh_.advertise<ReloadTrajectoryFrame>("impedance_controller/trajectory_frame", 1000);
  }

  void runOnce() {
    double t_start = ros::Time::now().toSec();
    double t = 0.0;
    while (ros::ok()) {
      t = ros::Time::now().toSec() - t_start;
      ReloadTrajectoryFrame rtf;
      rtf.position = getPosition(t);
      rtf.velocity = getVelocity(t);
      rtf.acceleration = getAcceleration(t);
      rtf.orientation = getOrientation(t);
      rtf.angular_velocity = getAngularVelocity(t);
      rtf.angular_acceleration = getAngularAcceleration(t);
      trajectory_publisher_.publish(rtf);

      ros::spinOnce();
      loop_rate_.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher trajectory_publisher_;
  ros::Rate loop_rate_;

  double T = 10;
  double min_ = -0.2;
  double max_ = 0.2;

  std::vector<double> getPosition(double t) {
    //t=1;
    double val1 = (std::sin(2*3.1415/T * t)+1)/2*(max_-min_)+min_;
    double val2 = (std::cos(2*3.1415/T * t)+1)/2*(max_-min_)+min_;
    //val = 0;
    //return {1.2, 0, 1.5};
    //return {1.2+val2, 0, 1.7};
    //return {val1, 0.1, 2+val2};
    return {0.8+val1, 0.0, 1.2}; // use this for ur10
  }

  std::vector<double> getVelocity(double t) {
    //t=1;
    double dval1 = 3.1415/T*(max_-min_)*std::cos(2*3.1415/T * t);
    double dval2 = -3.1415/T*(max_-min_)*std::sin(2*3.1415/T * t);
    //dval = 0;
    //return {dval2, 0, 0};
    //return {dval1, 0, dval2};
    //return {0, 0, 0};
    return {dval1, 0, 0}; // use this for ur10
  }

  std::vector<double> getAcceleration(double t) {
    //t=1;
    double ddval1 = 2*3.1415*3.1415/(T*T)*(max_-min_)*std::sin(2*3.1415/T * t);
    double ddval2 = -2*3.1415*3.1415/(T*T)*(max_-min_)*std::cos(2*3.1415/T * t);
    //ddval = 0;
    //return {ddval2, 0, 0};
    //return {ddval1, 0, ddval2};
    //return {0, 0, 0};
    return {ddval1, 0, 0}; // use this for ur10
  }

  std::vector<double> getOrientation(double t) {
    double angle = 0.5*T/(2*3.1415)*(1-std::cos(2*3.1415/T*t));
    double sin = std::sin( angle-1.57079632679 );
    double cos = std::cos( angle-1.57079632679 );
    
     // return {cos, 0, sin,
     //         0, 1, 0,
     //         -sin, 0, cos};

    return {1, 0, 0,
           0, 1, 0,
           0, 0, 1};
  }

  std::vector<double> getAngularVelocity(double t) {
    double val = 0.5*std::sin(2*3.1415/T*t);

    //return {0, val, 0};
    return {0, 0, 0};
    //return {0, val, 0}; // use this for ur10
  }

  std::vector<double> getAngularAcceleration(double t) {
    double dval = 0.5*2*3.1415/T*std::cos(2*3.1415/T*t);

    return {0, 0, 0};
    //return {0, dval, 0};
    //return {0, dval, 0}; // use this for ur10
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_publisher");
  ros::NodeHandle nh;

  TrajectoryPublisher tp;
  tp.init(nh);
  tp.runOnce();

  return 0;
}
