#include <ros/ros.h>
#include <ros/node_handle.h>

#include <finite_state_machine.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  ros::Rate loop_rate(2000);

  std::string urdf_file = "/home/hiqp2016/ros_workspace/src/universal_robot/ur_description/urdf/ur10_robot_generated.urdf";
  std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

  FiniteStateMachine fsm(nh, "admittance_controller");
  fsm.setJointNames(joint_names);
  fsm.setStartConfigurations(urdf_file, "world", "ee_link");
  fsm.setTrajectoryParams(20, 0.0, 0.35);

  bool run = true;
  while (ros::ok() && run) {
    if (fsm.update() != 0)
      run = false;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
