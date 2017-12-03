#include "ros/ros.h"

#include <lcp_val_simulator/lcp_dyn_simulator.hpp>
#include <lcp_val_simulator/val_rviz_translator.hpp>

#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "lcp_val_simulator_node");
  ros::NodeHandle n;

  LCP_Dyn_Simulator lcp_simulator;
  Val_Rviz_Translator val_rviz_translator;  

  ros::Rate loop_rate(10);

  std::cout << "Simulator Running" << std::endl;
  while (ros::ok()){
    lcp_simulator.MakeOneStepUpdate();

    std::cout << "Torso Yaw Val Rviz Joint Index:" << ValJoints::torsoYaw << std::endl;
    std::cout << "Torso Yaw SJ Index" << val_rviz_translator.SJ_joint_index_to_name[12] << std::endl;


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}