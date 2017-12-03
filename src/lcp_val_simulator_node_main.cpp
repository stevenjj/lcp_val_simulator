#include "ros/ros.h"

#include <lcp_val_simulator/lcp_dyn_simulator.hpp>
#include <lcp_val_simulator/val_rviz_translator.hpp>

#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "lcp_val_simulator_node");
  ros::NodeHandle n;

  // Simulation Joint State Publisher
  ros::Publisher sim_joint_state_pub = n.advertise<sensor_msgs::JointState>("val_lcp_robot/joint_states", 10);

  LCP_Dyn_Simulator lcp_simulator;
  Val_Rviz_Translator val_rviz_translator;  

  ros::Rate loop_rate(100);

  std::cout << "Simulator Running" << std::endl;

  sensor_msgs::JointState updated_joint_msg;
  while (ros::ok()){
    lcp_simulator.MakeOneStepUpdate();

    val_rviz_translator.populate_joint_state_msg(lcp_simulator.m_q, updated_joint_msg);

    sim_joint_state_pub.publish(updated_joint_msg);

    std::cout << "Torso Yaw Val Rviz Joint Index:" << ValJoints::torsoYaw << std::endl;
    std::cout << "Torso Yaw SJ Index" << val_rviz_translator.SJ_joint_index_to_name[12] << std::endl;


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}