#include "ros/ros.h"

#include <lcp_val_simulator/lcp_dyn_simulator.hpp>
#include <lcp_val_simulator/val_rviz_translator.hpp>

#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "lcp_val_simulator_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(1000);

  // Transform broadcaster
  tf::TransformBroadcaster      br;
  // Simulation Joint State Publisher
  ros::Publisher sim_joint_state_pub = n.advertise<sensor_msgs::JointState>("val_lcp_robot/joint_states", 10);



  LCP_Dyn_Simulator lcp_simulator;
  Val_Rviz_Translator val_rviz_translator;  

  std::cout << "Simulator Running" << std::endl;

  sensor_msgs::JointState updated_joint_msg; 
  tf::Transform           update_world_to_pelvis_transform;
  while (ros::ok()){
    lcp_simulator.MakeOneStepUpdate();

    // Translate To RVIZ joints
    val_rviz_translator.populate_joint_state_msg(lcp_simulator.m_q, update_world_to_pelvis_transform, updated_joint_msg);

    // Publish to Rviz
    sim_joint_state_pub.publish(updated_joint_msg);
    br.sendTransform(tf::StampedTransform(update_world_to_pelvis_transform, ros::Time::now(), "world",  "val_robot/pelvis"));

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}